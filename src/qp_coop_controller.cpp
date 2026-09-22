#include "dual_arm/qp_coop_controller.hpp"

#include "dual_arm/coop_kinematics.hpp"

#include <array>
#include <limits>
#include <stdexcept>

namespace dual_arm
{

QpCoopController::QpCoopController(
    std::shared_ptr<RobotModel> model,
    std::shared_ptr<const ObjectTrajectory> trajectory,
    const CoopConfig& coop_config,
    const TorqueQpConfig& qp_config,
    const CollisionConfig& collision_config,
    double timestep)
    : Controller(model),
      nominal_controller_(model, trajectory, coop_config),
      trajectory_(std::move(trajectory)),
      qp_config_(qp_config),
      timestep_(timestep),
      qp_(qp_config)
{
  if (!(timestep_ > 0.0))
  {
    throw std::invalid_argument("QpCoopController: timestep must be positive");
  }
  if (scene().relativeDofBetweenHands() != 0)
  {
    throw std::invalid_argument("QpCoopController requires zero relative DOF between the hands");
  }
  for (const ObstaclePairSpec& pair : scene().obstacles)
  {
    if (pair.safe_distance >= qp_config_.collision_influence_distance)
    {
      throw std::invalid_argument(
          "QpCoopController: pair safe distance must be smaller than collision influence distance");
    }
  }

  if (qp_config_.collision_avoidance_enabled && !scene().obstacles.empty())
  {
    CollisionConfig controller_collision_config = collision_config;
    controller_collision_config.margin = std::max(
        qp_config_.collision_influence_distance,
        qp_config_.reference_governor.enabled
            ? qp_config_.reference_governor.trigger_distance
            : 0.0);
    controller_collision_config.threads = qp_config_.collision_threads;
    collision_model_ = std::make_unique<CollisionModel>(
        scene(),
        std::array<Pose, kNumArms>{
            model_->basePose(Arm::Left),
            model_->basePose(Arm::Right)},
        controller_collision_config,
        timestep_);
  }

  for (Arm arm : kArms)
  {
    const int offset = armIndex(arm) * kArmDof;
    const Vector7d safe_limit =
        model_->torqueLimit(arm).array() - qp_config_.torque_margin;
    if ((safe_limit.array() <= 0.0).any())
    {
      throw std::invalid_argument("QpCoopController: torque margin exceeds actuator limit");
    }
    lower_bound_.segment<kArmDof>(offset) = -safe_limit;
    upper_bound_.segment<kArmDof>(offset) = safe_limit;
    position_lower_.segment<kArmDof>(offset) = model_->jointLowerLimit(arm);
    position_upper_.segment<kArmDof>(offset) = model_->jointUpperLimit(arm);
  }
}

const char* QpCoopController::governorPhaseName(GovernorPhase phase)
{
  switch (phase)
  {
    case GovernorPhase::Normal: return "NORMAL";
    case GovernorPhase::Lift: return "LIFT";
    case GovernorPhase::Cross: return "CROSS";
    case GovernorPhase::Descend: return "DESCEND";
  }
  return "UNKNOWN";
}

void QpCoopController::reset(const DualArmState& initial_state)
{
  nominal_controller_.reset(initial_state);
  Vector14d initial_torque;
  initial_torque.head<kArmDof>() = initial_state.arm(Arm::Left).tau;
  initial_torque.tail<kArmDof>() = initial_state.arm(Arm::Right).tau;
  qp_.reset(initial_torque);
  closed_chain_acceleration_residual_ = 0.0;
  active_collision_constraints_ = 0;
  collision_min_distance_ = collision_model_ ? collision_model_->margin() : 0.0;
  governor_phase_ = GovernorPhase::Normal;
  governor_offset_ = 0.0;
  governor_virtual_time_ = initial_state.t;
  governor_entry_normal_.setZero();
  governed_reference_ = trajectory_->evaluate(governor_virtual_time_);
}

std::pair<Vector7d, Vector7d> QpCoopController::compute(
    const DualArmState& state,
    double t)
{
  (void)t;
  const std::vector<DistanceInfo>* collision_distances = nullptr;
  collision_min_distance_ = collision_model_ ? collision_model_->margin() : 0.0;
  if (collision_model_)
  {
    collision_distances = &collision_model_->query(
        state.arm(Arm::Left).q,
        state.arm(Arm::Right).q);
    collision_min_distance_ = collision_model_->minDistance();
  }

  governed_reference_ = trajectory_->evaluate(governor_virtual_time_);
  bool freeze_trajectory = false;
  double offset_rate = 0.0;
  if (qp_config_.reference_governor.enabled && collision_model_)
  {
    const auto& governor = qp_config_.reference_governor;
    double obstacle_distance = collision_model_->margin();
    Vector3d obstacle_normal = Vector3d::Zero();
    if (collision_distances)
    {
      for (const DistanceInfo& info : *collision_distances)
      {
        if (collision_model_->groupName(info.group) == governor.obstacle_group &&
            info.distance < obstacle_distance)
        {
          obstacle_distance = info.distance;
          obstacle_normal = info.normal;
        }
      }
    }

    // obstacle pair 必须写成 [object, obstacle]：normal 从物体指向障碍物。
    // n·v_ref > 0 表示物体参考速度正在沿法向接近障碍物。
    const bool approaching =
        obstacle_normal.dot(governed_reference_.twist.head<3>()) > 1e-4;
    if (governor_phase_ == GovernorPhase::Normal &&
        obstacle_distance < governor.trigger_distance && approaching)
    {
      governor_phase_ = GovernorPhase::Lift;
      governor_entry_normal_ = obstacle_normal;
    }

    if (governor_phase_ == GovernorPhase::Lift)
    {
      freeze_trajectory = true;
      const double next = std::min(
          governor.avoidance_offset,
          governor_offset_ + governor.offset_speed * timestep_);
      offset_rate = (next - governor_offset_) / timestep_;
      governor_offset_ = next;
      if (governor_offset_ >= governor.avoidance_offset - 1e-12)
      {
        governor_phase_ = GovernorPhase::Cross;
      }
    }
    else if (governor_phase_ == GovernorPhase::Cross)
    {
      const bool passed_obstacle =
          governor_entry_normal_.squaredNorm() > 0.5 &&
          obstacle_normal.dot(governor_entry_normal_) < -0.2;
      if (passed_obstacle && obstacle_distance > governor.release_distance)
      {
        governor_phase_ = GovernorPhase::Descend;
      }
    }
    else if (governor_phase_ == GovernorPhase::Descend)
    {
      const double next = std::max(
          0.0,
          governor_offset_ - governor.offset_speed * timestep_);
      offset_rate = (next - governor_offset_) / timestep_;
      governor_offset_ = next;
      if (governor_offset_ <= 1e-12)
      {
        governor_phase_ = GovernorPhase::Normal;
      }
    }

    governed_reference_.pose.p +=
        governor_offset_ * governor.preferred_direction;
    if (freeze_trajectory)
    {
      governed_reference_.twist.setZero();
      governed_reference_.accel.setZero();
    }
    governed_reference_.twist.head<3>() +=
        offset_rate * governor.preferred_direction;
  }

  if (!freeze_trajectory)
  {
    governor_virtual_time_ += timestep_;
  }

  const auto [nominal_left, nominal_right] =
      nominal_controller_.computeWithReference(state, governed_reference_);

  Vector14d desired_torque;
  desired_torque.head<kArmDof>() = nominal_left;
  desired_torque.tail<kArmDof>() = nominal_right;

  Matrix14d mass_matrix = Matrix14d::Zero();
  Vector14d bias;
  Vector14d position;
  Vector14d velocity;
  for (Arm arm : kArms)
  {
    const int offset = armIndex(arm) * kArmDof;
    mass_matrix.block<kArmDof, kArmDof>(offset, offset) =
        model_->massMatrix(arm);
    bias.segment<kArmDof>(offset) = model_->bias(arm);
    position.segment<kArmDof>(offset) = state.arm(arm).q;
    velocity.segment<kArmDof>(offset) = state.arm(arm).dq;
  }

  const Pose left_pose = model_->eePose(Arm::Left);
  const Pose right_pose = model_->eePose(Arm::Right);
  const Vector3d r_left = state.object.pose.p - left_pose.p;
  const Vector3d r_right = state.object.pose.p - right_pose.p;
  const Matrix6x14d relative_jacobian = coop::relativeJacobian(
      model_->jacobian(Arm::Left),
      model_->jacobian(Arm::Right),
      r_left,
      r_right);

  auto accelerationAtObjectCenter = [&](Arm arm, const Vector3d& r)
  {
    Vector6d acceleration = model_->jacobianDotTimesQdot(arm);
    const Twist twist = model_->eeTwist(arm);
    const Vector3d omega = twist.tail<3>();
    const Vector3d alpha = acceleration.tail<3>();
    acceleration.head<3>() +=
        alpha.cross(r) + omega.cross(omega.cross(r));
    return acceleration;
  };

  const Vector6d relative_bias =
      accelerationAtObjectCenter(Arm::Right, r_right) -
      accelerationAtObjectCenter(Arm::Left, r_left);
  const Matrix6x14d acceleration_equality_matrix = relative_jacobian;
  const Vector6d acceleration_equality_target =
      -qp_config_.closed_chain_velocity_damping *
          (relative_jacobian * velocity) -
      relative_bias;

  JointSafetyTorqueQp::CollisionMatrix collision_matrix =
      JointSafetyTorqueQp::CollisionMatrix::Zero();
  JointSafetyTorqueQp::CollisionVector collision_lower =
      JointSafetyTorqueQp::CollisionVector::Zero();
  active_collision_constraints_ = 0;

  if (collision_model_ && collision_distances)
  {
    const auto& distances = *collision_distances;

    std::array<int, JointSafetyTorqueQp::kMaxCollisionConstraints>
        selected_indices;
    std::array<double, JointSafetyTorqueQp::kMaxCollisionConstraints>
        selected_risks;
    selected_indices.fill(-1);
    selected_risks.fill(std::numeric_limits<double>::infinity());

    const int capacity = std::min(
        qp_config_.collision_max_constraints,
        JointSafetyTorqueQp::kMaxCollisionConstraints);
    for (int index = 0; index < static_cast<int>(distances.size()); ++index)
    {
      const DistanceInfo& info = distances[index];
      const double pair_safe_distance =
          scene().obstacles[info.group].safe_distance >= 0.0
              ? scene().obstacles[info.group].safe_distance
              : qp_config_.collision_safe_distance;
      const double clearance = info.distance - pair_safe_distance;
      const double distance_rate = info.jacobian.dot(velocity);
      const double risk =
          clearance <= 0.0
              ? clearance
              : (distance_rate < -1e-6
                     ? clearance / (-distance_rate)
                     : std::numeric_limits<double>::infinity());
      if (info.distance >= qp_config_.collision_influence_distance ||
          info.jacobian.squaredNorm() < 1e-16)
      {
        continue;
      }
      for (int slot = 0; slot < capacity; ++slot)
      {
        if (risk >= selected_risks[slot])
        {
          continue;
        }
        for (int move = capacity - 1; move > slot; --move)
        {
          selected_risks[move] = selected_risks[move - 1];
          selected_indices[move] = selected_indices[move - 1];
        }
        selected_risks[slot] = risk;
        selected_indices[slot] = index;
        break;
      }
    }

    for (int slot = 0;
         slot < capacity && selected_indices[slot] >= 0;
         ++slot)
    {
      const DistanceInfo& info = distances[selected_indices[slot]];
      const double pair_safe_distance =
          scene().obstacles[info.group].safe_distance >= 0.0
              ? scene().obstacles[info.group].safe_distance
              : qp_config_.collision_safe_distance;
      const double distance_span =
          qp_config_.collision_influence_distance - pair_safe_distance;
      const double distance_rate = info.jacobian.dot(velocity);
      const double allowed_approach_rate =
          -qp_config_.collision_max_approach_speed *
          (info.distance - pair_safe_distance) /
          distance_span;
      collision_matrix.row(slot) = info.jacobian;
      collision_lower[slot] =
          (allowed_approach_rate - distance_rate) / timestep_;
      ++active_collision_constraints_;
    }
  }

  const Vector14d& solution = qp_.solve(
      desired_torque,
      lower_bound_,
      upper_bound_,
      mass_matrix,
      bias,
      position,
      velocity,
      position_lower_,
      position_upper_,
      acceleration_equality_matrix,
      acceleration_equality_target,
      collision_matrix,
      collision_lower,
      active_collision_constraints_);

  closed_chain_acceleration_residual_ =
      (acceleration_equality_matrix * qp_.predictedAcceleration() -
       acceleration_equality_target)
          .norm();

  return {
      solution.head<kArmDof>(),
      solution.tail<kArmDof>()};
}

} // namespace dual_arm
