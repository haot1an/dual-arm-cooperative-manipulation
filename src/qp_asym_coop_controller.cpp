#include "dual_arm/qp_asym_coop_controller.hpp"
#include "dual_arm/coop_kinematics.hpp"

#include <array>
#include <limits>
#include <stdexcept>

namespace dual_arm
{

  QpAsymmetricCoopController::QpAsymmetricCoopController(
      std::shared_ptr<RobotModel> model,
      std::shared_ptr<const ObjectTrajectory> trajectory,
      const AsymCoopConfig &asym_config,
      const TorqueQpConfig &qp_config,
      const CollisionConfig &collision_config,
      double timestep)
      : Controller(model),
        nominal_controller_(model, std::move(trajectory), asym_config),
        qp_config_(qp_config),
        timestep_(timestep),
        qp_(qp_config)
  {
    if (!(timestep_ > 0.0))
    {
      throw std::invalid_argument(
          "QpAsymmetricCoopController: timestep must be positive");
    }
    for (const ObstaclePairSpec &pair : scene().obstacles)
    {
      if (pair.safe_distance >=
          qp_config_.collision_influence_distance)
      {
        throw std::invalid_argument(
            "QpAsymmetricCoopController: pair safe distance must be smaller than collision influence distance");
      }
    }

    if (qp_config_.collision_avoidance_enabled &&
        !scene().obstacles.empty())
    {
      CollisionConfig controller_collision_config =
          collision_config;
      // 控制器只需要 influence distance 内的 pair；日志评估仍使用全局大 margin。
      controller_collision_config.margin =
          qp_config_.collision_influence_distance;
      controller_collision_config.threads =
          qp_config_.collision_threads;
      collision_model_ =
          std::make_unique<CollisionModel>(
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
          model_->torqueLimit(arm).array() -
          qp_config.torque_margin;

      if ((safe_limit.array() <= 0.0).any())
      {
        throw std::invalid_argument(
            "QpAsymmetricCoopController: torque margin exceeds actuator limit");
      }

      lower_bound_.segment<kArmDof>(offset) = -safe_limit;
      upper_bound_.segment<kArmDof>(offset) = safe_limit;
      position_lower_.segment<kArmDof>(offset) =
          model_->jointLowerLimit(arm);
      position_upper_.segment<kArmDof>(offset) =
          model_->jointUpperLimit(arm);
    }
  }

  void QpAsymmetricCoopController::reset(
      const DualArmState &initial_state)
  {
    nominal_controller_.reset(initial_state);

    Vector14d initial_torque;
    initial_torque.head<kArmDof>() =
        initial_state.arm(Arm::Left).tau;
    initial_torque.tail<kArmDof>() =
        initial_state.arm(Arm::Right).tau;
    qp_.reset(initial_torque);
    closed_chain_acceleration_residual_ = 0.0;
    active_collision_constraints_ = 0;
    collision_min_distance_ =
        collision_model_
            ? collision_model_->margin()
            : 0.0;
  }

  std::pair<Vector7d, Vector7d>
  QpAsymmetricCoopController::compute(
      const DualArmState &state,
      double t)
  {
    const auto [nominal_left, nominal_right] =
        nominal_controller_.compute(state, t);

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
      mass_matrix.block<kArmDof, kArmDof>(
          offset,
          offset) = model_->massMatrix(arm);
      bias.segment<kArmDof>(offset) =
          model_->bias(arm);
      position.segment<kArmDof>(offset) =
          state.arm(arm).q;
      velocity.segment<kArmDof>(offset) =
          state.arm(arm).dq;
    }

    const Pose left_pose =
        model_->eePose(Arm::Left);
    const Pose right_pose =
        model_->eePose(Arm::Right);
    const Vector3d r_left =
        state.object.pose.p - left_pose.p;
    const Vector3d r_right =
        state.object.pose.p - right_pose.p;

    const Matrix6x14d relative_jacobian =
        coop::relativeJacobian(
            model_->jacobian(Arm::Left),
            model_->jacobian(Arm::Right),
            r_left,
            r_right);

    auto accelerationAtObjectCenter =
        [&](Arm arm, const Vector3d &r)
    {
      Vector6d acceleration =
          model_->jacobianDotTimesQdot(arm);
      const Twist twist =
          model_->eeTwist(arm);
      const Vector3d omega =
          twist.tail<3>();
      const Vector3d alpha =
          acceleration.tail<3>();
      acceleration.head<3>() +=
          alpha.cross(r) +
          omega.cross(omega.cross(r));
      return acceleration;
    };

    const Vector6d relative_bias =
        accelerationAtObjectCenter(
            Arm::Right,
            r_right) -
        accelerationAtObjectCenter(
            Arm::Left,
            r_left);

    Matrix6d constraint_projector =
        Matrix6d::Identity();
    if (const ClosedChainConstraint *screw =
            scene().screwConstraint())
    {
      const Vector3d axis_world =
          state.object.pose.q *
          screw->axis.normalized();
      const Vector3d axis_point_world =
          state.object.pose.transformPoint(
              screw->point);
      const Twist screw_basis_at_object =
          coop::screwMotionBasisAtPoint(
              axis_world,
              axis_point_world,
              state.object.pose.p,
              screw->lead);
      constraint_projector =
          coop::screwConstraintProjector(
              screw_basis_at_object);
    }

    Matrix6x14d acceleration_equality_matrix;
    acceleration_equality_matrix.noalias() =
        constraint_projector *
        relative_jacobian;

    const Vector6d constrained_relative_velocity =
        constraint_projector *
        (relative_jacobian * velocity);
    const Vector6d acceleration_equality_target =
        -qp_config_.closed_chain_velocity_damping *
            constrained_relative_velocity -
        constraint_projector * relative_bias;

    JointSafetyTorqueQp::CollisionMatrix collision_matrix =
        JointSafetyTorqueQp::CollisionMatrix::Zero();
    JointSafetyTorqueQp::CollisionVector collision_lower =
        JointSafetyTorqueQp::CollisionVector::Zero();
    active_collision_constraints_ = 0;
    collision_min_distance_ =
        collision_model_
            ? collision_model_->margin()
            : 0.0;

    if (collision_model_)
    {
      const auto &distances =
          collision_model_->query(
              state.arm(Arm::Left).q,
              state.arm(Arm::Right).q);
      collision_min_distance_ =
          collision_model_->minDistance();

      std::array<int,
                 JointSafetyTorqueQp::kMaxCollisionConstraints>
          selected_indices;
      std::array<double,
                 JointSafetyTorqueQp::kMaxCollisionConstraints>
          selected_risks;
      selected_indices.fill(-1);
      selected_risks.fill(
          std::numeric_limits<double>::infinity());

      for (int index = 0;
           index < static_cast<int>(distances.size());
           ++index)
      {
        const DistanceInfo &info = distances[index];
        const double pair_safe_distance =
            scene().obstacles[info.group].safe_distance >= 0.0
                ? scene().obstacles[info.group].safe_distance
                : qp_config_.collision_safe_distance;
        const double clearance =
            info.distance - pair_safe_distance;
        const double distance_rate =
            info.jacobian.dot(velocity);
        const double risk =
            clearance <= 0.0
                ? clearance
                : (distance_rate < -1e-6
                       ? clearance / (-distance_rate)
                       : std::numeric_limits<double>::infinity());
        if (info.distance >=
                qp_config_.collision_influence_distance ||
            info.jacobian.squaredNorm() < 1e-16)
        {
          continue;
        }

        const int capacity =
            std::min(
                qp_config_.collision_max_constraints,
                JointSafetyTorqueQp::kMaxCollisionConstraints);
        for (int slot = 0; slot < capacity; ++slot)
        {
          if (risk >= selected_risks[slot])
          {
            continue;
          }
          for (int move = capacity - 1;
               move > slot;
               --move)
          {
            selected_risks[move] =
                selected_risks[move - 1];
            selected_indices[move] =
                selected_indices[move - 1];
          }
          selected_risks[slot] = risk;
          selected_indices[slot] = index;
          break;
        }
      }

      for (int slot = 0;
           slot < qp_config_.collision_max_constraints &&
           selected_indices[slot] >= 0;
           ++slot)
      {
        const DistanceInfo &info =
            distances[selected_indices[slot]];
        const double pair_safe_distance =
            scene().obstacles[info.group].safe_distance >= 0.0
                ? scene().obstacles[info.group].safe_distance
                : qp_config_.collision_safe_distance;
        const double distance_span =
            qp_config_.collision_influence_distance -
            pair_safe_distance;
        const double distance_rate =
            info.jacobian.dot(velocity);
        const double allowed_approach_rate =
            -qp_config_.collision_max_approach_speed *
            (info.distance - pair_safe_distance) /
            distance_span;

        collision_matrix.row(slot) =
            info.jacobian;
        collision_lower[slot] =
            (allowed_approach_rate - distance_rate) /
            timestep_;
        ++active_collision_constraints_;
      }
    }

    const Vector14d &solution =
        qp_.solve(
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
        (acceleration_equality_matrix *
             qp_.predictedAcceleration() -
         acceleration_equality_target)
            .norm();

    return {
        solution.head<kArmDof>(),
        solution.tail<kArmDof>()};
  }

} // namespace dual_arm
