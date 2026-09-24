#include "dual_arm/asym_coop_controller.hpp"
#include "dual_arm/coop_kinematics.hpp"
#include "dual_arm/math_utils.hpp"
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <stdexcept>

namespace dual_arm
{

  const char *AsymmetricCoopController::phaseName(Phase phase)
  {
    switch (phase)
    {
    case Phase::Preload:
      return "PRELOAD";
    case Phase::AngleTighten:
      return "ANGLE_TIGHTEN";
    case Phase::TorqueTighten:
      return "TORQUE_TIGHTEN";
    case Phase::Hold:
      return "HOLD";
    }
    return "UNKNOWN";
  }

  AsymmetricCoopController::AsymmetricCoopController(std::shared_ptr<RobotModel> model,
                                                     std::shared_ptr<const ObjectTrajectory> trajectory,
                                                     const AsymCoopConfig &params, bool contact_grasp)
      : Controller(std::move(model)), trajectory_(std::move(trajectory)), params_(params),
        contact_grasp_(contact_grasp)
  {
    if (!trajectory_)
      throw std::invalid_argument("AsymmetricCoopController: trajectory is null");
    n_rel_ = scene().relativeDofBetweenHands();
    if (n_rel_ < 0 || n_rel_ > 6)
      throw std::runtime_error("AsymmetricCoopController: invalid relative DOF");
  }

  void AsymmetricCoopController::reset(
      const DualArmState &initial_state)
  {
    model_->update(initial_state);

    holding_pose_reference_ =
        model_->eePose(holdingArm());

    working_pose_reference_ =
        model_->eePose(workingArm());

    working_pose_in_object_initial_ =
        initial_state.object.pose.inverse() *
        working_pose_reference_;

    screw_angle_initial_ =
        initial_state.screw.angle;

    reset_time_ = initial_state.t;
    phase_ = Phase::Preload;
    phase_start_time_ = initial_state.t;
    condition_start_time_ = -1.0;
    last_update_time_ = initial_state.t;
    wrench_filter_initialized_ = false;
    filtered_preload_ = 0.0;
    filtered_tightening_torque_ = 0.0;
    tightening_torque_reference_ = 0.0;
    measured_tightening_torque_ = 0.0;
    preload_reference_ = 0.0;
    measured_preload_ = 0.0;
    torque_angle_reference_ = initial_state.screw.angle;
    working_constraint_wrench_.setZero();
    initialized_ = true;

    std::fprintf(
        stderr,
        "[asym_coop] holding arm = %s, "
        "working arm = %s, "
        "relative DOF = %d, "
        "internal-force dim = %d\n",
        armName(holdingArm()),
        armName(workingArm()),
        relativeDof(),
        internalForceDim());
  }
  Wrench AsymmetricCoopController::computeHoldingWrench(
      const Pose &current_pose,
      const Twist &current_twist) const
  {
    const Vector6d pose_error =
        poseError(
            holding_pose_reference_,
            current_pose);

    const Vector6d twist_error =
        -current_twist;

    Wrench wrench =
        params_.holding_stiffness.cwiseProduct(
            pose_error);

    wrench +=
        params_.holding_damping.cwiseProduct(
            twist_error);

    return wrench;
  }
  Wrench AsymmetricCoopController::computeWorkingWrench(
      const DualArmState &state,
      const ObjectReference &reference)
  {
    if (!state.screw.valid)
    {
      tightening_torque_reference_ = 0.0;
      measured_tightening_torque_ = 0.0;
      preload_reference_ = 0.0;
      measured_preload_ = 0.0;
      working_constraint_wrench_.setZero();
      return Wrench::Zero();
    }

    const ClosedChainConstraint *screw =
        scene().screwConstraint();

    if (!screw)
    {
      throw std::logic_error(
          "AsymmetricCoopController requires "
          "a screw constraint");
    }

    const Vector3d axis_world =
        state.object.pose.q *
        screw->axis.normalized();

    const Vector3d axis_point_world =
        state.object.pose.transformPoint(
            screw->point);

    const Arm working_arm =
        workingArm();

    const Pose working_pose =
        model_->eePose(working_arm);

    const Twist working_twist =
        model_->eeTwist(working_arm);

    const Twist screw_basis =
        coop::screwMotionBasisAtPoint(
            axis_world,
            axis_point_world,
            working_pose.p,
            screw->lead);

    const double basis_squared_norm =
        screw_basis.squaredNorm();

    const Matrix6d constraint_projector =
        coop::screwConstraintProjector(
            screw_basis);

    // 沿螺纹轴且压向工件的单位 wrench，参考点为 TCP。
    // 若 TCP 不在轴上，力矩项使该力的作用线仍通过螺纹轴。
    Wrench axial_wrench =
        Wrench::Zero();

    axial_wrench.head<3>() =
        -axis_world;

    axial_wrench.tail<3>() =
        (axis_point_world - working_pose.p).cross(
            -axis_world);

    // 预紧力必须不在允许的螺旋运动方向上做功。
    Wrench preload_basis;

    preload_basis.noalias() =
        constraint_projector * axial_wrench;

    const double preload_basis_squared_norm =
        preload_basis.squaredNorm();

    if (preload_basis_squared_norm < 1e-12)
    {
      throw std::logic_error(
          "AsymmetricCoopController: invalid axial preload direction");
    }

    const Wrench measured_working_wrench =
        state.arm(working_arm).ft_ee_world;

    const double raw_preload =
        preload_basis.dot(
            measured_working_wrench) /
        preload_basis_squared_norm;

    const double raw_axis_torque =
        shiftWrenchRefPoint(
            measured_working_wrench,
            working_pose.p,
            axis_point_world)
            .tail<3>()
            .dot(axis_world);

    // 拧紧方向是负轴向力矩；对外暴露正值大小，方便设置阈值。
    const double raw_tightening_torque =
        -raw_axis_torque;

    const double dt =
        std::max(
            0.0,
            state.t - last_update_time_);

    if (!wrench_filter_initialized_)
    {
      filtered_preload_ = raw_preload;
      filtered_tightening_torque_ = raw_tightening_torque;
      wrench_filter_initialized_ = true;
    }
    else
    {
      const double alpha =
          params_.wrench_filter_time_constant > 0.0
              ? std::clamp(
                    dt /
                        (params_.wrench_filter_time_constant + dt),
                    0.0,
                    1.0)
              : 1.0;

      filtered_preload_ +=
          alpha *
          (raw_preload - filtered_preload_);

      filtered_tightening_torque_ +=
          alpha *
          (raw_tightening_torque - filtered_tightening_torque_);
    }

    last_update_time_ = state.t;
    measured_preload_ = filtered_preload_;
    measured_tightening_torque_ = filtered_tightening_torque_;

    const double elapsed =
        std::max(
            0.0,
            state.t - reset_time_);

    double preload_ramp = 1.0;

    if (params_.axial_preload_ramp_time > 0.0)
    {
      const double phase =
          std::clamp(
              elapsed /
                  params_.axial_preload_ramp_time,
              0.0,
              1.0);

      preload_ramp =
          phase * phase *
          (3.0 - 2.0 * phase);
    }

    preload_reference_ =
        preload_ramp *
        params_.axial_preload_force;

    auto conditionHeld =
        [&](bool condition, double duration)
    {
      if (!condition)
      {
        condition_start_time_ = -1.0;
        return false;
      }
      if (condition_start_time_ < 0.0)
      {
        condition_start_time_ = state.t;
      }
      return state.t - condition_start_time_ >= duration;
    };

    auto enterPhase =
        [&](Phase next)
    {
      phase_ = next;
      phase_start_time_ = state.t;
      condition_start_time_ = -1.0;
    };

    switch (phase_)
    {
    case Phase::Preload:
    {
      const bool ramp_finished =
          elapsed >= params_.axial_preload_ramp_time;

      const bool preload_ready =
          ramp_finished &&
          std::abs(
              measured_preload_ -
              params_.axial_preload_force) <=
              params_.preload_ready_tolerance;

      if (conditionHeld(
              preload_ready,
              params_.preload_ready_hold_time))
      {
        enterPhase(Phase::AngleTighten);
      }
      break;
    }
    case Phase::AngleTighten:
    {
      const bool seated =
          measured_tightening_torque_ >=
          params_.seat_torque_threshold;

      if (conditionHeld(
              seated,
              params_.seat_detect_hold_time))
      {
        enterPhase(Phase::TorqueTighten);
        torque_angle_reference_ =
            state.screw.angle;
      }
      break;
    }
    case Phase::TorqueTighten:
    {
      const bool torque_ramp_finished =
          state.t - phase_start_time_ >=
          params_.tightening_torque_ramp_time;

      const bool complete =
          torque_ramp_finished &&
          measured_tightening_torque_ >=
              params_.tightening_torque -
                  params_.completion_torque_tolerance &&
          std::abs(state.screw.rate) <=
              params_.completion_speed_threshold &&
          std::abs(
              measured_preload_ -
              params_.axial_preload_force) <=
              params_.completion_preload_tolerance;

      if (conditionHeld(
              complete,
              params_.completion_hold_time))
      {
        enterPhase(Phase::Hold);
      }
      break;
    }
    case Phase::Hold:
    {
      // HOLD 不是锁存的“成功”标签：真实夹持可能滑移，必须持续检查
      // 实际力矩和轴向预紧力，否则工具打滑后仍会误报完成。
      const bool hold_lost =
          measured_tightening_torque_ <
              params_.tightening_torque - params_.completion_torque_tolerance ||
          std::abs(measured_preload_ - params_.axial_preload_force) >
              params_.completion_preload_tolerance;
      if (contact_grasp_ && conditionHeld(hold_lost, params_.completion_hold_time))
      {
        enterPhase(Phase::TorqueTighten);
        torque_angle_reference_ = state.screw.angle;
      }
      if (!contact_grasp_) condition_start_time_ = -1.0;
      break;
    }
    }

    double torque_command = 0.0;
    tightening_torque_reference_ = 0.0;

    if (phase_ == Phase::AngleTighten)
    {
      const double angle_error =
          reference.screw_angle -
          state.screw.angle;

      const double rate_error =
          reference.screw_rate -
          state.screw.rate;

      torque_command =
          params_.task_gain * angle_error +
          params_.task_damping * rate_error;

      torque_command =
          std::clamp(
              torque_command,
              -params_.task_torque_limit,
              params_.task_torque_limit);

      tightening_torque_reference_ =
          std::max(0.0, -torque_command);
    }
    else if (phase_ == Phase::TorqueTighten ||
             phase_ == Phase::Hold)
    {
      double tightening_ramp = 1.0;

      if (phase_ == Phase::TorqueTighten &&
          params_.tightening_torque_ramp_time > 0.0)
      {
        const double phase =
            std::clamp(
                (state.t - phase_start_time_) /
                    params_.tightening_torque_ramp_time,
                0.0,
                1.0);

        tightening_ramp =
            phase * phase *
            (3.0 - 2.0 * phase);
      }

      tightening_torque_reference_ =
          params_.seat_torque_threshold +
          tightening_ramp *
              (params_.tightening_torque -
               params_.seat_torque_threshold);

      const double torque_error =
          tightening_torque_reference_ -
          measured_tightening_torque_;

      const double rate_reference =
          -std::clamp(
              params_.tightening_admittance *
                  std::max(0.0, torque_error),
              0.0,
              params_.tightening_max_rate);

      torque_angle_reference_ +=
          rate_reference * dt;

      torque_command =
          params_.task_gain *
              (torque_angle_reference_ -
               state.screw.angle) +
          params_.task_damping *
              (rate_reference -
               state.screw.rate);

      torque_command =
          std::clamp(
              torque_command,
              -params_.tightening_torque_limit,
              params_.tightening_torque_limit);
    }

    // 1D 螺旋运动任务，满足 S^T h_task = torque_command。
    Wrench task_wrench;

    task_wrench.noalias() =
        screw_basis *
        (torque_command / basis_squared_norm);

    double preload_command =
        preload_reference_ +
        params_.axial_preload_gain *
            (preload_reference_ - measured_preload_);

    preload_command =
        std::clamp(
            preload_command,
            -params_.axial_preload_limit,
            params_.axial_preload_limit);

    const Wrench preload_wrench =
        preload_basis *
        preload_command;

    // 从原来的 5D 约束空间中去掉预紧力方向，剩余 4D
    // 用位姿阻抗保持工具侧向位置和倾斜角对准。
    Matrix6d alignment_projector =
        constraint_projector;

    alignment_projector.noalias() -=
        preload_basis *
        preload_basis.transpose() /
        preload_basis_squared_norm;

    // 用完整螺旋变换更新工具参考位姿。仅把初始位姿做瞬时投影
    // 在大角度旋拧后会留下有限转动误差，从而产生伪约束 wrench。
    const Vector3d axis_object =
        screw->axis.normalized();

    const double screw_angle_delta =
        state.screw.angle -
        screw_angle_initial_;

    Pose screw_motion;
    screw_motion.q =
        Quaterniond(
            Eigen::AngleAxisd(
                screw_angle_delta,
                axis_object));

    const double screw_pitch =
        screw->lead /
        (2.0 * M_PI);

    screw_motion.p =
        screw->point -
        screw_motion.q * screw->point +
        screw_pitch *
            screw_angle_delta *
            axis_object;

    const Pose working_pose_desired =
        state.object.pose *
        screw_motion *
        working_pose_in_object_initial_;

    // 4D 工具对准任务。
    const Vector6d pose_error =
        poseError(
            working_pose_desired,
            working_pose);

    Twist working_twist_desired =
        shiftTwistRefPoint(
            state.object.twist,
            state.object.pose.p,
            working_pose.p);

    working_twist_desired.noalias() +=
        screw_basis *
        state.screw.rate;

    const Twist twist_error =
        working_twist_desired -
        working_twist;

    Vector6d constrained_pose_error;

    constrained_pose_error.noalias() =
        alignment_projector *
        pose_error;

    Vector6d constrained_twist_error;

    constrained_twist_error.noalias() =
        alignment_projector *
        twist_error;

    Wrench raw_constraint_wrench =
        params_.working_constraint_stiffness.cwiseProduct(
            constrained_pose_error);

    raw_constraint_wrench +=
        params_.working_constraint_damping.cwiseProduct(
            constrained_twist_error);

    Wrench constraint_wrench;

    constraint_wrench.noalias() =
        alignment_projector *
        raw_constraint_wrench;

    working_constraint_wrench_ =
        preload_wrench +
        constraint_wrench;

    return task_wrench +
           working_constraint_wrench_;
  }
  Vector7d AsymmetricCoopController::computeArmTorque(
      Arm arm,
      const Wrench &wrench) const
  {
    const Matrix6x7d J =
        model_->jacobian(arm);

    Vector7d tau;

    tau.noalias() =
        J.transpose() * wrench;

    tau += model_->bias(arm);

    return tau;
  }
  std::pair<Vector7d, Vector7d>
  AsymmetricCoopController::compute(
      const DualArmState &state,
      double t)
  {
    if (!initialized_)
    {
      throw std::logic_error(
          "AsymmetricCoopController::reset "
          "must be called before compute");
    }

    model_->update(state);
    const ObjectReference reference =
        trajectory_->evaluate(t);
    const Arm holding_arm =
        holdingArm();

    const Arm working_arm =
        workingArm();

    // 先计算作业臂 wrench，使持握臂能同周期前馈抵消其对工件的反力。
    const Wrench working_wrench =
        computeWorkingWrench(
            state,
            reference);

    const Pose holding_pose =
        model_->eePose(holding_arm);

    const Pose working_pose =
        model_->eePose(working_arm);

    const Twist holding_twist =
        model_->eeTwist(holding_arm);

    Wrench holding_wrench =
        computeHoldingWrench(
            holding_pose,
            holding_twist);

    // 工件重力前馈：求持握臂在抓取点应施加的 wrench。
    Wrench object_support_wrench =
        Wrench::Zero();

    object_support_wrench[2] =
        scene().object.mass * 9.81;

    const Vector3d r_holding =
        state.object.pose.p -
        holding_pose.p;

    const Matrix6d G_holding =
        coop::graspMatrixArm(
            r_holding);

    holding_wrench +=
        G_holding.inverse() *
        object_support_wrench;

    // 双臂约束反力前馈：只抵消预紧力和 4D 对准 wrench。
    // 旋拧任务力矩仅由作业臂施加，避免在相对螺旋自由度上将驱动力翻倍。
    const Wrench working_constraint_at_object =
        shiftWrenchRefPoint(
            working_constraint_wrench_,
            working_pose.p,
            state.object.pose.p);

    holding_wrench -=
        G_holding.inverse() *
        working_constraint_at_object;

    std::array<Vector7d, kNumArms> tau;

    // 持握臂：Cartesian impedance + 工件重力前馈。
    tau[armIndex(holding_arm)] =
        computeArmTorque(
            holding_arm,
            holding_wrench);

    // 作业臂：1D 螺旋角度 + 1D 轴向预紧力 + 4D 工具对准。
    tau[armIndex(working_arm)] =
        computeArmTorque(
            working_arm,
            working_wrench);

    return {
        tau[armIndex(Arm::Left)],
        tau[armIndex(Arm::Right)],
    };
  }
} // namespace dual_arm
