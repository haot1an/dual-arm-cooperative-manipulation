// 控制结构与公式见 include/dual_arm/coop_controller.hpp 和
// docs/cooperative_control.md §6。
#include "dual_arm/coop_controller.hpp"

#include "dual_arm/coop_kinematics.hpp"
#include "dual_arm/math_utils.hpp"
#include <cstdio>
#include <stdexcept>
#include <algorithm>
namespace dual_arm
{

  CoopController::CoopController(
      std::shared_ptr<RobotModel> model,
      std::shared_ptr<const ObjectTrajectory> trajectory,
      const CoopConfig &params,
      bool contact_grasp)
      : Controller(std::move(model)),
        trajectory_(std::move(trajectory)),
        params_(params),
        contact_grasp_(contact_grasp)
  {
    if (!trajectory_)
    {
      throw std::invalid_argument(
          "CoopController: trajectory is null");
    }

    if (scene().relativeDofBetweenHands() != 0)
    {
      throw std::invalid_argument(
          "CoopController requires zero relative DOF "
          "between the hands");
    }

    for (Arm a : kArms)
    {
      const int i = armIndex(a);

      if (scene().grasp[i].body != scene().object.body)
      {
        throw std::invalid_argument(
            "CoopController requires both grasp sites "
            "to belong to the main object");
      }

      grasp_in_object_[i] =
          scene().grasp[i].site_in_body;

      q_init_[i].setZero();
    }

    const double lambda_left =
        params_.load_share_left;

    const double lambda_right =
        1.0 - lambda_left;

    if (lambda_left <= 0.0 ||
        lambda_right <= 0.0)
    {
      throw std::invalid_argument(
          "CoopController: load_share_left "
          "must be in (0, 1)");
    }

    load_weight_.setZero();

    load_weight_.topLeftCorner<6, 6>() =
        Matrix6d::Identity() / lambda_left;

    load_weight_.bottomRightCorner<6, 6>() =
        Matrix6d::Identity() / lambda_right;
  }

  Matrix12d CoopController::allocationWeight() const
  {
    if (!contact_grasp_)
      return load_weight_;

    // 摩擦夹持时，绕指垫法向 n（TCP 的 y 轴 = 手指开合方向）的手部力矩只能靠
    // 接触斑内的摩擦传递（~μ·N·r，本场景 < 0.5 N·m）。等权 W 下，两手相距 2r 的物体
    // 在该方向所需力矩约 1/(1+r²) ≈ 92% 被分给手部力矩，物体姿态实际上失控。
    // 在该方向把力矩权重乘以 k：W_m,i = (I + (k−1) n nᵀ)/λ_i，最小加权范数解改由
    // 两手的力差（力臂）产生这一分量。其余方向保持与 weld 基线相同的分配。
    Matrix12d W = load_weight_;
    const double extra = params_.contact_torsion_weight - 1.0;
    for (Arm a : kArms)
    {
      const int i = armIndex(a);
      const Vector3d n = model_->eePose(a).R().col(1);
      W.block<3, 3>(6 * i + 3, 6 * i + 3) +=
          extra * load_weight_(6 * i + 3, 6 * i + 3) * (n * n.transpose());
    }
    return W;
  }

  void CoopController::reset(
      const DualArmState &initial_state)
  {
    for (Arm a : kArms)
    {
      q_init_[armIndex(a)] =
          initial_state.arm(a).q;
    }
  }
  Matrix6x12d CoopController::makeGraspMatrix(
      const Pose &object_pose) const
  {
    const Pose left_grasp_world =
        object_pose *
        grasp_in_object_[armIndex(Arm::Left)];

    const Pose right_grasp_world =
        object_pose *
        grasp_in_object_[armIndex(Arm::Right)];

    // r_i = p_object - p_grasp_i
    const Vector3d r_left =
        object_pose.p - left_grasp_world.p;

    const Vector3d r_right =
        object_pose.p - right_grasp_world.p;

    return coop::graspMatrix(r_left, r_right);
  }
  std::pair<Vector7d, Vector7d>
  CoopController::compute(
      const DualArmState &state,
      double t)
  {
    const ObjectReference reference =
        trajectory_->evaluate(t);
    return computeWithReference(state, reference);
  }

  std::pair<Vector7d, Vector7d>
  CoopController::computeWithReference(
      const DualArmState &state,
      const ObjectReference &reference)
  {
    // 1. 更新控制器侧名义机器人模型。
    model_->update(state);

    // 3. 根据当前物体位姿构造抓取矩阵。
    const Pose left_grasp_world =
        state.object.pose *
        grasp_in_object_[armIndex(Arm::Left)];

    const Pose right_grasp_world =
        state.object.pose *
        grasp_in_object_[armIndex(Arm::Right)];

    const Vector3d r_left =
        state.object.pose.p -
        left_grasp_world.p;

    const Vector3d r_right =
        state.object.pose.p -
        right_grasp_world.p;

    const Matrix6x12d G =
        coop::graspMatrix(
            r_left,
            r_right);


    Vector12d measured_hand_wrench;

    measured_hand_wrench.head<6>() =
        state.arm(Arm::Left).ft_ee_world;

    measured_hand_wrench.tail<6>() =
        state.arm(Arm::Right).ft_ee_world;
    const Vector12d measured_internal_wrench =
        coop::internalWrench(
            G,
            measured_hand_wrench);
    const Vector6d measured_internal_coordinates =
        coop::internalWrenchCoordinates(
            r_left,
            r_right,
            measured_internal_wrench);
    const double elapsed =
        std::max(0.0, state.t - reset_time_);

    double internal_ramp = 1.0;

    if (params_.internal_wrench_ramp_time > 0.0)
    {
      const double phase =
          std::clamp(
              elapsed /
                  params_.internal_wrench_ramp_time,
              0.0,
              1.0);

      // smoothstep：s(0)=0, s(1)=1，并且两端导数为 0。
      internal_ramp =
          phase * phase *
          (3.0 - 2.0 * phase);
    }

    const Vector6d desired_internal_coordinates =
        internal_ramp *
        params_.internal_wrench_des;
    const Vector6d internal_error =
        desired_internal_coordinates -
        measured_internal_coordinates;

    const Vector6d commanded_internal_coordinates =
        desired_internal_coordinates +
        params_.internal_force_gain *
            internal_error;
    // 4. 计算物体中心所需的合 wrench。
    const Wrench object_wrench =
        computeObjectWrench(
            reference,
            state.object);

    // 5. 第一版不主动施加内部力。
    const Vector12d internal_wrench_command =
        coop::internalWrenchFromCoordinates(
            r_left,
            r_right,
            commanded_internal_coordinates);

    // 6. 将物体 wrench 分配给两只手。
    const Vector12d hand_wrench =
        coop::distributeObjectWrench(
            G,
            object_wrench,
            internal_wrench_command,
            allocationWeight());

    const Wrench left_hand_wrench =
        hand_wrench.head<6>();

    const Wrench right_hand_wrench =
        hand_wrench.tail<6>();

    // 7. 末端 wrench 映射到关节力矩。
    const Vector7d tau_left =
        computeArmTorque(
            Arm::Left,
            left_hand_wrench);

    const Vector7d tau_right =
        computeArmTorque(
            Arm::Right,
            right_hand_wrench);

    return {
        tau_left,
        tau_right,
    };
  }
  Wrench CoopController::computeObjectWrench(
      const ObjectReference &reference,
      const ObjectState &measured) const
  {
    const double mass =
        scene().object.mass;

    // 物体惯量原本表达在物体系，需要旋转到世界系。
    const Matrix3d rotation =
        measured.pose.R();

    const Matrix3d inertia_world =
        rotation *
        scene().object.inertia *
        rotation.transpose();

    const Vector3d omega =
        measured.twist.tail<3>();

    Wrench wrench = Wrench::Zero();

    // 平移动力学前馈：m * a_des
    wrench.head<3>() =
        mass * reference.accel.head<3>();

    // 转动动力学前馈：I * alpha_des + omega × I*omega
    wrench.tail<3>() =
        inertia_world *
        reference.accel.tail<3>();

    wrench.tail<3>() +=
        omega.cross(inertia_world * omega);

    // 抵消物体向下的重力。
    wrench[2] += mass * 9.81;

    const Vector6d pose_error =
        poseError(reference.pose, measured.pose);

    const Vector6d twist_error =
        reference.twist - measured.twist;

    wrench +=
        params_.object_stiffness.cwiseProduct(
            pose_error);

    wrench +=
        params_.object_damping.cwiseProduct(
            twist_error);

    return wrench;
  }
  Vector7d CoopController::computeArmTorque(
      Arm arm,
      const Wrench &hand_wrench) const
  {
    const Matrix6x7d J =
        model_->jacobian(arm);

    Vector7d tau;

    tau.noalias() =
        J.transpose() * hand_wrench;

    // 机械臂自身的 C(q,dq)dq + g(q)
    tau += model_->bias(arm);

    return tau;
  }
} // namespace dual_arm
