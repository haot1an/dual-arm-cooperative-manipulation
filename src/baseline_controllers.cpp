#include "dual_arm/baseline_controllers.hpp"
#include "dual_arm/coop_kinematics.hpp"
#include "dual_arm/math_utils.hpp"

#include <stdexcept>
namespace dual_arm
{

  GravityCompJointPD::GravityCompJointPD(std::shared_ptr<RobotModel> model, const GravityPdConfig &params)
      : Controller(std::move(model)), params_(params)
  {
    for (auto &q : q_ref_)
      q.setZero();
  }

  void GravityCompJointPD::reset(const DualArmState &initial_state)
  {
    for (Arm a : kArms)
      q_ref_[armIndex(a)] = initial_state.arm(a).q;
  }

  std::pair<Vector7d, Vector7d> GravityCompJointPD::compute(const DualArmState &state, double /*t*/)
  {
    model_->update(state);
    std::array<Vector7d, kNumArms> tau;
    for (Arm a : kArms)
    {
      const ArmState &s = state.arm(a);
      const int i = armIndex(a);
      tau[i] = model_->gravity(a) + params_.kp[i].cwiseProduct(q_ref_[i] - s.q) - params_.kd[i].cwiseProduct(s.dq) +
               params_.extra_torque[i];
    }
    return {tau[0], tau[1]};
  }

  IndependentCartesianImpedance::
      IndependentCartesianImpedance(
          std::shared_ptr<RobotModel> model,
          std::shared_ptr<const ObjectTrajectory> trajectory,
          const CartesianImpedanceConfig &params)
      : Controller(std::move(model)),
        trajectory_(std::move(trajectory)),
        params_(params)
  {
    if (!trajectory_)
    {
      throw std::invalid_argument(
          "IndependentCartesianImpedance: trajectory is null");
    }

    if (scene().relativeDofBetweenHands() != 0)
    {
      throw std::invalid_argument(
          "IndependentCartesianImpedance requires "
          "zero relative DOF between the hands");
    }

    // 第一版只支持两只手直接抓同一个主物体。
    for (Arm a : kArms)
    {
      const int i = armIndex(a);

      if (scene().grasp[i].body != scene().object.body)
      {
        throw std::invalid_argument(
            "IndependentCartesianImpedance requires both "
            "grasp sites to belong to the main object");
      }

      grasp_in_object_[i] =
          scene().grasp[i].site_in_body;
    }

    const double lambda_left =
        params_.load_share_left;

    const double lambda_right =
        1.0 - lambda_left;

    if (lambda_left <= 0.0 ||
        lambda_right <= 0.0)
    {
      throw std::invalid_argument(
          "IndependentCartesianImpedance: "
          "load shares must be positive");
    }

    load_weight_.setZero();

    load_weight_.topLeftCorner<6, 6>() =
        Matrix6d::Identity() / lambda_left;

    load_weight_.bottomRightCorner<6, 6>() =
        Matrix6d::Identity() / lambda_right;
  }

  std::pair<Vector7d, Vector7d>
IndependentCartesianImpedance::compute(
    const DualArmState& state,
    double t) {
  // 1. 更新名义机器人模型。
  model_->update(state);

  // 2. 当前时刻物体参考。
  const ObjectReference object_ref =
      trajectory_->evaluate(t);

  // 3. 物体参考转换为两个末端参考。
  const auto ee_ref =
      coop::eeReferencesFromObject(
          object_ref,
          grasp_in_object_);

  // 4. 在期望几何处构造抓取矩阵。
  const Vector3d r_left =
      object_ref.pose.p -
      ee_ref[armIndex(Arm::Left)].pose.p;

  const Vector3d r_right =
      object_ref.pose.p -
      ee_ref[armIndex(Arm::Right)].pose.p;

  const Matrix6x12d G =
      coop::graspMatrix(r_left, r_right);

  // 5. 物体重力前馈。
  //
  // 世界系 z 轴向上，物体重力向下，因此两臂需要
  // 合计向物体施加 +m*g 的 z 向力。
  Wrench object_wrench_ff = Wrench::Zero();
  object_wrench_ff[2] =
      scene().object.mass * 9.81;

  // 只分配物体外载荷，不加入期望内力。
  const Vector12d hand_wrench_ff =
      coop::distributeObjectWrench(
          G,
          object_wrench_ff,
          Vector12d::Zero(),
          load_weight_);

  // 6. 两臂分别计算自己的阻抗 wrench 和关节力矩。
  std::array<Vector7d, kNumArms> tau;

  for (Arm a : kArms) {
    const int i = armIndex(a);

    // 只能使用名义 RobotModel 运动学。
    const Pose current_pose =
        model_->eePose(a);

    const Twist current_twist =
        model_->eeTwist(a);

    const Matrix6x7d J =
        model_->jacobian(a);

    const Vector6d pose_error =
        poseError(
            ee_ref[i].pose,
            current_pose);

    const Vector6d twist_error =
        ee_ref[i].twist -
        current_twist;

    Wrench wrench_cmd =
        hand_wrench_ff.segment<6>(6 * i);

    wrench_cmd +=
        params_.stiffness.cwiseProduct(
            pose_error);

    wrench_cmd +=
        params_.damping.cwiseProduct(
            twist_error);

    tau[i].noalias() =
        J.transpose() * wrench_cmd;

    // 补偿机械臂自身的 C*dq + g。
    tau[i] += model_->bias(a);
  }

  return {
      tau[armIndex(Arm::Left)],
      tau[armIndex(Arm::Right)],
  };
}


} // namespace dual_arm
