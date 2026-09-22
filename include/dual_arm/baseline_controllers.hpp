#pragma once
/**
 * @file baseline_controllers.hpp
 * @brief 基线控制器。
 *
 * GravityCompJointPD：τ_i = g_i(q_i) + K_p,i (q_ref,i − q_i) − K_d,i dq_i + τ_extra,i
 *   - g_i 来自 RobotModel（控制器模型），不含物体重量；物体重量由 PD 的静差承担
 *     （约几 mrad 的下沉）。
 *   - q_ref 在 reset() 时取初始关节角（保持初始构型）。
 *   - K_p / K_d / τ_extra 可按臂分别设置（config: controller.gravity_pd.{left,right}）；
 *     τ_extra 是恒定的附加关节力矩，用于“手动施力”实验（例如装配场景中给右臂 joint7 一个
 *     拧紧力矩，验证螺钉转动 / 进给与左臂反力矩）。
 *   - 只用于验证环境：力矩符号、重力补偿、weld 是否正常、日志是否完整。
 *     它不是“两臂独立笛卡尔阻抗”基线——那个基线建议由你实现，以便和协同控制公平对比。
 */
#include "dual_arm/config.hpp"
#include "dual_arm/controller.hpp"
#include "dual_arm/object_trajectory.hpp"
#include <array>
#include <memory>

namespace dual_arm
{

  class GravityCompJointPD final : public Controller
  {
  public:
    GravityCompJointPD(
        std::shared_ptr<RobotModel> model,
        const GravityPdConfig &params);

    const char *name() const override
    {
      return "gravity_pd";
    }

    void reset(
        const DualArmState &initial_state) override;

    std::pair<Vector7d, Vector7d> compute(
        const DualArmState &state,
        double t) override;

    void setReference(
        Arm arm,
        const Vector7d &q_ref)
    {
      q_ref_[armIndex(arm)] = q_ref;
    }

    const Vector7d &reference(Arm arm) const
    {
      return q_ref_[armIndex(arm)];
    }

    void setExtraTorque(
        Arm arm,
        const Vector7d &tau)
    {
      params_.extra_torque[armIndex(arm)] = tau;
    }

    const GravityPdConfig &params() const
    {
      return params_;
    }

  private:
    GravityPdConfig params_;
    std::array<Vector7d, kNumArms> q_ref_;
  };

  class IndependentCartesianImpedance final
      : public Controller
  {
  public:
    IndependentCartesianImpedance(
        std::shared_ptr<RobotModel> model,
        std::shared_ptr<const ObjectTrajectory> trajectory,
        const CartesianImpedanceConfig &params);

    const char *name() const override
    {
      return "cartesian_impedance";
    }

    std::pair<Vector7d, Vector7d> compute(
        const DualArmState &state,
        double t) override;

  private:
    std::shared_ptr<const ObjectTrajectory> trajectory_;
    CartesianImpedanceConfig params_;
    std::array<Pose, kNumArms> grasp_in_object_;
    Matrix12d load_weight_ = Matrix12d::Identity();
  };

} // namespace dual_arm