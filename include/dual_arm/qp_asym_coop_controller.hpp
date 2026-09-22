#pragma once
/**
 * @file qp_asym_coop_controller.hpp
 * @brief 非对称协同控制律 + 闭链关节安全 torque QP。
 *
 * 内层 AsymmetricCoopController 生成任务期望力矩；QP 跟踪该力矩，用相邻周期正则项
 * 抑制力矩跳变，并通过 M(q)ddq+h=tau 同时强制执行器力矩、预测关节位置、速度和
 * 加速度约束，并执行闭链相对加速度等式与基于距离梯度的碰撞速度阻尼约束。
 */
#include "dual_arm/asym_coop_controller.hpp"
#include "dual_arm/collision_model.hpp"
#include "dual_arm/torque_qp.hpp"

namespace dual_arm
{

  class QpAsymmetricCoopController final : public Controller
  {
  public:
    QpAsymmetricCoopController(
        std::shared_ptr<RobotModel> model,
        std::shared_ptr<const ObjectTrajectory> trajectory,
        const AsymCoopConfig &asym_config,
        const TorqueQpConfig &qp_config,
        const CollisionConfig &collision_config,
        double timestep);

    const char *name() const override { return "qp_asym_coop"; }
    void reset(const DualArmState &initial_state) override;
    std::pair<Vector7d, Vector7d> compute(
        const DualArmState &state,
        double t) override;

    AsymmetricCoopController::Phase phase() const
    {
      return nominal_controller_.phase();
    }
    double tighteningTorqueReference() const
    {
      return nominal_controller_.tighteningTorqueReference();
    }
    double measuredTighteningTorque() const
    {
      return nominal_controller_.measuredTighteningTorque();
    }
    double preloadReference() const
    {
      return nominal_controller_.preloadReference();
    }
    double measuredPreload() const
    {
      return nominal_controller_.measuredPreload();
    }
    JointSafetyTorqueQp::Status qpStatus() const { return qp_.status(); }
    int activeTorqueConstraints() const { return qp_.activeConstraints(); }
    int qpIterations() const { return qp_.iterations(); }
    double maxConstraintViolation() const
    {
      return qp_.maxConstraintViolation();
    }
    double closedChainAccelerationResidual() const
    {
      return closed_chain_acceleration_residual_;
    }
    int activeCollisionConstraints() const
    {
      return active_collision_constraints_;
    }
    double collisionMinDistance() const
    {
      return collision_min_distance_;
    }
    double maxCollisionSlack() const
    {
      return qp_.maxCollisionSlack();
    }
    const Vector14d &torqueLowerBound() const { return lower_bound_; }
    const Vector14d &torqueUpperBound() const { return upper_bound_; }

  private:
    AsymmetricCoopController nominal_controller_;
    TorqueQpConfig qp_config_;
    std::unique_ptr<CollisionModel> collision_model_;
    double timestep_ = 0.001;
    JointSafetyTorqueQp qp_;
    Vector14d lower_bound_ = Vector14d::Zero();
    Vector14d upper_bound_ = Vector14d::Zero();
    Vector14d position_lower_ = Vector14d::Zero();
    Vector14d position_upper_ = Vector14d::Zero();
    double closed_chain_acceleration_residual_ = 0.0;
    int active_collision_constraints_ = 0;
    double collision_min_distance_ = 0.0;
  };

} // namespace dual_arm
