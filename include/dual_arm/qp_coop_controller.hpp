#pragma once
/**
 * @file qp_coop_controller.hpp
 * @brief 对称双臂协同控制律 + 闭链、关节安全与碰撞约束 torque QP。
 *
 * CoopController 负责生成物体轨迹与内力任务的名义力矩；本控制器使用固定尺寸
 * JointSafetyTorqueQp 统一执行力矩、关节状态、六维刚性闭链和距离约束。
 * 适用于 lift、slot 等两手相对自由度为 0 的刚性协同搬运场景。
 */
#include "dual_arm/collision_model.hpp"
#include "dual_arm/coop_controller.hpp"
#include "dual_arm/torque_qp.hpp"

namespace dual_arm
{

class QpCoopController final : public Controller
{
public:
  enum class GovernorPhase : int
  {
    Normal = 0,
    Lift = 1,
    Cross = 2,
    Descend = 3,
  };

  QpCoopController(
      std::shared_ptr<RobotModel> model,
      std::shared_ptr<const ObjectTrajectory> trajectory,
      const CoopConfig& coop_config,
      const TorqueQpConfig& qp_config,
      const CollisionConfig& collision_config,
      double timestep);

  const char* name() const override { return "qp_coop"; }
  void reset(const DualArmState& initial_state) override;
  std::pair<Vector7d, Vector7d> compute(
      const DualArmState& state,
      double t) override;

  JointSafetyTorqueQp::Status qpStatus() const { return qp_.status(); }
  int activeTorqueConstraints() const { return qp_.activeConstraints(); }
  int qpIterations() const { return qp_.iterations(); }
  double maxConstraintViolation() const { return qp_.maxConstraintViolation(); }
  int maxConstraintViolationRow() const { return qp_.maxConstraintViolationRow(); }
  double closedChainAccelerationResidual() const
  {
    return closed_chain_acceleration_residual_;
  }
  int activeCollisionConstraints() const { return active_collision_constraints_; }
  double collisionMinDistance() const { return collision_min_distance_; }
  double maxCollisionSlack() const { return qp_.maxCollisionSlack(); }
  GovernorPhase governorPhase() const { return governor_phase_; }
  double governorOffset() const { return governor_offset_; }
  double governorVirtualTime() const { return governor_virtual_time_; }
  const ObjectReference& governedReference() const { return governed_reference_; }
  static const char* governorPhaseName(GovernorPhase phase);
  const Vector14d& torqueLowerBound() const { return lower_bound_; }
  const Vector14d& torqueUpperBound() const { return upper_bound_; }

private:
  CoopController nominal_controller_;
  std::shared_ptr<const ObjectTrajectory> trajectory_;
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
  GovernorPhase governor_phase_ = GovernorPhase::Normal;
  double governor_offset_ = 0.0;
  double governor_virtual_time_ = 0.0;
  Vector3d governor_entry_normal_ = Vector3d::Zero();
  ObjectReference governed_reference_;
};

} // namespace dual_arm
