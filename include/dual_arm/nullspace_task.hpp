#pragma once

#include "dual_arm/collision_model.hpp"
#include "dual_arm/config.hpp"
#include "dual_arm/robot_model.hpp"

namespace dual_arm {

struct NullspaceDiagnostics {
  double torque_norm = 0.0;
  double acceleration_leak = 0.0; ///< max_i ||J_i M_i^-1 tau_ns,i||，未经 QP 修正
  double joint_margin = 0.0;      ///< [rad] 当前两臂最小关节余量
  double projected_avoidance_norm = 0.0;
  int active_pairs = 0;           ///< 每个障碍组只使用最近的 geom pair
  int min_rank = 0;
};

/// 两手末端保持下的关节冗余任务。固定尺寸 SVD / LDLT，无控制周期堆分配。
/// 令 Z 为 null(J) 的基：P_tau = M Z (Z^T M Z)^-1 Z^T = N^T。
/// 理想满秩时 J M^-1 P_tau = 0；SVD 截断情况下残差由 diagnostics 显式记录。
class NullspaceTask {
 public:
  explicit NullspaceTask(const CoopConfig& config);
  void reset(const DualArmState& state);
  Vector14d compute(const DualArmState& state, const RobotModel& model,
                   const std::vector<DistanceInfo>* distances = nullptr);
  const NullspaceDiagnostics& diagnostics() const { return diagnostics_; }

  static Matrix7d torqueProjector(const Matrix6x7d& J, const Matrix7d& M,
                                 double rotation_length, double rank_tolerance,
                                 int* rank = nullptr);
  static Vector7d jointLimitTorque(const Vector7d& q, const Vector7d& lower,
                                  const Vector7d& upper, double activation, double gain);

 private:
  CoopConfig config_;
  std::array<Vector7d, kNumArms> reference_{};
  double reset_time_ = 0.0;
  NullspaceDiagnostics diagnostics_;
};

} // namespace dual_arm
