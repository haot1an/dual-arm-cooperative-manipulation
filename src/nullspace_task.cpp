#include "dual_arm/nullspace_task.hpp"

#include <Eigen/Cholesky>
#include <Eigen/SVD>
#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

namespace dual_arm {

NullspaceTask::NullspaceTask(const CoopConfig& config) : config_(config) {
  const auto& p = config_.nullspace;
  const double scalars[] = {config_.nullspace_kp, config_.nullspace_kd, p.ramp_time,
      p.max_torque, p.rank_tolerance, p.rotation_length, p.joint_limit_activation,
      p.joint_limit_gain, p.activation_distance, p.avoidance_gain};
  for (double value : scalars)
    if (!std::isfinite(value) || value < 0.0)
      throw std::invalid_argument("coop.nullspace: gains and scales must be finite and non-negative");
  if (!(p.max_torque > 0.0) || !(p.rank_tolerance > 0.0) || p.rank_tolerance >= 1.0 ||
      !(p.rotation_length > 0.0) || !(p.joint_limit_activation > 0.0) || !(p.activation_distance > 0.0))
    throw std::invalid_argument("coop.nullspace: invalid torque, rank, length or activation threshold");
  for (auto& q : reference_) q.setZero();
}

void NullspaceTask::reset(const DualArmState& state) {
  for (Arm arm : kArms) reference_[armIndex(arm)] = state.arm(arm).q;
  reset_time_ = state.t;
  diagnostics_ = {};
}

Matrix7d NullspaceTask::torqueProjector(const Matrix6x7d& J, const Matrix7d& M,
                                      double rotation_length, double rank_tolerance, int* rank) {
  Matrix6x7d scaled = J;
  scaled.bottomRows<3>() *= rotation_length;
  const Eigen::JacobiSVD<Matrix6x7d> svd(scaled, Eigen::ComputeFullV);
  const double threshold = rank_tolerance * svd.singularValues()[0];
  int r = 0;
  for (int k = 0; k < 6; ++k)
    if (svd.singularValues()[k] > threshold) ++r;
  if (rank) *rank = r;
  // 固定 7x7：前 r 列置零，并给这些不使用的维度加单位对角，避免动态矩阵。
  Matrix7d Z = svd.matrixV();
  for (int k = 0; k < r; ++k) Z.col(k).setZero();
  Matrix7d reduced = Z.transpose() * M * Z;
  for (int k = 0; k < r; ++k) reduced(k, k) = 1.0;
  const Eigen::LDLT<Matrix7d> ldlt(reduced);
  if (ldlt.info() != Eigen::Success || !ldlt.isPositive()) return Matrix7d::Zero();
  return M * Z * ldlt.solve(Z.transpose());
}

Vector7d NullspaceTask::jointLimitTorque(const Vector7d& q, const Vector7d& lower,
                                       const Vector7d& upper, double activation, double gain) {
  Vector7d torque;
  for (int j = 0; j < kArmDof; ++j)
    torque[j] = gain * (std::max(0.0, activation - (q[j] - lower[j])) -
                        std::max(0.0, activation - (upper[j] - q[j])));
  return torque;
}

Vector14d NullspaceTask::compute(const DualArmState& state, const RobotModel& model,
                               const std::vector<DistanceInfo>* distances) {
  diagnostics_ = {};
  if (!config_.nullspace.enabled) return Vector14d::Zero();
  const auto& p = config_.nullspace;
  Vector14d avoidance = Vector14d::Zero();
  if (p.avoidance_enabled && distances) {
    // 最近对代表每个组：不让网格数量决定排斥力大小；不使用 object~obstacle。
    const auto& groups = model.scene().obstacles;
    for (int group = 0; group < static_cast<int>(groups.size()); ++group) {
      const auto& spec = groups[group];
      if (spec.a == "object" || spec.b == "object") continue;
      const DistanceInfo* closest = nullptr;
      for (const auto& info : *distances)
        if (info.group == group && info.distance < p.activation_distance &&
            (!closest || info.distance < closest->distance)) closest = &info;
      if (!closest || closest->jacobian.squaredNorm() < 1e-16) continue;
      avoidance.noalias() += p.avoidance_gain * (p.activation_distance - closest->distance) *
                             closest->jacobian.transpose();
      ++diagnostics_.active_pairs;
    }
  }
  Vector14d torque;
  diagnostics_.joint_margin = std::numeric_limits<double>::infinity();
  diagnostics_.min_rank = 6;
  std::array<Matrix6x7d, kNumArms> jacobians;
  std::array<Matrix7d, kNumArms> masses;
  for (Arm arm : kArms) {
    const int i = armIndex(arm), offset = i * kArmDof;
    const auto& s = state.arm(arm);
    const Vector7d lower = model.jointLowerLimit(arm), upper = model.jointUpperLimit(arm);
    diagnostics_.joint_margin = std::min(diagnostics_.joint_margin,
        (s.q - lower).cwiseMin(upper - s.q).minCoeff());
    jacobians[i] = model.jacobian(arm);
    masses[i] = model.massMatrix(arm);
    int rank = 0;
    // M = I 时 M Z (Zᵀ M Z)⁻¹ Zᵀ 退化为正交投影 Z Zᵀ = I − J⁺J（运动学投影，对照用）。
    const bool dynamic = p.projection == NullspaceConfig::Projection::Dynamic;
    const Matrix7d P = torqueProjector(jacobians[i], dynamic ? masses[i] : Matrix7d::Identity().eval(),
                                       p.rotation_length, p.rank_tolerance, &rank);
    diagnostics_.min_rank = std::min(diagnostics_.min_rank, rank);
    const Vector7d repulsive = avoidance.segment<7>(offset);
    const Vector7d raw = config_.nullspace_kp * (reference_[i] - s.q) - config_.nullspace_kd * s.dq +
        jointLimitTorque(s.q, lower, upper, p.joint_limit_activation, p.joint_limit_gain) + repulsive;
    torque.segment<7>(offset).noalias() = P * raw;
    diagnostics_.projected_avoidance_norm += (P * repulsive).squaredNorm();
  }
  double scale = 1.0;
  if (p.ramp_time > 0.0) {
    const double phase = std::clamp((state.t - reset_time_) / p.ramp_time, 0.0, 1.0);
    scale = phase * phase * (3.0 - 2.0 * phase);
  }
  // 统一缩放，而不是逐关节 clip；后者会破坏零空间方向。
  const double peak = torque.cwiseAbs().maxCoeff();
  if (peak > p.max_torque) scale *= p.max_torque / peak;
  torque *= scale;
  diagnostics_.projected_avoidance_norm = scale * std::sqrt(diagnostics_.projected_avoidance_norm);
  diagnostics_.torque_norm = torque.norm();
  for (Arm arm : kArms) {
    const int i = armIndex(arm);
    diagnostics_.acceleration_leak = std::max(diagnostics_.acceleration_leak,
        (jacobians[i] * masses[i].ldlt().solve(torque.segment<7>(i * 7))).norm());
  }
  return torque;
}

} // namespace dual_arm
