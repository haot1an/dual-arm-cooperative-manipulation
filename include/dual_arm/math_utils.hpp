#pragma once
/**
 * @file math_utils.hpp
 * @brief 通用的空间几何小工具（与协同控制算法无关，环境和日志也要用）。
 */
#include "dual_arm/types.hpp"

namespace dual_arm {

/// 反对称矩阵 S(v)，满足 S(v) x = v × x。
Matrix3d skew(const Vector3d& v);

/// 由 roll-pitch-yaw（度）构造四元数：R = Rz(yaw) · Ry(pitch) · Rx(roll)（绕固定轴 X→Y→Z）。
Quaterniond quatFromRpyDeg(const Vector3d& rpy_deg);

/// 姿态误差（旋转向量，世界系）：e_o = log(R_des · R^T)^∨，满足 |e_o| = 转角 ∈ [0, π]。
Vector3d rotationError(const Matrix3d& R_des, const Matrix3d& R);

/// 位姿误差 [p_des − p; log(R_des R^T)^∨]，均在世界系下。
Vector6d poseError(const Pose& des, const Pose& cur);

/// 平移 wrench 的参考点（表达坐标系不变）：
///   输入：作用于参考点 a 的 wrench [f; m_a]
///   输出：同一力系对参考点 b 的 wrench [f; m_b]，m_b = m_a + (a − b) × f
Wrench shiftWrenchRefPoint(const Wrench& w_at_a, const Vector3d& a, const Vector3d& b);

/// 平移 twist 的参考点（表达坐标系不变）：
///   输入：刚体在参考点 a 的 twist [v_a; ω]
///   输出：同一刚体在参考点 b 的 twist [v_b; ω]，v_b = v_a + ω × (b − a)
Twist shiftTwistRefPoint(const Twist& v_at_a, const Vector3d& a, const Vector3d& b);

/// 位姿漂移分解到 6 个方向：给定初始位姿 T0、当前位姿 T 与分解坐标系的姿态 R_f（世界系），返回
///   [R_fᵀ (p − p0); R_fᵀ log(R R0ᵀ)^∨]
/// 即在 R_f 的三个轴上的平移分量与转动分量（例如 R_f = 螺纹轴坐标系：z = 轴向进给 / 绕轴转动）。
Vector6d decomposePoseDrift(const Pose& T0, const Pose& T, const Matrix3d& R_f);

/// 雅可比（6×7，[v; ω]）的奇异性指标。注意 J 的线速度行单位为 m/rad、角速度行无量纲，
/// 整体条件数与长度单位有关；因此同时给出只看平移部分 J_v（3×7）的指标。
struct JacobianMetrics {
  double cond = 0.0;            ///< σ_max / σ_min（6×7，SI 单位）
  double sigma_min = 0.0;       ///< 最小奇异值
  double cond_linear = 0.0;     ///< cond(J_v)
  double sigma_min_linear = 0.0;  ///< σ_min(J_v) [m/rad]
  double manipulability = 0.0;  ///< sqrt(det(J Jᵀ))
};
JacobianMetrics jacobianMetrics(const Matrix6x7d& J);

/// 检查数值结果中是否包含 NaN。
template <typename Derived>
bool hasNaN(const Eigen::MatrixBase<Derived>& m) {
  return !(m.array() == m.array()).all();
}

}  // namespace dual_arm
