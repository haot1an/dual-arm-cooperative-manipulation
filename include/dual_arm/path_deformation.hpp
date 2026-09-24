#pragma once
/**
 * @file path_deformation.hpp
 * @brief 规划得到的物体路径变形 δ(τ) 与重新计时 τ(t)（docs/trajectory_planning.md §5–§6）。
 *
 * 名义轨迹时间记为 τ（ObjectTrajectory::evaluateNominal 的自变量），执行时间记为 t：
 *   p(t) = p_nom(τ) + δ(τ),  R(t) = R_nom(τ),  τ = τ(t)
 *   v    = τ̇ (v_nom + δ'),   ω = τ̇ ω_nom
 *   a    = τ̇² (a_nom + δ'') + τ̈ (v_nom + δ'),   α = τ̇² α_nom + τ̈ ω_nom
 *
 * δ(τ)：均匀节点上的三次样条（两端一阶导为 0，C²），节点之外为 0。
 * τ(t)：先算放慢系数 k(τ) = max(1, ‖v‖/v_max, sqrt(‖a‖/a_max))，做“滑动最大 + 滑动平均”平滑（结果 ≥ 原值），
 *       再由 dt = k dτ 积分得到 t(τ) 并反查；τ̇ = 1/k，τ̈ = −k'(τ)/k³。这是分段放慢的近似时间参数化，
 *       不是 TOPP-RA 那样的时间最优解（见文档 §6）。
 * 联合优化（ObjectTrajectoryOptimizer）另外给出：
 *   φ(τ)：姿态偏移（世界系左乘，R = Exp(φ) R_nom），与 δ 同样的三次样条；
 *   τ(t)：由优化得到的节点执行时刻 t_k 做单调三次 Hermite 插值（Fritsch–Carlson），替代上面的分段放慢。
 * offset()/rotationOffset()/timeMap() 不做动态内存分配。
 */
#include "dual_arm/types.hpp"

#include <vector>

namespace dual_arm
{

  class ObjectTrajectory;

  class PathDeformation
  {
  public:
    /// knot_times 必须均匀递增；offsets 与之等长（首末通常为 0）。
    PathDeformation(std::vector<double> knot_times, std::vector<Vector3d> offsets);

    /// 按规划后路径的平移速度 / 加速度重新计时（nominal 只用其 evaluateNominal）。
    void retime(const ObjectTrajectory &nominal, double max_speed, double max_accel, double smoothing_window = 0.25);

    /// 节点姿态偏移 φ_k（与 knot_times 等长；首末通常为 0）
    void setRotationOffsets(std::vector<Vector3d> rotation_offsets);
    bool hasRotation() const { return !rotation_offsets_.empty(); }
    /// 节点执行时刻 t_k（严格递增，t_0 = τ_0）：定义 τ(t)，替代 retime()
    void setKnotExecutionTimes(std::vector<double> knot_execution_times);

    /// δ, δ', δ''（对名义时间 τ）
    void offset(double tau, Vector3d &d, Vector3d &d_prime, Vector3d &d_second) const;
    /// φ, φ', φ''（对名义时间 τ）；没有姿态偏移时为 0
    void rotationOffset(double tau, Vector3d &phi, Vector3d &phi_prime, Vector3d &phi_second) const;
    /// 执行时间 t → τ, τ̇, τ̈（未重新计时时为恒等映射）
    void timeMap(double t, double &tau, double &tau_dot, double &tau_ddot) const;
    /// 名义时间 τ 对应的执行时间
    double executionTime(double tau) const;

    const std::vector<double> &knotTimes() const { return knot_times_; }
    const std::vector<Vector3d> &offsets() const { return offsets_; }
    const std::vector<Vector3d> &rotationOffsets() const { return rotation_offsets_; }
    /// 重新计时带来的总时长增加 [s]
    double addedDuration() const;

  private:
    std::vector<double> knot_times_;
    std::vector<Vector3d> offsets_;
    std::vector<Vector3d> second_derivative_; ///< 样条节点处的 δ''
    std::vector<Vector3d> rotation_offsets_, rotation_second_derivative_;
    double h_ = 1.0;

    // 节点时间映射（setKnotExecutionTimes）
    std::vector<double> knot_exec_t_, knot_slope_; ///< t_k 与 dτ/dt 在节点处的斜率

    static std::vector<Vector3d> splineSecondDerivatives(const std::vector<Vector3d> &values, double h);
    void evaluateSpline(const std::vector<Vector3d> &values, const std::vector<Vector3d> &second, double tau,
                        Vector3d &v, Vector3d &v_prime, Vector3d &v_second) const;

    // 重新计时表（均匀 τ 网格）
    std::vector<double> grid_tau_, grid_t_, grid_k_;
    double grid_step_ = 0.0;
  };

} // namespace dual_arm
