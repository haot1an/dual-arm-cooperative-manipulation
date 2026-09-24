#pragma once
/**
 * @file cbf_reference_filter.hpp
 * @brief 连续 CBF 参考滤波：替代 NORMAL→LIFT→CROSS→DESCEND 状态机的在线避障参考生成。
 *
 * 公式、推导与实现清单见 docs/cbf_reference_governor.md。符号：
 *   p_nom(s), R_nom(s), v_nom, ω_nom, a_nom   原始物体轨迹在虚拟时间 s 处的参考（ObjectTrajectory）
 *   δ ∈ R³                                    物体参考的平移偏移（世界系）
 *   u = [δ̇; ṡ] ∈ R⁴                           每周期的决策变量（CBF-QP 的解）
 *   h_j = d_j − d_s                            第 j 个障碍对的屏障函数（d_j 来自 CollisionModel）
 *
 * 滤波后的参考（本周期用 δ_k 与 u_k，随后积分 δ_{k+1} = δ_k + δ̇ Δt，s_{k+1} = s_k + ṡ Δt）：
 *   p_ref = p_nom(s) + δ,  R_ref = R_nom(s)
 *   v_ref = ṡ v_nom + δ̇,   ω_ref = ṡ ω_nom
 *   a_ref ≈ ṡ² a_nom + (δ̇_k − δ̇_{k−1})/Δt  （忽略 s̈ 项，见文档 §2）
 *
 * 每周期 CBF-QP 由 solveCbfQp() 构造，用 small_qp（LDP / Lawson–Hanson NNLS，精确有限步）求解。
 * update() 与 solveCbfQp() 均不做动态内存分配。
 */
#include "dual_arm/config.hpp"
#include "dual_arm/object_trajectory.hpp"
#include "dual_arm/types.hpp"

#include <array>

namespace dual_arm
{

  /// 一个“物体 ~ 障碍”距离对（由 QpCoopController 从 CollisionModel 的 DistanceInfo 转换得到）。
  struct CbfObstacle
  {
    double distance = 0.0;               ///< d_j：在**实测**物体位姿下的有符号距离 [m]
    Vector3d normal = Vector3d::UnitX(); ///< 世界系单位法向，从物体指向障碍（d 沿 −normal 方向增大）
    Vector3d point = Vector3d::Zero();   ///< 物体表面上的最近点（世界系，实测位姿）
    int group = -1;                      ///< SceneSpec.obstacles 的下标
  };

  class CbfReferenceFilter
  {
  public:
    static constexpr int kMaxObstacles = 8;
    using Config = TorqueQpConfig::ReferenceGovernorConfig::Cbf;
    using Obstacles = std::array<CbfObstacle, kMaxObstacles>;

    enum class Status : int
    {
      Solved = 0,     ///< QP 有解，u 满足全部约束
      Relaxed = 1,    ///< 约束不可同时满足，返回了松弛解（实现者自定义松弛策略）
      Infeasible = 2, ///< 无可用解：update() 将令 δ̇ = 0、ṡ = 0（参考停住）
    };

    /// 每周期的决策变量
    struct Decision
    {
      Vector3d offset_rate = Vector3d::Zero(); ///< δ̇ [m/s]
      double time_rate = 1.0;                  ///< ṡ（1 = 按原轨迹速度推进）
    };

    CbfReferenceFilter(const Config &config, double timestep);

    /// 清零偏移、ṡ = 1，虚拟时间置为 virtual_time。
    void reset(double virtual_time = 0.0);

    /// 本周期应在哪个虚拟时间上评估原始轨迹（调用者：nominal = trajectory.evaluate(virtualTime())）。
    double virtualTime() const { return virtual_time_; }

    /**
     * 一个控制周期：调用 solveCbfQp 得到 u，合成滤波后的参考，并把 δ、s 积分到下一周期。
     * @param nominal        原始轨迹在 virtualTime() 处的参考
     * @param actual_object  实测物体位姿（obstacles 中的距离在该位姿下计算）
     * @param obstacles      前 count 项有效，按 distance 升序
     */
    const ObjectReference &update(const ObjectReference &nominal, const Pose &actual_object,
                                  const Obstacles &obstacles, int count);

    const ObjectReference &reference() const { return reference_; }
    const Vector3d &offset() const { return offset_; }
    const Decision &decision() const { return decision_; }
    Status status() const { return status_; }
    /// 本周期 min_j h_j（无障碍时为 +inf）
    double minBarrier() const { return min_barrier_; }
    /// 本周期起作用的约束数（由 solveCbfQp 的实现填写，供日志与测试使用）
    int activeConstraints() const { return active_constraints_; }
    /// 偏移或时间缩放是否在起作用（‖δ‖ > 0.1 mm 或 ṡ < 0.999）
    bool engaged() const;
    const Config &config() const { return config_; }

  private:
    Config config_;
    double timestep_;
    double virtual_time_ = 0.0;
    Vector3d offset_ = Vector3d::Zero();
    Decision decision_;
    Decision previous_decision_;
    ObjectReference reference_;
    Status status_ = Status::Solved;
    double min_barrier_ = 0.0;
    int active_constraints_ = 0;

    /**
     * 构造并求解本周期的 CBF-QP，把解写入 decision，返回求解状态；同时设置 active_constraints_。
     * 满足（文档 §4–§6）：
     *   (1) 每个障碍对的离散 CBF：ḣ_j(u) ≥ −α h_j，其中 ḣ_j 是 u 的仿射函数；
     *   (2) ‖δ‖ ≤ offset_max（以屏障形式写入），|δ̇_i| ≤ offset_speed_max，0 ≤ ṡ ≤ 1；
     *   (3) 相邻周期变化率：|δ̇ − δ̇_prev| ≤ offset_accel_max·Δt，|ṡ − ṡ_prev| ≤ time_rate_accel_max·Δt；
     *   (4) 代价：½ w_δ ‖δ̇ − δ̇_nom‖² + ½ w_s (ṡ − 1)²，δ̇_nom = −k_r δ + 逃逸项（§5，防死锁）；
     *   (5) 分工：δ 只做侧向偏移、沿路径的进度只由 ṡ 决定（§4 最后一条约束）。否则 δ 会沿 −v_nom
     *       “抵消”原轨迹推进，滑到 ‖δ‖ = offset_max 后与障碍屏障冲突而不可行（§6）；
     *   (6) 逃逸项只在参考正朝障碍运动时起作用（§5 的 σ_a），否则越墙后仍在墙附近时会一直抬高。
     * 约束不可同时满足时分级松弛：先去掉变化率约束，再去掉偏移上限屏障（CBF 行从不松弛），返回 Relaxed；
     * 仍无解返回 Infeasible。
     */
    Status solveCbfQp(const ObjectReference &nominal, const Pose &actual_object,
                      const Obstacles &obstacles, int count, Decision &decision);
  };

} // namespace dual_arm
