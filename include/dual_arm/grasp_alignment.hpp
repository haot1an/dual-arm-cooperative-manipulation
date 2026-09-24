#pragma once
/**
 * @file grasp_alignment.hpp
 * @brief 接触夹取的抓取对准：预抓取 → 接近 → 对准 → 闭合 → 完成。
 *
 * 只用于 simulation.contact_grasp。抓取点 T_g,i = T_body,i · T_{body,g_i}（被抓 body 的实测位姿，相当于视觉测量；
 * T_{body,g_i} = scene().grasp[i].site_in_body，已含 contact_depth）。
 *
 *   APPROACH  TCP 目标 T_g · Trans(0, 0, −d·(1 − s(τ)))，s 为 min-jerk，d = approach_distance；手指全开
 *   ALIGN     目标 = T_g；位置/姿态误差与 TCP 速度连续 align_hold_time 满足容差
 *   CLOSE     手指开度从 0.04 m 平滑降到 finger_opening − contact_squeeze；四指法向力均 > contact_force_min
 *             后再稳定 settle_time
 *   DONE      由调用者撤除临时托持工装并启动任务控制器
 *
 * 力矩律（每臂独立）：τ = Jᵀ[K e + D (v_ref − v)] + h(q, q̇) + N (k_p (q_grasp − q) − k_d q̇)，
 * N = I − Jᵀ (J Jᵀ)⁻¹ J 为动力学无关的零空间投影，q_grasp 为场景抓取构型 q_init。
 * compute() 不做动态内存分配。
 */
#include "dual_arm/config.hpp"
#include "dual_arm/robot_model.hpp"

#include <array>
#include <memory>
#include <utility>

namespace dual_arm
{

  class GraspAlignment
  {
  public:
    enum class Phase
    {
      Approach,
      Align,
      Close,
      Done,
      Failed,
    };

    GraspAlignment(std::shared_ptr<RobotModel> model, const GraspAlignmentConfig &config);

    /// 以 scene().q_init 为种子，用阻尼最小二乘 IK 求使 TCP 位于预抓取位姿的关节角。
    /// grasp_targets：各臂抓取点的世界系位姿。IK 不收敛时抛异常。
    std::array<Vector7d, kNumArms> preGraspConfiguration(const std::array<Pose, kNumArms> &grasp_targets);

    void reset(const DualArmState &state);

    /// grasp_targets：本周期实测抓取点；finger_normals[i][f]：各指对被抓体的法向力 [N]。
    std::pair<Vector7d, Vector7d> compute(const DualArmState &state,
                                          const std::array<Pose, kNumArms> &grasp_targets,
                                          const std::array<std::array<double, 2>, kNumArms> &finger_normals);

    /// 本周期各臂手指开度指令 [m]
    double fingerTarget(Arm arm) const { return finger_target_[armIndex(arm)]; }
    Phase phase() const { return phase_; }
    bool done() const { return phase_ == Phase::Done; }
    bool failed() const { return phase_ == Phase::Failed; }
    static const char *phaseName(Phase phase);
    /// 最近一次 compute 时两臂 TCP 到抓取点的最大位置误差 [m] / 姿态误差 [rad]
    double positionError() const { return position_error_; }
    double orientationError() const { return orientation_error_; }

  private:
    std::shared_ptr<RobotModel> model_;
    GraspAlignmentConfig config_;
    Phase phase_ = Phase::Approach;
    double start_time_ = 0.0;
    double phase_start_time_ = 0.0;
    double condition_start_time_ = -1.0;
    std::array<double, kNumArms> finger_target_{{0.04, 0.04}};
    double position_error_ = 0.0;
    double orientation_error_ = 0.0;

    void enterPhase(Phase phase, double t);
    bool conditionHeld(bool condition, double t, double hold_time);
  };

} // namespace dual_arm
