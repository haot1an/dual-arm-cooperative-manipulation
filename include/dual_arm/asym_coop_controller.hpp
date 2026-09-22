#pragma once
/**
 * @file asym_coop_controller.hpp
 * @brief 非对称双臂协同控制器：持握臂（holding arm）+ 作业臂（working arm）。
 *
 * 典型场景：装配（scene_assembly）。左手刚性夹住带螺纹孔的工件，右手握住起子（工具尖端 weld 在螺钉头上），
 * 螺钉与工件之间是螺旋副：绕轴转动与沿轴移动按导程耦合（1 个相对自由度）。
 *
 * 闭链：左手 —rigid— 工件 —screw(1)— 螺钉 —rigid— 工具 —rigid— 右手
 *   两手之间的总相对自由度 n_r = Σ relative_dof = 1（scene().relativeDofBetweenHands()），
 *   相对运动子空间 S = [lead/(2π)·a; a] ∈ R^{6×1}（scene().screwConstraint()->relativeMotionBasis()，
 *   a 为螺纹轴；需要换到世界系 / 两手的参考点）。
 *   内力空间维数 = 6 − n_r = 5：两手之间能“互相较劲”而不改变物体运动的方向只剩 5 个——
 *   沿 S 方向的相对 wrench 不再是内力，而是做功的“驱动力矩 + 轴向力”的组合（拧紧力矩），
 *   它被螺钉的阻力（阻尼、摩擦、座面阻力矩）平衡，而不是被闭链“吸收”。
 *   一般形式：内力空间 = null(G) 中与 S 功率正交的部分（推导与两个例子见 docs/cooperative_control.md §10）。
 *
 * 角色与目标：
 *   持握臂（asym_coop.holding_arm，默认 left）：
 *     - 目标：让工件保持在初始位姿（或按 scene().waypoints 缓慢移动），高刚度 6 维阻抗；
 *     - 承受作业臂拧紧时经闭链传过来的反力矩（绕螺纹轴）与轴向推力——这些在左腕 F/T 上可测；
 *     - 评价指标：工件漂移 decomposePoseDrift(T0, T, R_f) 的 6 个分量（日志 drift_*）。
 *   作业臂（另一只）：
 *     - 目标 1（沿 S，1 维）：跟踪螺钉转角 / 进给参考（trajectory 中的 screw_angle），或施加期望拧紧力矩；
 *     - 目标 2（S 的补空间，5 维）：调节两手之间的内力（例如保持一个小的轴向压紧力、其余为 0），
 *       避免因为两臂的位置误差在螺钉 / 工具上“别劲”；
 *     - 相对自由度沿 S 由作业臂主动驱动，持握臂只做被动的 6 维位置保持——这就是“非对称”：
 *       对称协同（CoopController）把物体运动均分给两臂、内力 6 维；这里运动任务只在作业臂上，
 *       持握臂只负责“站稳”，内力维数由约束决定（6 − n_r）。
 *
 * 建议实现步骤：
 *   0) model_->update(state)；读 scene().constraints，得到 n_r 与 S（构造 / reset 时预计算，控制环内不分配）
 *   1) 持握臂：τ_h = J_hᵀ (K_h e_h + D_h ė_h) + g_h（或 QP 中的高权重任务），e_h 为工件位姿误差
 *   2) 作业臂：沿 S 的任务（转角 / 力矩）+ 补空间内力调节 → h_w，τ_w = J_wᵀ h_w + g_w
 *   3) （之后）统一成力矩级 QP：闭链等式约束只约束 S 的补空间（J_r dq ∈ span(S)），
 *      避障用 CollisionModel 的 d、∂d/∂q
 *
 * 当前实现：持握臂用 6D Cartesian impedance 保持工件；作业臂执行
 * PRELOAD -> ANGLE_TIGHTEN -> TORQUE_TIGHTEN -> HOLD 状态机。控制器只用腕部 F/T
 * 沿螺纹轴投影后的力与力矩判断预紧和座面接触，不读取仿真的座面阻力真值。
 * 沿螺旋运动子空间先跟踪螺钉角度，座面接触后切换为单向 admittance 力矩控制；
 * 在其 5D 正交补空间中用 1D 力反馈建立轴向预紧力，用剩余 4D 阻抗保持工具对准。
 * 工具对准参考由完整有限螺旋变换生成，避免大角度旋拧时把合法螺旋运动误判成位姿误差。
 */
#include "dual_arm/config.hpp"
#include "dual_arm/controller.hpp"
#include "dual_arm/object_trajectory.hpp"
#include "dual_arm/robot_model.hpp"

#include <memory>

namespace dual_arm
{

  class AsymmetricCoopController final : public Controller
  {
  public:
    enum class Phase : int
    {
      Preload = 0,
      AngleTighten = 1,
      TorqueTighten = 2,
      Hold = 3,
    };

    AsymmetricCoopController(std::shared_ptr<RobotModel> model, std::shared_ptr<const ObjectTrajectory> trajectory,
                             const AsymCoopConfig &params);

    const char *name() const override { return "asym_coop"; }
    void reset(const DualArmState &initial_state) override;
    std::pair<Vector7d, Vector7d> compute(const DualArmState &state, double t) override;

    Arm holdingArm() const { return params_.holding_arm; }
    Arm workingArm() const { return params_.holding_arm == Arm::Left ? Arm::Right : Arm::Left; }
    /// 两手之间的相对自由度 n_r 与内力空间维数 6 − n_r（由 SceneSpec 推出）
    int relativeDof() const { return n_rel_; }
    int internalForceDim() const { return 6 - n_rel_; }
    Phase phase() const { return phase_; }
    static const char *phaseName(Phase phase);
    double tighteningTorqueReference() const { return tightening_torque_reference_; }
    double measuredTighteningTorque() const { return measured_tightening_torque_; }
    double preloadReference() const { return preload_reference_; }
    double measuredPreload() const { return measured_preload_; }

  private:
    std::shared_ptr<const ObjectTrajectory> trajectory_;
    AsymCoopConfig params_;
    int n_rel_ = 0;
    /// 持握臂在 reset 时的 TCP 位姿。
    Pose holding_pose_reference_;
    /// 作业臂初始 TCP 位姿；无螺旋约束的退化场景直接以它为参考。
    Pose working_pose_reference_;
    /// reset 时作业臂 TCP 在工件坐标系中的位姿。
    Pose working_pose_in_object_initial_;
    double screw_angle_initial_ = 0.0;
    /// reset 时刻，用于平滑建立轴向预紧力。
    double reset_time_ = 0.0;
    Phase phase_ = Phase::Preload;
    double phase_start_time_ = 0.0;
    double condition_start_time_ = -1.0;
    double last_update_time_ = 0.0;
    bool wrench_filter_initialized_ = false;
    double filtered_preload_ = 0.0;
    double filtered_tightening_torque_ = 0.0;
    double tightening_torque_reference_ = 0.0;
    double measured_tightening_torque_ = 0.0;
    double preload_reference_ = 0.0;
    double measured_preload_ = 0.0;
    double torque_angle_reference_ = 0.0;
    /// 需要由持握臂前馈抵消的作业臂约束 wrench（不含旋拧任务力矩）。
    Wrench working_constraint_wrench_ = Wrench::Zero();
    /// 是否已经完成 reset。
    bool initialized_ = false;
    Wrench computeHoldingWrench(
        const Pose &current_pose,
        const Twist &current_twist) const;
    Wrench computeWorkingWrench(
        const DualArmState &state,
        const ObjectReference &reference);
    Vector7d computeArmTorque(
        Arm arm,
        const Wrench &wrench) const;
  };

} // namespace dual_arm
