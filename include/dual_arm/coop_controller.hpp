#pragma once
/**
 * @file coop_controller.hpp
 * @brief 对称双臂协同控制器——闭链“相对自由度 0”的特例。
 *
 * 适用场景：两只手都刚性抓住同一个物体（lift、slot：两个 rigid 约束，
 * scene().relativeDofBetweenHands() == 0）。此时两手之间没有任何允许的相对运动：
 *   运动学：J_r dq = 0（6 维相对速度全部被约束）；
 *   静力学：内力空间 = null(G) 全部 6 维（挤压 / 拉伸、剪切、扭转、弯曲）。
 * 若场景的相对自由度 n_r > 0（例如装配：螺旋副 n_r = 1），应使用 AsymmetricCoopController，
 * 或把本控制器推广为“相对子空间 S 内允许运动、只在 S 的补空间里调节内力”（见 docs/cooperative_control.md §10）。
 *
 * 控制链路（公式编号见 docs/cooperative_control.md §6）：
 *   0) model_->update(state)；取 J_i, J̇_i dq_i, M_i, h_i, g_i, 名义末端位姿 T_i
 *   1) 物体状态：由名义抓取几何 T_o = T_1 · T_{o,g_1}^{-1}（T_{o,g_i} = scene().grasp[i].site_in_body）
 *      或两臂平均 / 用 state.object 得到 p_o, R_o；r_i = p_o − p_i；ν_o 由 J_a dq 得到     (3.6)
 *   2) 物体参考：ref = trajectory_->evaluate(t)（按 scene().waypoints 生成）              (ObjectTrajectory)
 *   3) 物体级阻抗 / 跟踪：w_o = M_o ν̇_o,des + c_o − w_g + K_o e_o + D_o ė_o
 *      （M_o 由 scene().object.{mass, inertia} 构造）                                     (6.3)
 *   4) 内力调节：h_r,cmd = h_r,des + K_f (h_r,des − h_r,meas)                             (6.4)
 *      或“相对空间柔顺”：h_r,cmd = K_r e_r + D_r ė_r                                      (6.5)
 *   5) 分配：h = G_W^+ w_o + V h_r,cmd                                                    (4.3/4.9)
 *   6) 力矩映射：τ_i = J_iᵀ h_i + h_i(q,dq) + N_i τ_0,i                                    (6.1)
 *   7) QpCoopController 在名义力矩之上统一处理闭链约束、关节/力矩限位、防碰撞
 *      （CollisionModel 提供 d、∂d/∂q；进入 QP 的方式见 collision_model.hpp）             (6.6)
 *   8) 把需要看的中间量（期望内力、物体误差……）写进成员变量，方便调试/日志扩展
 */
#include "dual_arm/config.hpp"
#include "dual_arm/controller.hpp"
#include "dual_arm/object_trajectory.hpp"
#include "dual_arm/robot_model.hpp"

#include <memory>

namespace dual_arm
{

  class CoopController final : public Controller
  {
  public:
    CoopController(std::shared_ptr<RobotModel> model, std::shared_ptr<const ObjectTrajectory> trajectory,
                   const CoopConfig &params, bool contact_grasp = false);

    const char *name() const override { return "coop"; }
    void reset(const DualArmState &initial_state) override;
    std::pair<Vector7d, Vector7d> compute(const DualArmState &state, double t) override;
    /// 使用外部生成的物体参考；供 reference governor / MPC 包装器复用协同控制律。
    std::pair<Vector7d, Vector7d> computeWithReference(
        const DualArmState &state,
        const ObjectReference &reference);

  private:
    std::shared_ptr<const ObjectTrajectory> trajectory_;
    CoopConfig params_;
    bool contact_grasp_ = false;

    /// 根据当前物体位姿构造 G = [G_left, G_right]。
    Matrix6x12d makeGraspMatrix(
        const Pose &object_pose) const;

    /// 计算物体中心处的期望合 wrench。
    Wrench computeObjectWrench(
        const ObjectReference &reference,
        const ObjectState &measured) const;

    /// 将一只手的期望 wrench 映射为该机械臂的关节力矩。
    Vector7d computeArmTorque(
        Arm arm,
        const Wrench &hand_wrench) const;
    double reset_time_ = 0.0;

    /// T_{object, grasp_i}：抓取点在物体系下的固定位姿。
    std::array<Pose, kNumArms> grasp_in_object_;

    /// W：物体 wrench 在两臂之间进行加权分配的权重矩阵。
    Matrix12d load_weight_ = Matrix12d::Zero();

    /// 本周期实际使用的 W；接触夹取时在 load_weight_ 上加重绕指垫法向的力矩项。
    Matrix12d allocationWeight() const;

    std::array<Vector7d, kNumArms> q_init_; ///< 初始构型（可用于零空间姿态保持）
  };

} // namespace dual_arm
