#pragma once
/**
 * @file coop_kinematics.hpp
 * @brief 【桩 / 由你实现】协同运动学与静力学。
 *
 * 这里只给出函数签名、维度、坐标约定和“应该算什么”，公式编号对应
 * docs/cooperative_control.md。所有桩函数目前返回 NaN 填充的结果（并在第一次调用时
 * 打印一次提示），这样没实现的量在日志/测试里一眼就能看出来；
 * tests/test_coop_template.cpp 中的测试在检测到 NaN 时会自动 SKIP，实现后自动生效。
 *
 * 统一约定（与 types.hpp 一致）：
 *  - 所有向量/矩阵都在**世界系 W** 下表达；
 *  - twist = [v; ω]，wrench = [f; m]；
 *  - arm 1 = Arm::Left，arm 2 = Arm::Right；12 维 / 14 维堆叠向量都是 [arm1; arm2]；
 *  - p_o：物体中心（物体系 o 原点，= 质心）；p_i：抓取点 i（= ee_site 原点）；
 *  - r_i := p_o − p_i ：**从抓取点 i 指向物体中心**的向量（Uchiyama 的“虚拟杆”方向）。
 *    注意有些文献用 p_i − p_o，符号相反！
 *  - h_i：arm i 的末端**施加给物体**的 wrench，参考点 p_i；
 *  - w_o：作用在物体上的合 wrench（不含重力与外扰），参考点 p_o。
 *
 * 实现要求：只用定长 Eigen 类型，不做动态内存分配（这些函数会在 1 kHz 控制环里调用）。
 */
#include "dual_arm/object_trajectory.hpp"
#include "dual_arm/robot_model.hpp"
#include "dual_arm/types.hpp"

#include <array>

namespace dual_arm::coop {

// ---------------------------------------------------------------------------
// §3 运动学
// ---------------------------------------------------------------------------
/**
 * 螺旋副在指定参考点处的单位广义速度方向。
 *
 * screw_axis_world:
 *   螺纹轴方向，世界系，函数内部会归一化。
 *
 * axis_point_world:
 *   螺纹轴上的任意一点，世界系。
 *
 * reference_point_world:
 *   twist 线速度所对应的参考点，通常取 working arm 的 TCP。
 *
 * lead:
 *   螺纹导程 [m/rev]。
 *
 * 返回：
 *
 *   S = [h a + a × (p - c);
 *        a]
 *
 * 其中 h = lead / (2π)。
 *
 * 对螺钉角速度 psi_dot：
 *
 *   twist = S * psi_dot
 */
Twist screwMotionBasisAtPoint(
    const Vector3d& screw_axis_world,
    const Vector3d& axis_point_world,
    const Vector3d& reference_point_world,
    double lead);

/**
 * 螺旋运动方向的正交补投影矩阵。
 *
 *   P_perp = I - S Sᵀ / (Sᵀ S)
 *
 * 满足：
 *
 *   P_perp S = 0
 *   P_perp² = P_perp
 *   P_perpᵀ = P_perp
 *
 * 对任意约束 wrench h_c = P_perp h：
 *
 *   Sᵀ h_c = 0
 *
 * 即该 wrench 不在允许的螺旋运动方向上做功。
 */
Matrix6d screwConstraintProjector(const Twist& screw_basis);
/**
 * 单臂抓取矩阵 G_i ∈ R^{6×6}（式 3.2）。
 *
 *   G_i = [  I₃       0  ]
 *         [ −S(r_i)   I₃ ] ,        r_i = p_o − p_i（世界系）
 *
 * 含义：
 *   静力学（式 4.1）：arm i 在 p_i 处施加的 wrench h_i，等效到物体中心为 G_i h_i；
 *   运动学（式 3.1）：刚性抓取时 ν_i = G_iᵀ ν_o（ν_o 为物体中心 twist）。
 * @param r_i  3 维，世界系，从抓取点 i 指向物体中心
 * @return     6×6
 */
Matrix6d graspMatrixArm(const Vector3d& r_i);

/**
 * 整体抓取矩阵 G = [G_1  G_2] ∈ R^{6×12}（式 4.1）：w_o = G h，h = [h_1; h_2]。
 * rank(G) = 6，零空间维数 12 − 6 = 6（内力空间，§4）。
 */
Matrix6x12d graspMatrix(const Vector3d& r_1, const Vector3d& r_2);

/**
 * 绝对雅可比 J_a ∈ R^{6×14}（式 3.6）：v_a = J_a [dq_1; dq_2]
 *
 *   J_a = ½ [ G_1^{-T} J_1    G_2^{-T} J_2 ]
 *
 * 其中 G_i^{-T} ν_i = ṽ_i 是“经由 arm i 的虚拟杆算出的物体中心 twist”（式 3.3）。
 * 理想刚性闭链下 v_a = ν_o（物体中心 twist）。
 * @param J_1, J_2  6×7 几何雅可比（世界系，参考点 ee_site，[v; ω]），来自 RobotModel
 * @param r_1, r_2  虚拟杆向量 p_o − p_i（世界系）
 */
Matrix6x14d absoluteJacobian(const Matrix6x7d& J_1, const Matrix6x7d& J_2, const Vector3d& r_1,
                             const Vector3d& r_2);

/**
 * 相对雅可比 J_r ∈ R^{6×14}（式 3.7）：v_r = J_r [dq_1; dq_2]
 *
 *   J_r = [ −G_1^{-T} J_1    G_2^{-T} J_2 ]
 *
 * v_r = ṽ_2 − ṽ_1 是两条虚拟杆末端之间的相对 twist；理想刚性闭链下
 * **J_r dq = 0**（式 3.8），即相对运动被箱子约束为零。
 */
Matrix6x14d relativeJacobian(const Matrix6x7d& J_1, const Matrix6x7d& J_2, const Vector3d& r_1,
                             const Vector3d& r_2);

/// 单臂末端参考量：ee_site 的期望位姿 / twist / 加速度（世界系，参考点 ee_site）
struct EeReference {
  Pose pose;
  Twist twist = Twist::Zero();
  Vector6d accel = Vector6d::Zero();
};

/**
 * 由物体期望运动计算两臂末端期望量（式 3.9–3.11）：
 *
 *   T_i,des = T_o,des · T_{o,g_i}                                        (3.9)
 *   ν_i,des = G_iᵀ ν_o,des                                               (3.10)
 *   ν̇_i,des = G_iᵀ ν̇_o,des + [ −ω_o × (ω_o × r_i) ; 0 ]                   (3.11)
 *
 * 其中 r_i = p_o,des − p_i,des。
 * @param object_ref       物体中心参考（世界系），来自 ObjectTrajectory
 * @param grasp_in_object  T_{o,g_i}：抓取点在物体系中的位姿（RobotModel::objectParams()）
 * @return                 [arm1, arm2] 的末端参考
 */
std::array<EeReference, kNumArms> eeReferencesFromObject(
    const ObjectReference& object_ref, const std::array<Pose, kNumArms>& grasp_in_object);

// ---------------------------------------------------------------------------
// §4–5 静力学：负载分配与内力
// ---------------------------------------------------------------------------

/**
 * 加权伪逆 G_W^+ ∈ R^{12×6}（式 5.2）：
 *
 *   G_W^+ = W^{-1} Gᵀ (G W^{-1} Gᵀ)^{-1}
 *
 * 它给出满足 G h = w_o 且使 hᵀ W h 最小的解。W = I 时退化为 Moore–Penrose 伪逆（式 5.1）。
 * 例：W = blkdiag(I₆/λ_1, I₆/λ_2)，λ_1 + λ_2 = 1 时（纯力、r_i→0 的极限下）arm i 承担 λ_i 的负载（式 5.3）。
 * @param W  12×12 对称正定权重
 */
Matrix12x6d weightedPseudoInverse(const Matrix6x12d& G, const Matrix12d& W);

/**
 * 物体 wrench 在两臂间的分配（式 4.3）：
 *
 *   h = G_W^+ w_o + (I₁₂ − G_W^+ G) h_int
 *
 * 第一项：产生物体运动所需的“外力”部分；第二项：投影到 G 的零空间的内力部分，
 * 不改变物体的合 wrench（G · (I − G_W^+ G) = 0）。
 * @param G        6×12 抓取矩阵
 * @param w_o      期望物体合 wrench（世界系，参考点 p_o）
 * @param h_int    期望内力（12 维，任意；只有其零空间分量起作用）
 * @param W        权重（同 weightedPseudoInverse）
 * @return         h = [h_1; h_2] ∈ R^12，每个 h_i 为世界系、参考点 p_i
 */
Vector12d distributeObjectWrench(const Matrix6x12d& G, const Wrench& w_o, const Vector12d& h_int,
                                 const Matrix12d& W);

/**
 * 内力测量（式 4.4）：把测得的两臂 wrench 投影到 G 的零空间
 *
 *   h_int = (I₁₂ − G^+ G) h_meas
 *
 * h_meas 应是“末端施加给物体的 wrench”，世界系、参考点 p_i，并已扣除夹爪（hand 子树）重力，
 * 例如 ArmState::ft_ee_world（腕部 F/T 换算）或 ArmState::weld_wrench（仿真真值）。
 * 注意 ft_ee_world 的参考点是 ee_site，而 weld_wrench 的参考点是抓取点，二者在
 * 有标定误差 / init_mode=current 时可能相差几毫米。
 */
Vector12d internalWrench(const Matrix6x12d& G, const Vector12d& h_meas);

/**
 * 内力的 6 维坐标 h_r（式 4.7），与相对 twist v_r 功率对偶：
 *
 *   h_r = ½ (G_2 h_2 − G_1 h_1)
 *
 * 纯内力时 G_1 h_1 = −G_2 h_2 = −h_r。沿两抓取点连线方向（左→右为正向）时，
 * h_r 的该分量 < 0 表示挤压、> 0 表示拉伸。
 */
Vector6d internalWrenchCoordinates(const Vector3d& r_1, const Vector3d& r_2, const Vector12d& h);

/**
 * 【日志钩子】run_sim 每步调用一次，结果写入 CSV 的 int_l_* / int_r_* 列（12 维）。
 * 对所有控制器（包括基线）都会调用，所以基线和协同控制的内力可以用同一口径对比。
 *
 * 建议实现：
 *   1) 由名义模型得到 p_1, p_2（model.eePose），物体中心 p_o 可取 state.object.pose.p，
 *      或由名义抓取几何从 ee 位姿推算（后者不需要物体测量，更接近真实系统）；
 *   2) r_i = p_o − p_i，G = graspMatrix(r_1, r_2)；
 *   3) h_meas = [state.arms[0].ft_ee_world; state.arms[1].ft_ee_world]（或 weld_wrench）；
 *   4) return internalWrench(G, h_meas)。
 * 注意：调用时 model 已在本周期被控制器 update 过。
 */
Vector12d internalWrenchForLogging(const DualArmState& state, const RobotModel& model);


/**
 * 将六维内部力坐标 h_r 转换为十二维双臂纯内部 wrench。
 *
 * 约定：
 *   G_1 h_1 = -h_r
 *   G_2 h_2 =  h_r
 *
 * 因此：
 *   G_1 h_1 + G_2 h_2 = 0
 */
Vector12d internalWrenchFromCoordinates(
    const Vector3d& r_1,
    const Vector3d& r_2,
    const Vector6d& h_r);
}  // namespace dual_arm::coop
