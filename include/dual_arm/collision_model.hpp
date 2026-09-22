#pragma once
/**
 * @file collision_model.hpp
 * @brief 距离查询：SceneSpec.obstacles 中列出的 geom 对之间的有符号距离、法向、最近点与 ∂d/∂q。
 *
 * 实现方式（全部用 MuJoCo 自己的碰撞检测）：
 *   构造时单独编译一份场景模型（基座位姿由调用方给定：控制器用名义位姿，日志 / 评估用 plant 真实位姿），
 *   并对它做两处修改：
 *     (1) 把 obstacles 中每一对名字展开成 geom 对，全部加为显式 <pair>，margin = collision.margin（例如 0.2 m）；
 *     (2) 所有 geom 的 contype = conaffinity = 0，于是 mj_collision 只检测这些显式 pair；
 *     (3) 参与 pair 的 box geom 换成同尺寸的 8 顶点凸网格：MuJoCo 的 box–box 专用函数在分离时给出的不是
 *         欧氏最近点对（分离轴 + 面裁剪的接触生成），换成网格后所有 pair 都走 GJK / EPA，距离与最近点精确。
 *   MuJoCo 对 dist < margin 的 pair 生成 contact：contact.dist 为有符号距离（< 0 为穿透深度），
 *   contact.frame[0:3] 为从 geom[0] 指向 geom[1] 的单位法向，contact.pos 为两表面最近点的中点，所以
 *       p1 = pos − ½·d·n，p2 = pos + ½·d·n，  且 d = nᵀ(p2 − p1)。
 *   同一 pair 可能生成多个 contact（box–box、multiccd），只保留 d 最小的那个。
 *
 * 被抓物体 / 工具：它们在本模型中是 free body，位姿由抓取它的手决定（T_body = T_tcp · T_grasp⁻¹，
 * 主物体被两只手抓时按左手计算——闭链约束满足时两者相同）。其上的点对 q 的雅可比按“固连在该手的 hand body 上”计算。
 *
 * ---------------------------------------------------------------------------------------------
 * ∂d/∂q 的推导（q = [q_left; q_right] ∈ R^14）
 * ---------------------------------------------------------------------------------------------
 *   d(q) = n(q)ᵀ (p2(q) − p1(q))，p1、p2 为两 geom 上的最近点，n = (p2 − p1)/d（d ≠ 0 时）。
 *   对时间求导：
 *       ḋ = ṅᵀ(p2 − p1) + nᵀ(ṗ2 − ṗ1)
 *   (a) p2 − p1 = d·n，而单位向量满足 nᵀṅ = 0 ⇒ 第一项 = d·nᵀṅ = 0；
 *   (b) ṗ_k = （固连在 body_k 上、此刻位于 p_k 的物质点的速度）+（最近点沿表面的滑移速度）。
 *       最近点处表面的切平面垂直于 n（最近点的一阶最优性条件），滑移速度在切平面内 ⇒ 与 n 正交，被 nᵀ 消掉。
 *   因此  ḋ = nᵀ (J_p(body2, p2) − J_p(body1, p1)) q̇，即
 *       ∂d/∂q = nᵀ (J_p2 − J_p1) ∈ R^{1×14}
 *   其中 J_pk = mj_jac(body_k, p_k) 的平动部分（世界系、参考点 p_k），只取两臂 14 列。
 *   固定在世界上的 geom（台面、槽座、立柱）J = 0。穿透时（d < 0）同一公式成立（n 仍为分离方向）。
 *   这是“包络定理”式的结果：在最近点唯一且表面局部光滑时精确；在棱 / 顶点切换处 d(q) 只是分段光滑，
 *   公式给出的是其中一侧的梯度（次梯度）。tests/test_collision_model.cpp 用中心差分验证。
 *
 * ---------------------------------------------------------------------------------------------
 * 如何进入 QP（这里只提供距离与梯度，避障约束 / CBF 本身由你实现）
 * ---------------------------------------------------------------------------------------------
 *   (1) 速度级线性化不等式（速度阻尼器 / velocity damper）：
 *         ḋ = (∂d/∂q) q̇ ≥ −ξ (d − d_s)/(d_i − d_s)，  仅对 d < d_i 的 pair 施加
 *       d_i 影响距离、d_s 安全距离、ξ 收敛速度。力矩级 QP 的决策变量是 q̈ / τ，用 q̇_{k+1} = q̇ + q̈Δt 代入，
 *       得到关于 q̈ 的线性不等式。简单，但只在一个周期内“一阶正确”，高速时可能冲过去。
 *   (2) 二阶控制障碍函数（HOCBF / ECBF，h = d − d_s，相对阶 2）：
 *         ḧ + k1 ḣ + k0 h ≥ 0，  ḣ = (∂d/∂q) q̇，  ḧ = (∂d/∂q) q̈ + (d/dt ∂d/∂q) q̇
 *       对 q̈ 线性，可直接放进力矩级 QP（q̈ 与 τ 通过动力学等式约束相连）。需要 (d/dt ∂d/∂q) q̇（可用有限差分
 *       或忽略法向 / 最近点变化的近似），并且需要 k0、k1 使多项式 s² + k1 s + k0 的根为负实数。
 *   优先级：闭链等式约束（J_r q̈ + J̇_r q̇ = 0 或其螺旋副版本）是“物理事实”，必须作为硬等式约束；
 *   避障不等式在可行时也应作为硬约束，与之冲突时（例如被夹物体将要碰到槽框、而闭链不允许单臂避让）
 *   给避障约束加松弛变量并在代价中重罚，而不是放松闭链——放松闭链意味着请求物理上不可能的运动，
 *   结果只是巨大的内力。详细讨论见 docs/cooperative_control.md §10。
 *
 * 控制环内的 query() 不做动态内存分配：输出缓冲区在构造时按 pair 数预分配（上限 collision.max_pairs）。
 */
#include "dual_arm/config.hpp"
#include "dual_arm/mujoco_utils.hpp"
#include "dual_arm/scene_spec.hpp"
#include "dual_arm/types.hpp"

#include <array>
#include <string>
#include <vector>

namespace dual_arm {

using RowVector14d = Eigen::Matrix<double, 1, 14>;

/// 一个 geom 对的距离信息（只报告 d < margin 的 pair）
struct DistanceInfo {
  int pair = -1;                        ///< geom 对编号（CollisionModel::pairName(pair)）
  int group = -1;                       ///< 来自 SceneSpec.obstacles 的第几项
  int geom1 = -1, geom2 = -1;           ///< geom id（geom1 属于 obstacles[group].a）
  double distance = 0.0;                ///< 有符号距离 d [m]（< 0 = 穿透深度）
  Vector3d normal = Vector3d::UnitZ();  ///< 世界系单位法向，从 geom1 指向 geom2
  Vector3d point1 = Vector3d::Zero();   ///< geom1 表面上的最近点（世界系）
  Vector3d point2 = Vector3d::Zero();   ///< geom2 表面上的最近点（世界系）
  RowVector14d jacobian = RowVector14d::Zero();  ///< ∂d/∂[q_left; q_right]
};

class CollisionModel {
 public:
  /// base_poses：本模型使用的基座位姿（控制器用 nominalBasePose，评估用 plantBasePose）
  CollisionModel(const SceneSpec& scene, const std::array<Pose, kNumArms>& base_poses,
                 const CollisionConfig& cfg, double timestep = 0.001);

  /// 在关节角 q = [q_left; q_right] 下查询全部 d < margin 的 pair（每个 geom 对一条，按 pair 编号无序）。
  /// 返回的引用在下一次 query 前有效。
  const std::vector<DistanceInfo>& query(const Vector7d& q_left, const Vector7d& q_right);
  const std::vector<DistanceInfo>& query(const Vector14d& q) {
    return query(Vector7d(q.head<7>()), Vector7d(q.tail<7>()));
  }

  /// 上一次 query 中第 group 组的最小距离（该组没有 pair 在 margin 内时返回 +margin）
  double groupMinDistance(int group) const;
  /// 上一次 query 中所有 pair 的最小距离（没有则返回 +margin）；可选输出对应的 DistanceInfo 下标
  double minDistance(int* index = nullptr) const;

  int numPairs() const { return static_cast<int>(pair_geom_.size()); }
  int numGroups() const { return static_cast<int>(group_name_.size()); }
  /// "a~b"（obstacles 中的名字）
  const std::string& groupName(int group) const { return group_name_[group]; }
  /// "geom1~geom2"
  std::string pairName(int pair) const;
  double margin() const { return margin_; }
  const mjModel* model() const { return model_.get(); }
  mjData* data() const { return data_.get(); }

 private:
  SceneSpec scene_;
  double margin_ = 0.2;
  MjModelPtr model_;
  MjDataPtr data_;
  std::array<int, kNumArms> qpos_adr_{}, dof_adr_{}, ee_site_{}, hand_body_{};
  struct Attached {
    int body = -1, qpos_adr = -1;
    Arm arm = Arm::Left;
    Pose grasp_in_body;
  };
  std::vector<Attached> attached_;
  std::vector<int> jac_body_;                   ///< 每个 geom 计算雅可比所用的 body
  std::vector<std::array<int, 2>> pair_geom_;   ///< 每个 pair 的 (geom1, geom2)
  std::vector<int> pair_group_;
  std::vector<int> pair_lookup_;                ///< ngeom×ngeom → pair（−1 = 不是 pair）
  std::vector<std::string> group_name_;
  // 预分配缓冲区
  std::vector<int> slot_;                       ///< pair → out_ 下标（本次 query）
  std::vector<DistanceInfo> out_;
  std::vector<mjtNum> jac1_, jac2_;
};

}  // namespace dual_arm
