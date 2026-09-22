#pragma once
/**
 * @file scene_spec.hpp
 * @brief SceneSpec：场景的完整描述。控制器只通过它了解场景，不对布局 / 物体 / 约束做任何硬编码假设。
 *
 * 来源：config/scenes/<name>.yaml（名字、约束、航点……）+ 编译后的 mjModel（质量、惯量、尺寸、
 * 抓取点位姿等数值，由 completeSceneSpec() 从模型读出，保证“一个数只写一处”）。
 *
 * 闭链约束与内力空间维数（你后面要用的关键信息）：
 *   两臂之间的闭链由 constraints 串联而成（手 → 物体 / 工具 → … → 手）。每个约束允许的相对自由度数
 *   relative_dof 如实填写：rigid（weld）= 0，screw（螺旋副：转动与移动按导程耦合）= 1。
 *   两手之间的总相对自由度 n_r = Σ relative_dof（relativeDofBetweenHands()）。
 *   内力空间维数 = 6 − n_r：刚性抓取 0 + 6；螺纹连接 1 + 5（推导见 docs/cooperative_control.md §10）。
 */
#include "dual_arm/types.hpp"

#include <array>
#include <string>
#include <vector>

namespace dual_arm {

enum class ConstraintType { Rigid, Screw };

/// 闭链中的一个约束（两 body 之间）
struct ClosedChainConstraint {
  std::string name;          ///< rigid: weld equality 名；screw: 约束的名字（仅标识）
  ConstraintType type = ConstraintType::Rigid;
  std::string body1, body2;  ///< 涉及的两个 body
  int relative_dof = 0;      ///< 该约束允许的相对自由度数（rigid 0，screw 1）

  // ---- screw 专用 ----
  double lead = 0.0;                     ///< 导程 [m/rev]：每转一圈沿轴移动的距离
  Vector3d axis = Vector3d::UnitZ();     ///< 螺纹轴方向（body1 坐标系，单位向量）
  Vector3d point = Vector3d::Zero();     ///< 轴上一点（body1 坐标系）
  std::string hinge, slide, coupling;    ///< 绕轴转动关节、沿轴移动关节、耦合 equality 的名字

  /// 该约束允许的相对运动方向（6 维 twist 基，表达在 body1 系、参考点 point）：
  /// rigid 返回空；screw 返回 1 列：[lead/(2π)·axis; axis]。
  std::vector<Vector6d> relativeMotionBasis() const;
};

/// 一条臂的抓取：抓取点 site（在被抓 body 上）与夹爪的 weld
struct GraspSpec {
  std::string site;              ///< 抓取点 site（TCP 目标）
  std::string weld;              ///< 夹爪 hand ↔ 被抓 body 的 weld equality 名
  double finger_opening = 0.04;  ///< 手指固定开度（每指）[m]
  // ---- 由模型补全 ----
  std::string body;              ///< site 所属的 body（被抓的物体或工具）
  Pose site_in_body;             ///< 抓取点在该 body 坐标系中的位姿
};

/// 被操作的主物体（必须带 free joint）
struct ObjectSpec {
  std::string body, geom;
  // ---- 由模型补全 ----
  double mass = 0.0;                     ///< [kg]
  Matrix3d inertia = Matrix3d::Zero();   ///< 绕质心的惯量，body 坐标系 [kg·m²]
  Vector3d com = Vector3d::Zero();       ///< 质心在 body 系中的位置
  Vector3d half_size = Vector3d::Zero(); ///< geom 为 box 时的半尺寸 [m]（其它类型为 geom_size 原值）
};

/// 需要做避障 / 距离检查的一对名字。名字可以是：left_arm / right_arm（该臂除 link0 外的全部碰撞体）、
/// object（主物体）、body 名（该 body 的全部碰撞体）或 geom 名。
struct ObstaclePairSpec {
  std::string a, b;
  /// < 0：使用 controller.torque_qp.collision_safe_distance；>= 0：该 pair 的任务相关安全距离。
  double safe_distance = -1.0;
};

/// 任务航点：主物体中心在世界系下的位姿（可带螺钉转角）
struct Waypoint {
  std::string name;
  Pose pose;
  bool has_screw_angle = false;
  double screw_angle = 0.0;  ///< [rad]（负 = 从上往下看顺时针 = 拧紧）
};

/// 螺钉动力学参数（只有含 screw 约束的场景有效）
struct ScrewParams {
  double damping = 0.02;          ///< hinge 粘性阻尼 [N·m·s/rad]
  double frictionloss = 0.05;     ///< hinge 库仑摩擦 [N·m]
  double seat_depth = 0.0005;     ///< 拧入这么深后螺钉头贴合座面 [m]
  double seat_stiffness = 2e4;    ///< 贴合后阻力矩 = k·(拧入深度 − seat_depth) [N·m/m]
  double seat_torque_max = 8.0;   ///< [N·m]
};

struct SceneSpec {
  std::string name, description;
  std::string model_path;                    ///< 场景 MJCF（绝对路径）
  std::array<Pose, kNumArms> base;           ///< 名义基座位姿（来自 layout）
  std::array<Vector7d, kNumArms> q_init;     ///< 初始关节角（抓取位姿）
  ObjectSpec object;
  std::array<GraspSpec, kNumArms> grasp;
  std::vector<ClosedChainConstraint> constraints;
  std::vector<ObstaclePairSpec> obstacles;
  std::vector<Waypoint> waypoints;
  std::vector<std::string> cameras;
  ScrewParams screw;

  const GraspSpec& graspOf(Arm a) const { return grasp[armIndex(a)]; }
  /// 两手之间闭链的总相对自由度 Σ relative_dof（内力空间维数 = 6 − 它）
  int relativeDofBetweenHands() const;
  /// 场景中的螺旋副约束（没有则返回 nullptr）
  const ClosedChainConstraint* screwConstraint() const;
  /// 按名字查约束（没有则返回 nullptr）
  const ClosedChainConstraint* constraint(const std::string& name) const;
};

}  // namespace dual_arm
