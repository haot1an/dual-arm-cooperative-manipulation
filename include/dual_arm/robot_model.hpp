#pragma once
/**
 * @file robot_model.hpp
 * @brief 控制器侧模型的抽象接口。控制器只通过它获取运动学/动力学量。
 *
 * 目前的实现是 MujocoRobotModel（用一份独立的、名义基座位姿的 mjModel 计算），
 * 以后可以换成 Pinocchio 实现而不改控制器代码。
 *
 * 约定（所有实现都必须遵守）：
 *  - 所有量均基于**名义模型**（config 中的基座位姿），与仿真 plant 的真实位姿无关；
 *  - 末端 = ee_site（TCP：两指之间、法兰前方 0.1034 m，z 轴为接近方向，y 轴为手指开合方向）；
 *  - 几何雅可比 J_i ∈ R^{6×7}：
 *        [v_i; ω_i] = J_i · dq_i
 *    行顺序：**线速度在前（前 3 行），角速度在后（后 3 行）**；
 *    参考点：**ee_site 原点**（v_i 是 ee_site 原点的线速度）；
 *    表达坐标系：**世界系 W**（v_i、ω_i 都在世界系下投影）。
 *  - 关节空间动力学（单臂，7 维）：
 *        M_i(q) ddq + h_i(q, dq) = τ_i + J_iᵀ h_ext,i − D·dq（关节阻尼由 plant 施加）
 *    h_i = C(q, dq)·dq + g(q)（偏置项），g_i(q) 为重力项；
 *    M_i 包含转子惯量 armature（menagerie 中为 0.1 kg·m²）。
 *  - 两臂在运动学/动力学上互不耦合（它们只通过被抓物体 / 工具耦合，那部分是协同控制要处理的）。
 *  - 场景信息（物体质量 / 惯量、抓取点、闭链约束及相对自由度、航点、障碍 geom 对）一律从 scene() 读取，
 *    控制器中不得硬编码任何布局或物体假设。
 *
 * 使用方式：每个控制周期先调用一次 update(state)，之后的查询都基于这一时刻。
 */
#include "dual_arm/scene_spec.hpp"
#include "dual_arm/types.hpp"

namespace dual_arm {

class RobotModel {
 public:
  virtual ~RobotModel() = default;

  /// 用当前关节状态（state.arms[i].q / dq）更新内部缓存。每个控制周期调用一次。
  virtual void update(const DualArmState& state) = 0;

  /// ee_site 在世界系中的位姿（名义正运动学）
  virtual Pose eePose(Arm arm) const = 0;
  /// ee_site 的 twist [v; ω] = J·dq（世界系，参考点 ee_site）
  virtual Twist eeTwist(Arm arm) const = 0;
  /// 几何雅可比 6×7（约定见文件头）
  virtual Matrix6x7d jacobian(Arm arm) const = 0;
  /// J̇·dq（6 维，与 jacobian 相同的行顺序/参考点/坐标系），用于加速度级任务
  virtual Vector6d jacobianDotTimesQdot(Arm arm) const = 0;
  /// 关节空间质量矩阵 7×7（含 armature）
  virtual Matrix7d massMatrix(Arm arm) const = 0;
  /// 偏置项 h = C(q,dq)·dq + g(q)
  virtual Vector7d bias(Arm arm) const = 0;
  /// 重力项 g(q)
  virtual Vector7d gravity(Arm arm) const = 0;

  /// 名义基座位姿（世界系）
  virtual Pose basePose(Arm arm) const = 0;
  virtual Vector7d jointLowerLimit(Arm arm) const = 0;
  virtual Vector7d jointUpperLimit(Arm arm) const = 0;
  /// 关节力矩上限（对称），[87 87 87 87 12 12 12] N·m
  virtual Vector7d torqueLimit(Arm arm) const = 0;
  /// 场景描述（名义值；数值字段已由控制器模型补全）：
  ///   scene().object.{mass, inertia, com}      主物体参数
  ///   scene().grasp[i].{body, site_in_body}    抓取点 T_{b,g_i}（ee_site 与之重合；b 可能是物体或工具）
  ///   scene().constraints / relativeDofBetweenHands()   闭链约束与内力空间维数
  virtual const SceneSpec& scene() const = 0;
};

}  // namespace dual_arm
