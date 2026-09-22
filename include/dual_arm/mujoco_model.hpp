#pragma once
/**
 * @file mujoco_model.hpp
 * @brief 用 MuJoCo 实现的 RobotModel。
 *
 * 持有一份**独立于仿真 plant** 的 mjModel/mjData（“控制器模型”），基座使用 config 中的
 * 名义位姿。即使 plant 注入了标定误差，这里的正运动学/雅可比也不会知道。
 *
 * update() 中的计算（全部在构造时预分配的缓冲区中完成，无动态内存分配）：
 *   qpos ← q, qvel ← 0
 *   mj_kinematics → mj_comPos → mj_crb        : 位姿、质量矩阵 M（含 armature）
 *   mj_comVel → mj_rne(flg_acc=0)             : qvel=0 时的偏置 = 重力项 g
 *   qvel ← dq, mj_comVel → mj_rne(flg_acc=0)  : 偏置 h = C·dq + g
 *   mj_jacSite / mj_jacDot                    : J（世界系，ee_site）与 J̇
 *   mj_fullM                                  : 稠密 M，截取每条臂的 7×7 对角块
 * 两臂之间没有运动学耦合，所以 M 的非对角块为 0，h 只含本臂的项。
 * 注意这里不调用 mj_forward：不需要接触/约束，且 weld 约束力不属于 h。
 */
#include "dual_arm/config.hpp"
#include "dual_arm/mujoco_utils.hpp"
#include "dual_arm/robot_model.hpp"

#include <vector>

namespace dual_arm {

class MujocoRobotModel final : public RobotModel {
 public:
  /// 用 cfg.scene（场景模型、手指开度）与名义基座位姿构建控制器模型
  explicit MujocoRobotModel(const SimConfig& cfg);

  void update(const DualArmState& state) override;

  Pose eePose(Arm arm) const override { return cache_[armIndex(arm)].ee; }
  Twist eeTwist(Arm arm) const override { return cache_[armIndex(arm)].twist; }
  Matrix6x7d jacobian(Arm arm) const override { return cache_[armIndex(arm)].J; }
  Vector6d jacobianDotTimesQdot(Arm arm) const override { return cache_[armIndex(arm)].Jdot_dq; }
  Matrix7d massMatrix(Arm arm) const override { return cache_[armIndex(arm)].M; }
  Vector7d bias(Arm arm) const override { return cache_[armIndex(arm)].h; }
  Vector7d gravity(Arm arm) const override { return cache_[armIndex(arm)].g; }

  Pose basePose(Arm arm) const override { return base_[armIndex(arm)]; }
  Vector7d jointLowerLimit(Arm arm) const override { return q_lo_[armIndex(arm)]; }
  Vector7d jointUpperLimit(Arm arm) const override { return q_hi_[armIndex(arm)]; }
  Vector7d torqueLimit(Arm arm) const override { return tau_lim_[armIndex(arm)]; }
  const SceneSpec& scene() const override { return scene_; }

  const mjModel* model() const { return model_.get(); }

 private:
  struct ArmCache {
    Pose ee;
    Twist twist = Twist::Zero();
    Matrix6x7d J = Matrix6x7d::Zero();
    Vector6d Jdot_dq = Vector6d::Zero();
    Matrix7d M = Matrix7d::Identity();
    Vector7d h = Vector7d::Zero();
    Vector7d g = Vector7d::Zero();
  };

  SceneSpec scene_;
  MjModelPtr model_;
  MjDataPtr data_;
  SceneIndices idx_;
  std::array<Pose, kNumArms> base_;
  std::array<Vector7d, kNumArms> q_lo_, q_hi_, tau_lim_;
  std::array<ArmCache, kNumArms> cache_;

  // 预分配的 MuJoCo 输出缓冲区（nv 维）
  std::vector<mjtNum> jacp_, jacr_, jacp_dot_, jacr_dot_, full_m_, bias_, grav_;
};

}  // namespace dual_arm
