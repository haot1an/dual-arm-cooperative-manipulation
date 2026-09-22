// 力测量的符号 / 参考点 / 坐标系约定：
//  - weld 约束 wrench（efc_force → Jᵀf）与约束施加在机械臂关节上的广义力一致（牛顿第三定律）；
//  - weld 约束 wrench 与腕部 F/T（扣除法兰重力、修正 MuJoCo weld 力矩问题后）在静止时一致；
//  - 复现 MuJoCo 3.12 原始 F/T 力矩读数中 weld 部分偏大一倍的问题（fix_weld_torque=false）；
//  - 两臂 weld 合力与物体重力、外加扰动平衡；
//  - 物体 twist（由 free joint qvel 换算）与 mj_objectVelocity 一致。
// 场景：lift（两手刚性抓同一根杆）。
#include "dual_arm/baseline_controllers.hpp"
#include "dual_arm/math_utils.hpp"
#include "dual_arm/mujoco_model.hpp"
#include "dual_arm/sim_env.hpp"
#include "test_common.hpp"

#include <gtest/gtest.h>

#include <memory>

using namespace dual_arm;

namespace {

/// 用 GravityCompJointPD 仿真 n 步
void runPd(SimEnv& env, GravityCompJointPD& ctrl, int n) {
  for (int k = 0; k < n; ++k) {
    const auto [tl, tr] = ctrl.compute(env.state(), env.time());
    env.step(tl, tr);
  }
}

}  // namespace

TEST(WristSensor, AgreesWithWeldWrenchInStaticHold) {
  const SimConfig cfg = test::testConfig();
  SimEnv env(cfg);
  auto model = std::make_shared<MujocoRobotModel>(cfg);
  GravityCompJointPD ctrl(model, cfg.gravity_pd);
  ctrl.reset(env.state());
  runPd(env, ctrl, 2000);

  const DualArmState& s = env.lastStep();
  const mjModel* m = env.model();
  for (Arm a : kArms) {
    const ArmState& as = s.arm(a);
    // weld wrench 从抓取点换到 ee_site，与 F/T 换算结果比较
    const Wrench weld_at_ee = shiftWrenchRefPoint(as.weld_wrench, test::graspPoint(env, a), as.ee_pose.p);
    EXPECT_LT((as.ft_ee_world.head<3>() - weld_at_ee.head<3>()).norm(), 0.05)
        << armName(a) << " force: ft " << as.ft_ee_world.head<3>().transpose() << " vs weld "
        << weld_at_ee.head<3>().transpose();
    EXPECT_LT((as.ft_ee_world.tail<3>() - weld_at_ee.tail<3>()).norm(), 0.02)
        << armName(a) << " torque: ft " << as.ft_ee_world.tail<3>().transpose() << " vs weld "
        << weld_at_ee.tail<3>().transpose();

    // 原始读数的符号：link7 对 hand 子树的力（世界系）向上，大小 = 夹爪（hand + 两指）重力 + 该手托住的那部分物体重量
    // （闭链静不定：两手的分担比例取决于两臂的关节刚度与构型，不一定各一半）
    const int fs = env.indices()[a].ft_site;
    const Eigen::Map<const Eigen::Matrix<double, 3, 3, Eigen::RowMajor>> R_s(env.data()->site_xmat + 9 * fs);
    const Vector3d F_l7_world = R_s * as.ft_raw.head<3>();
    const double m_hand = m->body_subtreemass[env.indices()[a].hand_body];
    EXPECT_NEAR(m_hand, 0.76, 1e-9);  // 0.73 + 2 × 0.015：hand 质量与惯量保留
    EXPECT_NEAR(F_l7_world.z(), m_hand * 9.81 + as.weld_wrench.z(), 0.05) << armName(a);
    EXPECT_GT(as.weld_wrench.z(), 0.2 * env.scene().object.mass * 9.81) << armName(a);
  }
}

TEST(WeldWrench, ConsistentWithGeneralizedForceOnArmJoints) {
  // weld 行对机械臂关节的广义力 J_armᵀ f 应等于 J_eeᵀ · (−weld_wrench 平移到 ee_site)
  const SimConfig cfg = test::testConfig();
  SimEnv env(cfg);
  auto model = std::make_shared<MujocoRobotModel>(cfg);
  GravityCompJointPD ctrl(model, cfg.gravity_pd);
  ctrl.reset(env.state());
  runPd(env, ctrl, 1500);
  env.forward();  // 在当前状态重新计算约束力，便于直接读 efc 与雅可比

  const mjModel* m = env.model();
  const mjData* d = env.data();
  const int nv = m->nv;
  model->update(env.state());  // 无标定误差时名义雅可比 = 真实雅可比
  for (Arm a : kArms) {
    const auto& ai = env.indices()[a];
    Vector7d q_c = Vector7d::Zero();
    for (int r = 0; r < d->nefc; ++r) {
      if (d->efc_type[r] != mjCNSTR_EQUALITY || d->efc_id[r] != ai.weld_eq) continue;
      for (int j = 0; j < kArmDof; ++j) q_c[j] += d->efc_J[r * nv + ai.dof_adr + j] * d->efc_force[r];
    }
    const ArmState& as = env.state().arm(a);
    const Wrench on_ee = -shiftWrenchRefPoint(as.weld_wrench, test::graspPoint(env, a), as.ee_pose.p);
    EXPECT_LT((model->jacobian(a).transpose() * on_ee - q_c).norm(), 1e-8 * (1.0 + q_c.norm()))
        << armName(a);
  }
}

TEST(WristSensor, RawMujocoTorqueDoublesWeldContribution) {
  // 记录 MuJoCo 3.12 的行为：关闭修正时，腕部力矩中 weld 引起的部分是真实值的 2 倍。
  // 若升级 MuJoCo 后此测试失败，说明上游已修复，应把 sensors.fix_weld_torque 设为 false。
  // 为了让 weld 力矩足够大、与抓取几何无关，对杆施加绕 x（长轴）的 2 N·m 力矩（主要由两个 weld 的力偶承担）。
  SimConfig cfg = test::testConfig({"sensors.fix_weld_torque=false"});
  DisturbanceConfig dist;
  dist.t_start = 0.0;
  dist.t_end = 100.0;
  dist.torque = Vector3d(2.0, 0.0, 0.0);
  cfg.disturbances = {dist};
  SimEnv env(cfg);
  auto model = std::make_shared<MujocoRobotModel>(cfg);
  GravityCompJointPD ctrl(model, cfg.gravity_pd);
  ctrl.reset(env.state());
  runPd(env, ctrl, 2000);
  const DualArmState& s = env.lastStep();
  for (Arm a : kArms) {
    const ArmState& as = s.arm(a);
    const Wrench weld_at_ee = shiftWrenchRefPoint(as.weld_wrench, test::graspPoint(env, a), as.ee_pose.p);
    // 力分量不受影响
    EXPECT_LT((as.ft_ee_world.head<3>() - weld_at_ee.head<3>()).norm(), 0.05);
    // 取 weld 力矩最大的分量比较：weld 的力矩（相对抓取点 ≈ TCP）全部来自转动行的纯力偶，原始读数翻倍
    Eigen::Index k;
    weld_at_ee.tail<3>().cwiseAbs().maxCoeff(&k);
    const double couple_true = weld_at_ee[3 + k];
    ASSERT_GT(std::abs(couple_true), 0.5) << armName(a);
    EXPECT_NEAR(as.ft_ee_world[3 + k] / couple_true, 2.0, 0.05) << armName(a);
  }
}

TEST(WeldWrench, BalancesGravityAndDisturbance) {
  // 1.0–3.0 s 对杆施加 F = (0, 0, −20) N 与 M = (1.5, 0, 0) N·m。关掉接触：杆离台面只有 1 mm，扰动会把它压到台面上
  const SimConfig cfg = test::testConfig({"simulation.contacts=false"});
  SimConfig cfg_d = cfg;
  DisturbanceConfig dist;
  dist.t_start = 1.0;
  dist.t_end = 3.0;
  dist.force = Vector3d(0, 0, -20);
  dist.torque = Vector3d(1.5, 0, 0);
  cfg_d.disturbances = {dist};

  SimEnv env(cfg_d);
  auto model = std::make_shared<MujocoRobotModel>(cfg_d);
  GravityCompJointPD ctrl(model, cfg_d.gravity_pd);
  ctrl.reset(env.state());

  runPd(env, ctrl, 900);  // t = 0.9 s：无扰动
  const double mg = env.scene().object.mass * 9.81;
  Wrench sum = test::sumWeldWrenchAtObjectCenter(env, env.lastStep());
  EXPECT_NEAR(sum[2], mg, 0.05);

  runPd(env, ctrl, 1900);  // t = 2.8 s：扰动已作用 1.8 s，已稳定
  const DualArmState& s = env.lastStep();
  EXPECT_LT((s.disturbance - (Wrench() << dist.force, dist.torque).finished()).norm(), 1e-12);
  sum = test::sumWeldWrenchAtObjectCenter(env, s);
  // 静力平衡：两臂合 wrench + 重力 + 扰动 = 0
  EXPECT_NEAR(sum[2], mg + 20.0, 0.1);
  EXPECT_NEAR(sum[3], -1.5, 0.03);
  EXPECT_LT(Vector3d(sum[0], sum[1], 0).norm(), 0.1);
  EXPECT_LT(Vector3d(0, sum[4], sum[5]).norm(), 0.03);
}

TEST(ObjectState, TwistMatchesMujocoObjectVelocity) {
  const SimConfig cfg = test::testConfig();
  SimConfig cfg_d = cfg;
  DisturbanceConfig dist;
  dist.t_start = 0.0;
  dist.t_end = 1.0;
  dist.force = Vector3d(5, 10, 0);
  dist.torque = Vector3d(0, 0, 2.0);
  cfg_d.disturbances = {dist};
  SimEnv env(cfg_d);
  auto model = std::make_shared<MujocoRobotModel>(cfg_d);
  GravityCompJointPD ctrl(model, cfg_d.gravity_pd);
  ctrl.reset(env.state());
  runPd(env, ctrl, 100);

  mjtNum rotlin[6];
  mj_objectVelocity(env.model(), env.data(), mjOBJ_BODY, env.indices().object_body, rotlin, 0);
  const Twist v = env.state().object.twist;
  EXPECT_GT(v.norm(), 1e-3);  // 确实在动
  EXPECT_LT((v.head<3>() - Vector3d(rotlin[3], rotlin[4], rotlin[5])).norm(), 1e-9);
  EXPECT_LT((v.tail<3>() - Vector3d(rotlin[0], rotlin[1], rotlin[2])).norm(), 1e-9);
}
