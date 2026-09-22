// 标定误差注入：plant 与控制器模型使用不同的右臂基座位姿。
#include "dual_arm/baseline_controllers.hpp"
#include "dual_arm/math_utils.hpp"
#include "dual_arm/mujoco_model.hpp"
#include "dual_arm/sim_env.hpp"
#include "test_common.hpp"

#include <gtest/gtest.h>

#include <memory>

using namespace dual_arm;

namespace {

double weldViolation(const SimEnv& env, Arm a) {
  const mjData* d = env.data();
  double v = 0.0;
  for (int r = 0; r < d->nefc; ++r) {
    if (d->efc_type[r] == mjCNSTR_EQUALITY && d->efc_id[r] == env.indices()[a].weld_eq) {
      v = std::max(v, std::abs(d->efc_pos[r]));
    }
  }
  return v;
}

}  // namespace

TEST(Calibration, PlantBaseDiffersFromNominalByConfiguredOffset) {
  const SimConfig cfg = test::testConfig({"calibration_error.enabled=true", "simulation.contacts=false"});
  SimEnv env(cfg);
  MujocoRobotModel model(cfg);

  const Pose Tn = model.basePose(Arm::Right);
  const Pose Tt = env.basePose(Arm::Right);
  EXPECT_LT((Tt.p - Tn.p - cfg.calibration_error.right_base_offset_xyz).norm(), 1e-12);
  const Quaterniond q_err = quatFromRpyDeg(cfg.calibration_error.right_base_offset_rpy_deg);
  EXPECT_LT(rotationError((q_err * Tn.q).toRotationMatrix(), Tt.R()).norm(), 1e-12);
  // 左臂不受影响
  EXPECT_LT((env.basePose(Arm::Left).p - model.basePose(Arm::Left).p).norm(), 1e-12);

  // 相同关节角下：真实 ee 位姿 = T_err ∘（名义 ee 位姿，相对名义基座）
  model.update(env.state());
  const Pose ee_nom = model.eePose(Arm::Right);
  const Pose ee_true = env.state().arm(Arm::Right).ee_pose;
  const Vector3d expected = Tn.p + cfg.calibration_error.right_base_offset_xyz +
                            q_err * (ee_nom.p - Tn.p);
  EXPECT_LT((ee_true.p - expected).norm(), 1e-9);
  const double err_mm = 1e3 * (ee_true.p - ee_nom.p).norm();
  EXPECT_GT(err_mm, 1.0);  // 默认 3/3/2 mm + 0.3° yaw，末端误差为毫米级
  EXPECT_LT(err_mm, 20.0);
  const auto& L = env.state().arm(Arm::Left);
  model.update(env.state());
  EXPECT_LT((L.ee_pose.p - model.eePose(Arm::Left).p).norm(), 1e-12);
}

/// 用 GravityCompJointPD 保持初始构型运行 1 s，返回最后一步的两臂 weld wrench
std::array<Wrench, 2> holdAndGetWeldWrenches(const SimConfig& cfg, SimEnv& env) {
  auto model = std::make_shared<MujocoRobotModel>(cfg);
  GravityCompJointPD ctrl(model, cfg.gravity_pd);
  ctrl.reset(env.state());
  for (int k = 0; k < 1000; ++k) {
    const auto [tl, tr] = ctrl.compute(env.state(), env.time());
    env.step(tl, tr);
  }
  EXPECT_FALSE(env.diverged());
  return {env.lastStep().arm(Arm::Left).weld_wrench, env.lastStep().arm(Arm::Right).weld_wrench};
}

TEST(Calibration, CurrentInitModeHasNoPreload) {
  // 注意：即使没有标定误差，两臂在物体重力作用下的变形也会在闭链中产生内力。
  // 这里比较的是“有/无标定误差”的差别。关掉接触：杆离台面只有 1 mm。
  const SimConfig cfg0 = test::testConfig({"simulation.contacts=false"});
  SimEnv env0(cfg0);
  const auto w0 = holdAndGetWeldWrenches(cfg0, env0);

  const SimConfig cfg = test::testConfig({"calibration_error.enabled=true", "weld.init_mode=current", "simulation.contacts=false"});
  SimEnv env(cfg);
  // 右臂 ee_site 与抓取点之间存在毫米级偏差，但 relpose 吸收了它：t=0 约束误差为 0
  EXPECT_GT(env.initialGraspMismatch(Arm::Right).head<3>().norm(), 1e-3);
  EXPECT_LT(env.initialGraspMismatch(Arm::Left).norm(), 1e-5);  // q_init 写到 1e-6 rad → 微米级
  for (Arm a : kArms) EXPECT_LT(weldViolation(env, a), 1e-9) << armName(a);  // relpose = 当前相对位姿

  // 关节 PD 保持初始构型：标定误差不会带来额外内力
  const auto w = holdAndGetWeldWrenches(cfg, env);
  for (int i = 0; i < 2; ++i) {
    EXPECT_LT((w[i].head<3>() - w0[i].head<3>()).norm(), 0.2) << "arm " << i;
  }
}

TEST(Calibration, NominalInitModeCreatesPreload) {
  const SimConfig cfg0 = test::testConfig({"simulation.contacts=false"});
  SimEnv env0(cfg0);
  const auto w0 = holdAndGetWeldWrenches(cfg0, env0);

  // 不做过渡（nominal_ramp_time = 0）：右臂 weld 在 t=0 就被“拉开”了毫米级
  {
    const SimConfig cfg_now = test::testConfig(
        {"calibration_error.enabled=true", "weld.init_mode=nominal", "weld.nominal_ramp_time=0", "simulation.contacts=false"});
    SimEnv env_now(cfg_now);
    EXPECT_GT(weldViolation(env_now, Arm::Right), 1e-3);
    EXPECT_LT(weldViolation(env_now, Arm::Left), 1e-5);  // 左臂只有 q_init 舍入带来的微米级误差
  }
  // 默认 0.5 s 过渡：t=0 无约束误差，过渡结束后达到名义 relpose
  const SimConfig cfg = test::testConfig({"calibration_error.enabled=true", "weld.init_mode=nominal", "simulation.contacts=false"});
  SimEnv env(cfg);
  EXPECT_LT(weldViolation(env, Arm::Right), 1e-9);

  const auto w = holdAndGetWeldWrenches(cfg, env);
  // 装配误差被刚性闭链变成内力：两臂的附加力大小相等、方向相反，合力不变（仍只托住箱子重力）
  const Vector3d dl = w[0].head<3>() - w0[0].head<3>();
  const Vector3d dr = w[1].head<3>() - w0[1].head<3>();
  EXPECT_GT(dl.norm(), 2.0) << "additional internal force [N]";
  EXPECT_LT((dl + dr).norm(), 0.05);
}

TEST(Calibration, NominalRampAvoidsStartupSpike) {
  const SimConfig cfg = test::testConfig({"calibration_error.enabled=true", "weld.init_mode=nominal", "simulation.contacts=false"});
  SimEnv env(cfg);
  auto model = std::make_shared<MujocoRobotModel>(cfg);
  GravityCompJointPD ctrl(model, cfg.gravity_pd);
  ctrl.reset(env.state());
  double peak = 0.0;
  for (int k = 0; k < 1000; ++k) {
    const auto [tl, tr] = ctrl.compute(env.state(), env.time());
    env.step(tl, tr);
    for (Arm a : kArms) peak = std::max(peak, env.lastStep().arm(a).weld_wrench.head<3>().norm());
  }
  EXPECT_LT(peak, 30.0) << "peak |weld force| during the 0.5 s ramp [N]";
}
