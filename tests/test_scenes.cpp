// 场景级验收：
//  1) 四个场景在 GravityCompJointPD 下稳定运行 5 s（不发散、物体漂移小、约束力有限）；
//  2) slot 判别性：平放的板落不进槽（被槽壁托住），滚转 90° 的板能落到槽底；
//     把平放的板硬放到插入深度时与槽壁穿透 > 1 cm；
//  3) assembly：右臂 joint7 施加拧紧力矩 → 螺钉转动并按导程进给（feed = lead·|angle|/2π），
//     座面贴合后停住；左臂（持握臂）腕部测到绕螺纹轴的反力矩，与右臂施加的力矩大小相等、方向相反。
#include "dual_arm/baseline_controllers.hpp"
#include "dual_arm/mujoco_model.hpp"
#include "dual_arm/scene_monitor.hpp"
#include "dual_arm/sim_env.hpp"
#include "test_common.hpp"

#include <gtest/gtest.h>

#include <cmath>
#include <memory>

using namespace dual_arm;

namespace {

void runPd(SimEnv& env, GravityCompJointPD& ctrl, int n) {
  for (int k = 0; k < n; ++k) {
    const auto [tl, tr] = ctrl.compute(env.state(), env.time());
    env.step(tl, tr);
  }
}

}  // namespace

// ---------------------------------------------------------------------------
// 1) 5 s 稳定性
// ---------------------------------------------------------------------------
class SceneStability : public ::testing::TestWithParam<std::string> {};

TEST_P(SceneStability, GravityPdHoldsFor5Seconds) {
  // 不加扰动（assembly_simple 的力矩剖面单独测试）
  const SimConfig cfg = test::testConfig({}, GetParam());
  SimEnv env(cfg);
  auto model = std::make_shared<MujocoRobotModel>(cfg);
  GravityCompJointPD ctrl(model, cfg.gravity_pd);
  ctrl.reset(env.state());
  const Pose T0 = env.state().object.pose;
  double max_drift = 0.0, max_weld = 0.0, max_rot = 0.0;
  for (int k = 0; k < 5000; ++k) {
    const auto [tl, tr] = ctrl.compute(env.state(), env.time());
    env.step(tl, tr);
    ASSERT_FALSE(env.diverged()) << GetParam() << " diverged at t = " << env.time();
    const DualArmState& s = env.lastStep();
    max_drift = std::max(max_drift, (s.object.pose.p - T0.p).norm());
    max_rot = std::max(max_rot, rotationError(s.object.pose.R(), T0.R()).norm());
    for (Arm a : kArms) max_weld = std::max(max_weld, s.arm(a).weld_wrench.head<3>().norm());
  }
  EXPECT_LT(max_drift, 0.01) << GetParam() << " object drift [m]";
  EXPECT_LT(max_rot, 0.02) << GetParam() << " object rotation [rad]";
  EXPECT_LT(max_weld, 60.0) << GetParam() << " |weld force| [N]";
  EXPECT_LT(env.state().object.twist.norm(), 0.01) << GetParam();
  std::printf("[stability] %-16s max drift %.2f mm, max rotation %.3f deg, max |weld force| %.2f N\n",
              GetParam().c_str(), 1e3 * max_drift, max_rot * 180 / M_PI, max_weld);
}

INSTANTIATE_TEST_SUITE_P(AllScenes, SceneStability, ::testing::ValuesIn(test::allScenes()),
                         [](const ::testing::TestParamInfo<std::string>& p) { return p.param; });

// ---------------------------------------------------------------------------
// 2) slot 判别性
// ---------------------------------------------------------------------------
namespace {

struct SlotGeometry {
  double mouth_z = 0.0, bottom_z = 0.0, center_y = 0.0;
};

SlotGeometry slotGeometry(const mjModel* m, const mjData* d) {
  const int base = mj_name2id(m, mjOBJ_GEOM, "slot_base");
  const int wall = mj_name2id(m, mjOBJ_GEOM, "slot_wall_front");
  SlotGeometry g;
  g.bottom_z = d->geom_xpos[3 * base + 2] + m->geom_size[3 * base + 2];
  g.mouth_z = d->geom_xpos[3 * wall + 2] + m->geom_size[3 * wall + 2];
  g.center_y = d->geom_xpos[3 * base + 1];
  return g;
}

/// 松开板（关掉两个 weld），把它放到槽口正上方 z_bottom（板最低点）处，以滚转角 roll 自由下落 1.5 s，返回板最低点高度
double dropPlate(double roll_deg) {
  const SimConfig cfg = test::testConfig({}, "slot");
  SimEnv env(cfg);
  auto model = std::make_shared<MujocoRobotModel>(cfg);
  GravityCompJointPD ctrl(model, cfg.gravity_pd);
  ctrl.reset(env.state());
  mjData* d = env.data();
  const SlotGeometry sg = slotGeometry(env.model(), d);
  for (Arm a : kArms) d->eq_active[env.indices()[a].weld_eq] = 0;
  const Vector3d half = env.scene().object.half_size;
  const Quaterniond q = quatFromRpyDeg(Vector3d(roll_deg, 0, 0));
  const double half_h = std::abs(q.toRotationMatrix().row(2).dot(Vector3d(0, half.y(), 0))) +
                        std::abs(q.toRotationMatrix().row(2).dot(Vector3d(0, 0, half.z())));
  mjtNum* qp = d->qpos + env.indices().object_qpos_adr;
  qp[0] = 0.0;
  qp[1] = sg.center_y;
  qp[2] = sg.mouth_z + 0.03 + half_h;
  qp[3] = q.w();
  qp[4] = q.x();
  qp[5] = q.y();
  qp[6] = q.z();
  env.forward();
  runPd(env, ctrl, 1500);
  EXPECT_FALSE(env.diverged());
  const Pose T = env.state().object.pose;
  double zmin = 1e9;
  for (int k = 0; k < 8; ++k) {
    const Vector3d c((k & 1 ? 1 : -1) * half.x(), (k & 2 ? 1 : -1) * half.y(), (k & 4 ? 1 : -1) * half.z());
    zmin = std::min(zmin, T.transformPoint(c).z());
  }
  return zmin;
}

}  // namespace

TEST(SlotDiscriminative, FlatPlateRestsOnSlotWalls) {
  const SimConfig cfg = test::testConfig({}, "slot");
  SimEnv env(cfg);
  const SlotGeometry sg = slotGeometry(env.model(), env.data());
  const double zmin = dropPlate(0.0);
  EXPECT_NEAR(zmin, sg.mouth_z, 3e-3) << "flat plate should be stopped by the wall tops";
}

TEST(SlotDiscriminative, RolledPlateDropsToSlotBottom) {
  const SimConfig cfg = test::testConfig({}, "slot");
  SimEnv env(cfg);
  const SlotGeometry sg = slotGeometry(env.model(), env.data());
  const Eigen::AngleAxisd aa(env.scene().waypoints.back().pose.q);  // 最终航点：绕 ±x 滚转
  const double roll = aa.angle() * (aa.axis().x() >= 0 ? 1.0 : -1.0) * 180.0 / M_PI;
  EXPECT_GT(std::abs(roll), 60.0);
  const double zmin = dropPlate(roll);
  EXPECT_NEAR(zmin, sg.bottom_z, 3e-3) << "rolled plate should slide down to the slot bottom";
}

TEST(SlotDiscriminative, UnrolledPlateInterferesAtInsertedPose) {
  const SimConfig cfg = test::testConfig({}, "slot");
  SimEnv env(cfg);
  mjData* d = env.data();
  const mjModel* m = env.model();
  const SlotGeometry sg = slotGeometry(m, d);
  const Vector3d half = env.scene().object.half_size;
  const int plate = mj_name2id(m, mjOBJ_GEOM, env.scene().object.geom.c_str());
  auto frameDistance = [&](double roll_deg) {
    const Quaterniond q = quatFromRpyDeg(Vector3d(roll_deg, 0, 0));
    const Matrix3d R = q.toRotationMatrix();
    const double half_h = std::abs(R(2, 1)) * half.y() + std::abs(R(2, 2)) * half.z();
    mjtNum* qp = d->qpos + env.indices().object_qpos_adr;
    qp[0] = 0.0;
    qp[1] = sg.center_y;
    qp[2] = sg.bottom_z + 0.001 + half_h;  // 最低点在槽底上方 1 mm（插到底）
    qp[3] = q.w();
    qp[4] = q.x();
    qp[5] = q.y();
    qp[6] = q.z();
    mj_kinematics(m, d);
    double dmin = 1e9;
    for (const char* g : {"slot_base", "slot_wall_front", "slot_wall_back"}) {
      dmin = std::min(dmin, mj_geomDistance(m, d, plate, mj_name2id(m, mjOBJ_GEOM, g), 0.2, nullptr));
    }
    return dmin;
  };
  const double d_flat = frameDistance(0.0);
  const double d_rolled = frameDistance(-90.0);
  std::printf("[slot] plate-frame signed distance at the inserted pose: flat %.1f mm, rolled -90 deg %.1f mm\n",
              1e3 * d_flat, 1e3 * d_rolled);
  EXPECT_LT(d_flat, -0.01);
  EXPECT_GT(d_rolled, 0.0);
}

// ---------------------------------------------------------------------------
// 3) assembly：手动拧紧
// ---------------------------------------------------------------------------
TEST(Assembly, TorqueOnRightWristTurnsAndFeedsTheScrew) {
  // 右臂 joint7 不做位置保持（kp7 = kd7 = 0），加一个恒定拧紧力矩 τ7；其余关节 PD 保持。
  // 工具轴（= joint7 轴 = TCP z）朝下，所以 τ7 > 0 绕世界 −z，即从上往下看顺时针 = 拧紧（螺钉转角为负）。
  const double tau7 = 1.0;
  SimConfig cfg = test::testConfig({}, "assembly");
  cfg.gravity_pd.kp[1][6] = 0.0;
  cfg.gravity_pd.kd[1][6] = 0.0;
  cfg.gravity_pd.extra_torque[1][6] = tau7;
  SimEnv env(cfg);
  auto model = std::make_shared<MujocoRobotModel>(cfg);
  GravityCompJointPD ctrl(model, cfg.gravity_pd);
  ctrl.reset(env.state());
  SceneMonitor mon(env, false);
  const ClosedChainConstraint* screw = env.scene().screwConstraint();
  ASSERT_NE(screw, nullptr);
  const double lead = screw->lead;

  // 1 s：自由旋入阶段（未贴合），进给 = 导程 × 转数
  runPd(env, ctrl, 1000);
  const ScrewState s1 = env.state().screw;
  ASSERT_TRUE(s1.valid);
  EXPECT_LT(s1.angle, -0.5) << "positive joint7 torque must tighten (negative angle)";
  EXPECT_LT(s1.feed, env.scene().screw.seat_depth);
  EXPECT_LT(std::abs(s1.lead_error), 2e-5) << "thread coupling violation [m]";
  EXPECT_NEAR(s1.feed, lead * std::abs(s1.angle) / (2 * M_PI), 2e-5) << "feed must follow the thread lead";

  // 到 5 s：螺钉头贴合座面后停住
  runPd(env, ctrl, 4000);
  ASSERT_FALSE(env.diverged());
  const DualArmState& rec = env.lastStep();
  mon.update(rec);
  const ScrewState& s = rec.screw;
  std::printf("[assembly] tau7 %.2f: angle %.1f deg, feed %.4f mm, lead err %.4f mm, tau_seat %.3f, fric %.3f, "
              "hold_ft %.3f, hold_weld %.3f, work_ft %.3f, work_weld %.3f N·m, workpiece drift rz %.4f rad\n",
              tau7, s.angle * 180 / M_PI, 1e3 * s.feed, 1e3 * s.lead_error, s.tau_seat, s.tau_friction,
              mon.value("hold_ft_axis"), mon.value("hold_weld_axis"), mon.value("work_ft_axis"),
              mon.value("work_weld_axis"), mon.value("obj_drift_rz"));
  EXPECT_GT(s.feed, env.scene().screw.seat_depth);            // 已贴合
  EXPECT_LT(std::abs(s.rate), 0.02);                          // 停住
  EXPECT_NEAR(s.tau_seat, tau7, env.scene().screw.frictionloss + 0.02);  // 座面阻力矩 ≈ 拧紧力矩（差值 ≤ 静摩擦）
  // 作业臂经工具施加绕螺纹轴（+z）的力矩 ≈ −τ7，持握臂承受大小相等、方向相反的反力矩
  EXPECT_NEAR(mon.value("work_weld_axis"), -tau7, 0.03);
  EXPECT_NEAR(mon.value("hold_weld_axis"), tau7, 0.03);
  // 左腕 F/T（扣夹爪重力后）测到同一个反力矩；右腕同理
  EXPECT_NEAR(mon.value("hold_ft_axis"), mon.value("hold_weld_axis"), 0.02);
  EXPECT_NEAR(mon.value("work_ft_axis"), mon.value("work_weld_axis"), 0.02);
  // 工件绕螺纹轴有小的转动漂移（持握臂只有关节 PD），其余方向更小
  EXPECT_LT(std::abs(mon.value("obj_drift_rz")), 0.03);
}

TEST(AssemblySimple, TorqueProfileIsSharedByBothArms) {
  // assembly_simple：工具与工件刚性相连（相对自由度 0），场景 overrides 中的力矩剖面（绕世界 z −3 N·m，
  // t = 1–5 s，1 s 斜坡）作用在工件上；静止时两臂经 weld 施加的绕轴力矩之和与之平衡
  const SimConfig cfg = loadConfig("config/default.yaml", {"log.enabled=false"}, "assembly_simple");
  ASSERT_EQ(cfg.disturbances.size(), 1u);
  const DisturbanceConfig& dc = cfg.disturbances[0];
  SimEnv env(cfg);
  auto model = std::make_shared<MujocoRobotModel>(cfg);
  GravityCompJointPD ctrl(model, cfg.gravity_pd);
  ctrl.reset(env.state());
  SceneMonitor mon(env, false);
  EXPECT_NEAR(dc.profile(0.5), 0.0, 1e-12);
  EXPECT_NEAR(dc.profile(1.5), 0.5, 1e-12);
  EXPECT_NEAR(dc.profile(3.0), 1.0, 1e-12);
  runPd(env, ctrl, 3500);  // t = 3.5 s：满幅已持续 1.5 s
  ASSERT_FALSE(env.diverged());
  const DualArmState& rec = env.lastStep();
  mon.update(rec);
  EXPECT_LT((rec.disturbance.tail<3>() - dc.torque).norm(), 1e-9);
  const double sum = mon.value("hold_weld_axis") + mon.value("work_weld_axis");
  std::printf("[assembly_simple] disturbance %.2f N·m about z: hold %.3f + work %.3f = %.3f N·m, drift rz %.4f rad\n",
              dc.torque.z(), mon.value("hold_weld_axis"), mon.value("work_weld_axis"), sum, mon.value("obj_drift_rz"));
  EXPECT_NEAR(sum, -dc.torque.z(), 0.05);
  EXPECT_GT(mon.value("hold_weld_axis"), 0.0);
  EXPECT_NEAR(mon.value("hold_ft_axis"), mon.value("hold_weld_axis"), 0.03);
}

TEST(Assembly, DriftDecompositionIsExact) {
  Pose T0;
  T0.p = Vector3d(0.1, -0.4, 0.9);
  T0.q = quatFromRpyDeg(Vector3d(10, -20, 30));
  const Matrix3d Rf = T0.R();
  Pose T = T0;
  T.p += Rf * Vector3d(0.001, -0.002, 0.003);
  T.q = (Quaterniond(Eigen::AngleAxisd(0.01, Rf.col(2))) * T0.q).normalized();
  const Vector6d dd = decomposePoseDrift(T0, T, Rf);
  EXPECT_LT((dd.head<3>() - Vector3d(0.001, -0.002, 0.003)).norm(), 1e-12);
  EXPECT_LT((dd.tail<3>() - Vector3d(0, 0, 0.01)).norm(), 1e-12);
}
