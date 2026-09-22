// 航点插值器（ObjectTrajectory::Waypoints + TrapezoidProfile）：
//  - 梯形剖面到达终点、不超速度 / 加速度上限、速度连续；
//  - 轨迹经过每个航点（段结束时刻）并在停留期间静止；
//  - twist / accel 与位姿 / twist 的数值微分一致；各段的平移 / 转动 / 螺钉角速度都不超限、同步结束。
#include "dual_arm/math_utils.hpp"
#include "dual_arm/object_trajectory.hpp"
#include "test_common.hpp"

#include <gtest/gtest.h>

using namespace dual_arm;

TEST(TrapezoidProfile, ReachesEndWithinLimits) {
  for (const auto& [L, v, a] : {std::tuple{1.0, 0.5, 1.0}, std::tuple{0.1, 0.5, 1.0}, std::tuple{2.0, 0.3, 5.0}}) {
    const TrapezoidProfile p = TrapezoidProfile::make(L, v, a);
    double s, ds, dds, prev_ds = 0.0;
    for (double t = 0.0; t <= p.T + 1e-9; t += 1e-4) {
      p.sample(t, s, ds, dds);
      EXPECT_LE(ds, v + 1e-9);
      EXPECT_LE(std::abs(dds), a + 1e-9);
      EXPECT_LT(std::abs(ds - prev_ds), a * 1e-4 + 1e-9);  // 速度连续
      prev_ds = ds;
    }
    p.sample(p.T, s, ds, dds);
    EXPECT_NEAR(s, L, 1e-12);
    EXPECT_NEAR(ds, 0.0, 1e-12);
  }
}

TEST(ObjectTrajectory, PassesThroughSceneWaypoints) {
  for (const std::string scene : {"lift", "slot", "assembly"}) {
    const SimConfig cfg = test::testConfig({}, scene);
    const auto& wps = cfg.scene.waypoints;
    ObjectTrajectory traj(cfg.object_trajectory, wps.front().pose, wps);
    const auto& segs = traj.segments();
    ASSERT_EQ(segs.size(), wps.size() - 1);
    const auto& lim = cfg.object_trajectory.waypoints;
    for (std::size_t k = 0; k < segs.size(); ++k) {
      // 段结束 + 停留期间：位于航点 k+1、静止
      for (double dt : {0.0, 0.5 * lim.dwell}) {
        const ObjectReference r = traj.evaluate(segs[k].t0 + segs[k].T + dt);
        EXPECT_LT((r.pose.p - wps[k + 1].pose.p).norm(), 1e-9) << scene << " " << wps[k + 1].name;
        EXPECT_LT(rotationError(r.pose.R(), wps[k + 1].pose.R()).norm(), 1e-9) << scene << " " << wps[k + 1].name;
        EXPECT_LT(r.twist.norm(), 1e-9);
        if (wps[k + 1].has_screw_angle) {
          EXPECT_NEAR(r.screw_angle, wps[k + 1].screw_angle, 1e-9);
        }
      }
      // 段内：速度上限与数值微分
      const double h = 1e-5;
      for (int i = 1; i < 20; ++i) {
        const double t = segs[k].t0 + segs[k].T * i / 20.0;
        const ObjectReference r = traj.evaluate(t);
        EXPECT_LE(r.twist.head<3>().norm(), lim.v_max + 1e-9);
        EXPECT_LE(r.twist.tail<3>().norm(), lim.w_max + 1e-9);
        EXPECT_LE(std::abs(r.screw_rate), lim.w_max + 1e-9);
        EXPECT_LE(r.accel.head<3>().norm(), lim.a_max + 1e-9);
        EXPECT_LE(r.accel.tail<3>().norm(), lim.alpha_max + 1e-9);
        const ObjectReference rp = traj.evaluate(t + h), rm = traj.evaluate(t - h);
        Twist fd;
        fd << (rp.pose.p - rm.pose.p) / (2 * h), rotationError(rp.pose.R(), rm.pose.R()) / (2 * h);
        EXPECT_LT((fd - r.twist).norm(), 1e-5) << scene << " segment " << k << " t " << t;
        // 避开加速段 / 匀速段 / 减速段的切换点（加速度在那里跳变）
        const double tau = t - segs[k].t0, ta = segs[k].prof.t_acc, tf = segs[k].prof.t_acc + segs[k].prof.t_flat;
        if (std::abs(tau - ta) > 2 * h && std::abs(tau - tf) > 2 * h) {
          EXPECT_LT(((rp.twist - rm.twist) / (2 * h) - r.accel).norm(), 1e-4) << scene << " segment " << k;
        }
      }
    }
    EXPECT_NEAR(traj.endTime(), segs.back().t0 + segs.back().T + lim.dwell, 1e-12);
    // 开始之前保持初始位姿
    EXPECT_LT((traj.evaluate(0.0).pose.p - wps.front().pose.p).norm(), 1e-12);
  }
}
