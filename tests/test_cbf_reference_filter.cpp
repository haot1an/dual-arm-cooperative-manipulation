// CbfReferenceFilter 的验收测试（docs/cbf_reference_governor.md §8）：
//   (1) 无障碍时不改变原始参考；
//   (2) 屏障前向不变：理想跟踪下 h ≥ 0 全程成立；
//   (3) 有限高墙：不死锁，越过后偏移回到 0；
//   (4) 偏移、速度、变化率与 ṡ 的界；
//   (5) 场景级：slot_avoid 用 cbf 模式完成越墙与插槽；
//   (6) 控制循环零动态分配（见 test_no_alloc.cpp）。
#include "dual_arm/cbf_reference_filter.hpp"
#include "dual_arm/math_utils.hpp"
#include "dual_arm/mujoco_model.hpp"
#include "dual_arm/qp_coop_controller.hpp"
#include "test_common.hpp"

#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <limits>

using namespace dual_arm;

namespace
{
  using Config = CbfReferenceFilter::Config;

  Config defaultCbfConfig()
  {
    return test::testConfig({}, "slot_avoid").torque_qp.reference_governor.cbf;
  }

  /// 轴对齐盒子障碍：点到盒子的有符号距离与“点 → 盒子”单位法向。
  struct Box
  {
    Vector3d lo, hi;
    CbfObstacle distanceFrom(const Vector3d &p) const
    {
      CbfObstacle o;
      o.point = p;
      const Vector3d closest = p.cwiseMax(lo).cwiseMin(hi);
      const Vector3d diff = closest - p;
      if (diff.norm() > 1e-12)
      {
        o.distance = diff.norm();
        o.normal = diff / o.distance;
        return o;
      }
      // 点在盒内：取最近的面，距离为负
      double best = std::numeric_limits<double>::infinity();
      for (int k = 0; k < 3; ++k)
      {
        for (int side = 0; side < 2; ++side)
        {
          const double depth = side == 0 ? p[k] - lo[k] : hi[k] - p[k];
          if (depth < best)
          {
            best = depth;
            o.normal = Vector3d::Zero();
            o.normal[k] = side == 0 ? 1.0 : -1.0;  // 物体要沿 −normal 离开，normal 指向盒内
          }
        }
      }
      o.distance = -best;
      return o;
    }
  };

  /// 原始参考：沿 +x 匀速 0.08 m/s 的直线，姿态不变。
  ObjectReference straightLine(double s)
  {
    ObjectReference r;
    r.pose.p = Vector3d(0.08 * s, 0.0, 0.10);
    r.twist.head<3>() = Vector3d(0.08, 0.0, 0.0);
    return r;
  }

  struct RunStats
  {
    double min_barrier = std::numeric_limits<double>::infinity();
    double max_offset = 0.0;
    double max_speed = 0.0;
    double max_rate_change = 0.0;
    double max_time_rate_change = 0.0;
    double min_time_rate = 1.0, max_time_rate = 0.0;
    Vector3d final_position = Vector3d::Zero();
    Vector3d final_offset = Vector3d::Zero();
  };

  /// 理想跟踪（实测物体 = 上一周期参考），点状物体对一个盒子障碍，距离 < range 时送入滤波器。
  RunStats simulate(const Config &cfg, const Box &box, double duration, double range = 0.09)
  {
    const double dt = 0.001;
    CbfReferenceFilter filter(cfg, dt);
    RunStats st;
    Pose actual = straightLine(0.0).pose;
    CbfReferenceFilter::Decision previous;
    for (int k = 0; k < static_cast<int>(duration / dt); ++k)
    {
      CbfReferenceFilter::Obstacles obstacles;
      int count = 0;
      const CbfObstacle o = box.distanceFrom(actual.p);
      if (o.distance < range)
        obstacles[count++] = o;
      st.min_barrier = std::min(st.min_barrier, o.distance - cfg.safe_distance);

      const ObjectReference &ref = filter.update(straightLine(filter.virtualTime()), actual, obstacles, count);
      const auto &u = filter.decision();
      st.max_offset = std::max(st.max_offset, filter.offset().norm());
      st.max_speed = std::max(st.max_speed, u.offset_rate.cwiseAbs().maxCoeff());
      st.max_rate_change = std::max(st.max_rate_change, (u.offset_rate - previous.offset_rate).cwiseAbs().maxCoeff());
      st.max_time_rate_change = std::max(st.max_time_rate_change, std::abs(u.time_rate - previous.time_rate));
      st.min_time_rate = std::min(st.min_time_rate, u.time_rate);
      st.max_time_rate = std::max(st.max_time_rate, u.time_rate);
      previous = u;
      actual = ref.pose;
    }
    st.final_position = actual.p;
    st.final_offset = filter.offset();
    return st;
  }

  // 墙：x ∈ [0.30, 0.34]，y 两侧无限；finite 顶面 z = 0.20（物体在 z = 0.10，须抬高 ≥ 0.10 + d_s）
  const Box kFiniteWall{Vector3d(0.30, -1.0, -1.0), Vector3d(0.34, 1.0, 0.20)};
  const Box kInfiniteWall{Vector3d(0.30, -1.0, -1.0), Vector3d(0.34, 1.0, 10.0)};

} // namespace

TEST(CbfReferenceFilter, PassesNominalReferenceThroughWithoutObstacles)
{
  CbfReferenceFilter filter(defaultCbfConfig(), 0.001);
  for (int k = 0; k < 2000; ++k)
  {
    const ObjectReference nominal = straightLine(filter.virtualTime());
    const ObjectReference &ref = filter.update(nominal, nominal.pose, CbfReferenceFilter::Obstacles{}, 0);
    ASSERT_LT((ref.pose.p - nominal.pose.p).norm(), 1e-9);
    ASSERT_LT((ref.twist - nominal.twist).norm(), 1e-9);
  }
  EXPECT_NEAR(filter.virtualTime(), 2.0, 1e-9);
  EXPECT_EQ(filter.status(), CbfReferenceFilter::Status::Solved);
  EXPECT_FALSE(filter.engaged());
}

TEST(CbfReferenceFilter, BarrierStaysNonNegativeInFrontOfInfiniteWall)
{
  const Config cfg = defaultCbfConfig();
  const RunStats st = simulate(cfg, kInfiniteWall, 10.0);
  EXPECT_GE(st.min_barrier, -1e-4);          // 前向不变（离散化误差 0.1 mm 内）
  EXPECT_LE(st.max_offset, cfg.offset_max + 1e-6);
  EXPECT_LT(st.final_position.x(), 0.30);    // 越不过无限高的墙：停在墙前
}

TEST(CbfReferenceFilter, ClimbsOverFiniteWallWithoutDeadlockAndReturnsToPath)
{
  const Config cfg = defaultCbfConfig();
  const RunStats st = simulate(cfg, kFiniteWall, 14.0);
  EXPECT_GE(st.min_barrier, -1e-4);
  EXPECT_GT(st.final_position.x(), 0.60);           // 越过墙（14 s 原轨迹应到 x = 1.12）
  EXPECT_GT(st.max_offset, 0.10);                   // 确实抬过了墙顶
  EXPECT_LT(st.final_offset.norm(), 1e-3);          // 越过后偏移回到原路径
}

TEST(CbfReferenceFilter, RespectsOffsetRateAndTimeScalingBounds)
{
  const Config cfg = defaultCbfConfig();
  const double dt = 0.001;
  const RunStats st = simulate(cfg, kFiniteWall, 14.0);
  EXPECT_LE(st.max_offset, cfg.offset_max + 1e-6);
  EXPECT_LE(st.max_speed, cfg.offset_speed_max + 1e-9);
  EXPECT_LE(st.max_rate_change, cfg.offset_accel_max * dt + 1e-9);
  EXPECT_LE(st.max_time_rate_change, cfg.time_rate_accel_max * dt + 1e-9);
  EXPECT_GE(st.min_time_rate, -1e-12);
  EXPECT_LE(st.max_time_rate, 1.0 + 1e-12);
}

TEST(CbfReferenceFilter, SlotAvoidTransportClearsBarrierAndInserts)
{
  const SimConfig cfg = test::testConfig(
      {
          "controller.type=qp_coop",
          "controller.torque_qp.reference_governor.mode=cbf",
          "object_trajectory.type=waypoints",
          "simulation.contacts=false",
      },
      "slot_avoid");
  SimEnv env(cfg);
  auto model = std::make_shared<MujocoRobotModel>(cfg);
  auto trajectory =
      std::make_shared<ObjectTrajectory>(cfg.object_trajectory, env.state().object.pose, env.scene().waypoints);
  QpCoopController controller(model, trajectory, cfg.coop, cfg.torque_qp, cfg.collision, cfg.timestep);
  controller.reset(env.state());
  ASSERT_NE(controller.cbfFilter(), nullptr);

  double min_barrier = std::numeric_limits<double>::infinity();
  double max_offset = 0.0;
  for (int k = 0; k < 24000; ++k)
  {
    const auto [tau_left, tau_right] = controller.compute(env.state(), env.time());
    ASSERT_TRUE(tau_left.allFinite() && tau_right.allFinite());
    min_barrier = std::min(min_barrier, controller.cbfFilter()->minBarrier());
    max_offset = std::max(max_offset, controller.governorOffset());
    env.step(tau_left, tau_right);
  }
  const Vector6d final_error = poseError(env.scene().waypoints.back().pose, env.state().object.pose);
  EXPECT_FALSE(env.diverged());
  EXPECT_EQ(controller.qpStatus(), JointSafetyTorqueQp::Status::Solved);
  EXPECT_GE(min_barrier, -0.0005);  // 实测物体 ~ 墙距离 ≥ d_s − 0.5 mm（含跟踪误差）
  EXPECT_GT(max_offset, 0.05);      // 确实绕行了
  EXPECT_LT(controller.governorOffset(), 1e-3);
  EXPECT_LT(final_error.head<3>().norm(), 0.002);
  EXPECT_LT(final_error.tail<3>().norm(), 0.01);
}

// ---------------------------------------------------------------------------
// small_qp：对角 Hessian QP（LDP / NNLS）与穷举有效集的暴力解对照
// ---------------------------------------------------------------------------
#include "dual_arm/small_qp.hpp"

#include <Eigen/Dense>
#include <random>

namespace
{
  /// 穷举所有 ≤ n 行的等式子集，取原始可行且代价最小者 —— 即 QP 的最优解。
  bool bruteForceQp(const small_qp::RowMatrix &A, const small_qp::RowVector &b, int m, int n,
                    const small_qp::VarVector &w, const small_qp::VarVector &t, Eigen::VectorXd &best)
  {
    double best_cost = std::numeric_limits<double>::infinity();
    const Eigen::VectorXd W = w.head(n), T = t.head(n);
    for (int mask = 0; mask < (1 << m); ++mask)
    {
      std::vector<int> rows;
      for (int j = 0; j < m; ++j)
        if (mask & (1 << j))
          rows.push_back(j);
      if (static_cast<int>(rows.size()) > n)
        continue;
      const int r = static_cast<int>(rows.size());
      // KKT：W(x − t) + Crᵀ μ = 0，Cr x = br
      Eigen::MatrixXd K = Eigen::MatrixXd::Zero(n + r, n + r);
      Eigen::VectorXd rhs = Eigen::VectorXd::Zero(n + r);
      K.topLeftCorner(n, n) = W.asDiagonal();
      rhs.head(n) = W.cwiseProduct(T);
      for (int k = 0; k < r; ++k)
      {
        K.block(0, n + k, n, 1) = A.row(rows[k]).head(n).transpose();
        K.block(n + k, 0, 1, n) = A.row(rows[k]).head(n);
        rhs[n + k] = b[rows[k]];
      }
      Eigen::FullPivLU<Eigen::MatrixXd> lu(K);
      if (lu.rank() < n + r)
        continue;
      const Eigen::VectorXd x = lu.solve(rhs).head(n);
      bool feasible = true;
      for (int j = 0; j < m; ++j)
        feasible &= A.row(j).head(n).dot(x) <= b[j] + 1e-9;
      if (!feasible)
        continue;
      const double cost = 0.5 * (x - T).cwiseProduct(W).dot(x - T);
      if (cost < best_cost)
      {
        best_cost = cost;
        best = x;
      }
    }
    return std::isfinite(best_cost);
  }
} // namespace

TEST(SmallQp, MatchesBruteForceOnRandomProblems)
{
  std::mt19937 rng(11);
  std::uniform_real_distribution<double> u(-1.0, 1.0), pos(0.05, 2.0);
  int feasible_cases = 0;
  for (int trial = 0; trial < 400; ++trial)
  {
    const int n = 1 + trial % small_qp::kMaxVars;
    const int m = 1 + (trial / 4) % 8;
    small_qp::RowMatrix A = small_qp::RowMatrix::Zero();
    small_qp::RowVector b = small_qp::RowVector::Zero();
    small_qp::VarVector w = small_qp::VarVector::Ones(), t = small_qp::VarVector::Zero();
    for (int i = 0; i < n; ++i)
    {
      w[i] = pos(rng);
      t[i] = 2.0 * u(rng);
    }
    for (int j = 0; j < m; ++j)
    {
      for (int i = 0; i < n; ++i)
        A(j, i) = u(rng);
      b[j] = u(rng);
    }
    small_qp::VarVector x;
    const small_qp::Result result = small_qp::solveDiagonalQp(A, b, m, n, w, t, x);
    Eigen::VectorXd reference;
    const bool reference_feasible = bruteForceQp(A, b, m, n, w, t, reference);
    ASSERT_EQ(result.feasible, reference_feasible) << "trial " << trial;
    if (!reference_feasible)
      continue;
    ++feasible_cases;
    EXPECT_LT((x.head(n) - reference).norm(), 1e-7) << "trial " << trial;
  }
  EXPECT_GT(feasible_cases, 200);  // 随机问题约 2/3 可行；不可行者也已与暴力解对照（feasible 标志一致）
}

TEST(SmallQp, ReportsInfeasibility)
{
  small_qp::RowMatrix A = small_qp::RowMatrix::Zero();
  small_qp::RowVector b = small_qp::RowVector::Zero();
  A(0, 0) = 1.0;  // x ≤ −1
  b[0] = -1.0;
  A(1, 0) = -1.0; // x ≥ 1
  b[1] = -1.0;
  small_qp::VarVector x;
  const small_qp::Result result = small_qp::solveDiagonalQp(
      A, b, 2, 1, small_qp::VarVector::Ones(), small_qp::VarVector::Zero(), x);
  EXPECT_FALSE(result.feasible);
}
