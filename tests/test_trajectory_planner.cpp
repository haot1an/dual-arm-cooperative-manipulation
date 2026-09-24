// 物体空间轨迹优化（docs/trajectory_planning.md §8）：
//   (1) ADMM 稀疏 QP 与穷举有效集的暴力解一致；
//   (2) 路径变形样条：过节点、导数与差分一致、节点外为 0；
//   (3) 重新计时后平移速度 / 加速度不超过上限；
//   (4) slot_gate：规划成功（名义初值不可行，多初值中“向下”胜出）、偏移为下压、节点满足要求距离；
//   (5) 闭环：规划 + CBF 从横梁下方通过并插槽，跟踪误差小；
//   (6) 带姿态偏移与节点时间映射的 evaluate()：速度 / 角速度与数值微分一致；
//   (7) SE(3) + 时间联合优化：可行、使用了姿态自由度、明显缩短时长，闭环完成任务。
#include "dual_arm/collision_model.hpp"
#include "dual_arm/math_utils.hpp"
#include "dual_arm/mujoco_model.hpp"
#include "dual_arm/path_deformation.hpp"
#include "dual_arm/qp_coop_controller.hpp"
#include "dual_arm/sparse_qp.hpp"
#include "dual_arm/trajectory_optimizer.hpp"
#include "dual_arm/trajectory_planner.hpp"
#include "test_common.hpp"

#include <Eigen/Dense>
#include <gtest/gtest.h>

#include <cmath>
#include <limits>
#include <random>

using namespace dual_arm;

namespace
{
  /// 一般 P 的暴力解：约束 A x ≤ b，穷举 ≤ n 行等式子集，取可行且代价最小者。
  bool bruteForce(const Eigen::MatrixXd &P, const Eigen::VectorXd &q, const Eigen::MatrixXd &A,
                  const Eigen::VectorXd &b, Eigen::VectorXd &best)
  {
    const int n = static_cast<int>(P.rows()), m = static_cast<int>(A.rows());
    double best_cost = std::numeric_limits<double>::infinity();
    for (int mask = 0; mask < (1 << m); ++mask)
    {
      std::vector<int> rows;
      for (int j = 0; j < m; ++j)
        if (mask & (1 << j))
          rows.push_back(j);
      const int r = static_cast<int>(rows.size());
      if (r > n)
        continue;
      Eigen::MatrixXd K = Eigen::MatrixXd::Zero(n + r, n + r);
      Eigen::VectorXd rhs = Eigen::VectorXd::Zero(n + r);
      K.topLeftCorner(n, n) = P;
      rhs.head(n) = -q;
      for (int k = 0; k < r; ++k)
      {
        K.block(0, n + k, n, 1) = A.row(rows[k]).transpose();
        K.block(n + k, 0, 1, n) = A.row(rows[k]);
        rhs[n + k] = b[rows[k]];
      }
      Eigen::FullPivLU<Eigen::MatrixXd> lu(K);
      if (lu.rank() < n + r)
        continue;
      const Eigen::VectorXd x = lu.solve(rhs).head(n);
      if (((A * x).array() > b.array() + 1e-9).any())
        continue;
      const double cost = 0.5 * x.dot(P * x) + q.dot(x);
      if (cost < best_cost)
      {
        best_cost = cost;
        best = x;
      }
    }
    return std::isfinite(best_cost);
  }

  SimConfig gateConfig(std::vector<std::string> extra = {})
  {
    extra.insert(extra.begin(), {"controller.type=qp_coop", "simulation.contacts=false"});
    return test::testConfig(extra, "slot_gate");
  }

  /// 场景默认（联合优化）规划：先只平移规划，再 SE(3) + 时间联合优化
  PlanResult planFull(const SimConfig &cfg, std::shared_ptr<RobotModel> model,
                      std::shared_ptr<const ObjectTrajectory> traj)
  {
    ObjectPathPlanner planner(cfg.planner, cfg.torque_qp.collision_safe_distance, cfg.collision, model, traj);
    const PlanResult lateral = planner.plan();
    ObjectTrajectoryOptimizer optimizer(cfg.planner, cfg.torque_qp.collision_safe_distance, cfg.collision, model, traj);
    return optimizer.optimize(lateral.success ? &lateral : nullptr);
  }
} // namespace

TEST(SparseQp, MatchesBruteForceOnRandomProblems)
{
  std::mt19937 rng(5);
  std::uniform_real_distribution<double> u(-1.0, 1.0);
  int checked = 0;
  for (int trial = 0; trial < 120; ++trial)
  {
    const int n = 2 + trial % 4, m = 2 + (trial / 4) % 6;
    Eigen::MatrixXd L = Eigen::MatrixXd::Random(n, n);
    const Eigen::MatrixXd P = L * L.transpose() + 0.1 * Eigen::MatrixXd::Identity(n, n);
    Eigen::VectorXd q(n), b(m);
    Eigen::MatrixXd A(m, n);
    for (int i = 0; i < n; ++i)
      q[i] = 2.0 * u(rng);
    for (int j = 0; j < m; ++j)
    {
      for (int i = 0; i < n; ++i)
        A(j, i) = u(rng);
      b[j] = std::abs(u(rng)) + 0.1; // x = 0 严格可行，保证有解
    }
    Eigen::VectorXd reference;
    ASSERT_TRUE(bruteForce(P, q, A, b, reference));
    Eigen::VectorXd x;
    const Eigen::VectorXd lower = Eigen::VectorXd::Constant(m, -std::numeric_limits<double>::infinity());
    const sparse_qp::Result result = sparse_qp::solve(P.sparseView(), q, A.sparseView(), lower, b, x);
    EXPECT_TRUE(result.converged) << "trial " << trial;
    EXPECT_LT((x - reference).norm(), 1e-4) << "trial " << trial;
    ++checked;
  }
  EXPECT_EQ(checked, 120);
}

TEST(PathDeformation, SplineInterpolatesKnotsWithConsistentDerivatives)
{
  std::vector<double> t;
  std::vector<Vector3d> d;
  for (int k = 0; k <= 10; ++k)
  {
    t.push_back(0.2 * k);
    const double s = std::sin(M_PI * k / 10.0);
    d.push_back(Vector3d(0.0, 0.01 * s, -0.03 * s * s));
  }
  const PathDeformation path(t, d);
  Vector3d v, dv, ddv;
  for (int k = 0; k <= 10; ++k)
  {
    path.offset(t[k] + (k == 10 ? -1e-12 : 0.0), v, dv, ddv);
    EXPECT_LT((v - d[k]).norm(), 1e-9) << k;
  }
  // 导数与中心差分一致
  const double eps = 1e-5;
  for (double tau = 0.05; tau < 1.95; tau += 0.137)
  {
    Vector3d p1, p0, a;
    path.offset(tau + eps, p1, a, a);
    path.offset(tau - eps, p0, a, a);
    path.offset(tau, v, dv, ddv);
    EXPECT_LT((dv - (p1 - p0) / (2.0 * eps)).norm(), 1e-6);
    Vector3d d1, d0;
    path.offset(tau + eps, a, d1, a);
    path.offset(tau - eps, a, d0, a);
    EXPECT_LT((ddv - (d1 - d0) / (2.0 * eps)).norm(), 1e-4);
  }
  // 节点之外为 0（首末偏移为 0）
  path.offset(-1.0, v, dv, ddv);
  EXPECT_LT(v.norm() + dv.norm(), 1e-12);
  path.offset(5.0, v, dv, ddv);
  EXPECT_LT(v.norm() + dv.norm(), 1e-12);
}

TEST(TrajectoryPlanner, PlansUnderTheGateInSlotGate)
{
  const SimConfig cfg = gateConfig();
  ASSERT_TRUE(cfg.planner.enabled);
  SimEnv env(cfg);
  auto model = std::make_shared<MujocoRobotModel>(cfg);
  auto traj = std::make_shared<ObjectTrajectory>(cfg.object_trajectory, env.state().object.pose, env.scene().waypoints);
  ObjectPathPlanner planner(cfg.planner, cfg.torque_qp.collision_safe_distance, cfg.collision, model, traj);
  const PlanResult plan = planner.plan();
  EXPECT_TRUE(plan.success) << plan.message;
  EXPECT_EQ(plan.initialization, "down"); // 从名义轨迹出发会停在局部解（穿透法向沿路径方向）
  EXPECT_GT(plan.starts, 1);
  EXPECT_LE(plan.worst_violation, 5e-4);
  EXPECT_GT(plan.max_offset, 0.07);  // 横梁要求下压约 8 cm
  EXPECT_LT(plan.max_offset, 0.15);
  EXPECT_LT(plan.planning_time_s, 10.0);
  // 偏移主要是向下（z < 0），不是往上翻越
  double min_z = 0.0, max_z = 0.0;
  for (const Vector3d &d : plan.offsets)
  {
    min_z = std::min(min_z, d.z());
    max_z = std::max(max_z, d.z());
  }
  EXPECT_LT(min_z, -0.07);
  EXPECT_LT(max_z, 0.005);
  for (std::size_t k = 0; k < plan.knot_min_margin.size(); ++k)
  {
    if (std::isfinite(plan.knot_min_margin[k]))
    {
      EXPECT_GE(plan.knot_min_margin[k], -5e-4) << "knot " << k;
    }
  }

  // 重新计时后的平移速度 / 加速度（分段放慢是近似时间参数化，加速度留 30% 余量）
  traj->setDeformation(plan.deformation);
  double max_speed = 0.0, max_accel = 0.0;
  for (double t = 0.0; t < traj->endTime(); t += 0.002)
  {
    const ObjectReference ref = traj->evaluate(t);
    max_speed = std::max(max_speed, ref.twist.head<3>().norm());
    max_accel = std::max(max_accel, ref.accel.head<3>().norm());
  }
  EXPECT_LE(max_speed, cfg.planner.max_speed * 1.02);
  EXPECT_LE(max_accel, cfg.planner.max_accel * 1.3);
}

TEST(TrajectoryPlanner, PlannedPathWithCbfPassesUnderGateAndInserts)
{
  const SimConfig cfg = gateConfig();
  SimEnv env(cfg);
  auto model = std::make_shared<MujocoRobotModel>(cfg);
  auto traj = std::make_shared<ObjectTrajectory>(cfg.object_trajectory, env.state().object.pose, env.scene().waypoints);
  ObjectPathPlanner planner(cfg.planner, cfg.torque_qp.collision_safe_distance, cfg.collision, model, traj);
  traj->setDeformation(planner.plan().deformation);
  QpCoopController controller(model, traj, cfg.coop, cfg.torque_qp, cfg.collision, cfg.timestep);
  controller.reset(env.state());

  // 独立的距离监视：plant 基座位姿下物体 ~ 横梁、两臂 ~ 横梁
  CollisionModel monitor(env.scene(), {plantBasePose(cfg, Arm::Left), plantBasePose(cfg, Arm::Right)}, cfg.collision);
  int object_gate = -1, left_gate = -1, right_gate = -1;
  for (int g = 0; g < monitor.numGroups(); ++g)
  {
    object_gate = monitor.groupName(g) == "object~gate" ? g : object_gate;
    left_gate = monitor.groupName(g) == "left_arm~gate" ? g : left_gate;
    right_gate = monitor.groupName(g) == "right_arm~gate" ? g : right_gate;
  }
  ASSERT_GE(object_gate, 0);
  double min_object = 1.0, min_arm = 1.0, max_tracking = 0.0;
  for (int k = 0; k < 20000; ++k)
  {
    const auto [tau_left, tau_right] = controller.compute(env.state(), env.time());
    ASSERT_TRUE(tau_left.allFinite() && tau_right.allFinite());
    env.step(tau_left, tau_right);
    if (k % 10 == 0)
    {
      const DualArmState &s = env.state();
      monitor.query(s.arm(Arm::Left).q, s.arm(Arm::Right).q);
      min_object = std::min(min_object, monitor.groupMinDistance(object_gate));
      min_arm = std::min({min_arm, monitor.groupMinDistance(left_gate), monitor.groupMinDistance(right_gate)});
      max_tracking = std::max(max_tracking, (traj->evaluate(s.t).pose.p - s.object.pose.p).norm());
    }
  }
  const Vector6d final_error = poseError(env.scene().waypoints.back().pose, env.state().object.pose);
  EXPECT_FALSE(env.diverged());
  EXPECT_GT(min_object, 0.015); // 规划要求 20 mm，执行误差内仍 ≥ 15 mm
  EXPECT_GT(min_arm, 0.03);
  EXPECT_LT(max_tracking, 0.02);
  EXPECT_LT(final_error.head<3>().norm(), 0.002);
  EXPECT_LT(final_error.tail<3>().norm(), 0.01);
}

TEST(PathDeformation, RotationAndKnotTimeMapMatchNumericalDerivatives)
{
  // 名义：slot_gate 的航点轨迹（含滚转）；叠加平滑的平移 / 姿态偏移和非均匀节点时间
  const SimConfig cfg = gateConfig();
  SimEnv env(cfg);
  auto traj = std::make_shared<ObjectTrajectory>(cfg.object_trajectory, env.state().object.pose, env.scene().waypoints);
  const double T = traj->nominalEndTime();
  const int N = 40;
  std::vector<double> tau(N + 1), t_exec(N + 1);
  std::vector<Vector3d> d(N + 1), phi(N + 1);
  for (int k = 0; k <= N; ++k)
  {
    tau[k] = T * k / N;
    const double s = std::sin(M_PI * k / N);
    d[k] = Vector3d(0.0, 0.02 * s, -0.05 * s * s);
    phi[k] = Vector3d(0.4 * s * s, 0.0, 0.0);
    t_exec[k] = k == 0 ? 0.0 : t_exec[k - 1] + (T / N) * (0.6 + 0.3 * std::cos(0.3 * k));
  }
  auto path = std::make_shared<PathDeformation>(tau, d);
  path->setRotationOffsets(phi);
  path->setKnotExecutionTimes(t_exec);
  traj->setDeformation(path);
  const double eps = 1e-5;
  for (double t = 0.3; t < traj->endTime() - 0.3; t += 0.173)
  {
    const ObjectReference r0 = traj->evaluate(t - eps), r1 = traj->evaluate(t + eps), r = traj->evaluate(t);
    const Vector3d v_fd = (r1.pose.p - r0.pose.p) / (2.0 * eps);
    const Eigen::AngleAxisd dR(r1.pose.q * r0.pose.q.conjugate());
    const Vector3d w_fd = dR.angle() * dR.axis() / (2.0 * eps);
    EXPECT_LT((r.twist.head<3>() - v_fd).norm(), 1e-5) << "t = " << t;
    EXPECT_LT((r.twist.tail<3>() - w_fd).norm(), 1e-5) << "t = " << t;
    const Vector3d a_fd = (r1.twist.head<3>() - r0.twist.head<3>()) / (2.0 * eps);
    EXPECT_LT((r.accel.head<3>() - a_fd).norm(), 1e-3) << "t = " << t;
  }
}

TEST(TrajectoryOptimizer, JointSe3TimeOptimizationRollsWhileMovingAndIsFaster)
{
  const SimConfig cfg = gateConfig();
  ASSERT_EQ(cfg.planner.formulation, PlannerConfig::Formulation::Full);
  SimEnv env(cfg);
  auto model = std::make_shared<MujocoRobotModel>(cfg);
  auto traj = std::make_shared<ObjectTrajectory>(cfg.object_trajectory, env.state().object.pose, env.scene().waypoints);
  const PlanResult plan = planFull(cfg, model, traj);
  EXPECT_TRUE(plan.success) << plan.message;
  EXPECT_EQ(plan.formulation, "full");
  EXPECT_GT(plan.starts, 1);
  EXPECT_GT(plan.max_rotation, 10.0 * M_PI / 180.0); // 确实用上了姿态自由度
  EXPECT_LT(plan.added_duration, -3.0);             // 边转边走，明显快于名义轨迹
  EXPECT_LT(plan.planning_time_s, 30.0);
  for (std::size_t k = 0; k < plan.knot_min_margin.size(); ++k)
  {
    if (std::isfinite(plan.knot_min_margin[k]))
    {
      EXPECT_GE(plan.knot_min_margin[k], -5e-4) << "knot " << k;
    }
  }
}

TEST(TrajectoryOptimizer, OptimizedTrajectoryWithCbfCompletesInsertion)
{
  const SimConfig cfg = gateConfig();
  SimEnv env(cfg);
  auto model = std::make_shared<MujocoRobotModel>(cfg);
  auto traj = std::make_shared<ObjectTrajectory>(cfg.object_trajectory, env.state().object.pose, env.scene().waypoints);
  traj->setDeformation(planFull(cfg, model, traj).deformation);
  QpCoopController controller(model, traj, cfg.coop, cfg.torque_qp, cfg.collision, cfg.timestep);
  controller.reset(env.state());
  CollisionModel monitor(env.scene(), {plantBasePose(cfg, Arm::Left), plantBasePose(cfg, Arm::Right)}, cfg.collision);
  int object_gate = -1, left_gate = -1, right_gate = -1;
  for (int g = 0; g < monitor.numGroups(); ++g)
  {
    object_gate = monitor.groupName(g) == "object~gate" ? g : object_gate;
    left_gate = monitor.groupName(g) == "left_arm~gate" ? g : left_gate;
    right_gate = monitor.groupName(g) == "right_arm~gate" ? g : right_gate;
  }
  double min_object = 1.0, min_arm = 1.0, max_tracking = 0.0;
  const int steps = static_cast<int>((traj->endTime() + 1.0) / cfg.timestep);
  for (int k = 0; k < steps; ++k)
  {
    const auto [tau_left, tau_right] = controller.compute(env.state(), env.time());
    ASSERT_TRUE(tau_left.allFinite() && tau_right.allFinite());
    env.step(tau_left, tau_right);
    if (k % 10 == 0)
    {
      const DualArmState &s = env.state();
      monitor.query(s.arm(Arm::Left).q, s.arm(Arm::Right).q);
      min_object = std::min(min_object, monitor.groupMinDistance(object_gate));
      min_arm = std::min({min_arm, monitor.groupMinDistance(left_gate), monitor.groupMinDistance(right_gate)});
      max_tracking = std::max(max_tracking, (traj->evaluate(s.t).pose.p - s.object.pose.p).norm());
    }
  }
  const Vector6d final_error = poseError(env.scene().waypoints.back().pose, env.state().object.pose);
  EXPECT_FALSE(env.diverged());
  EXPECT_LT(traj->endTime(), traj->nominalEndTime() - 3.0);
  EXPECT_GT(min_object, 0.015);
  EXPECT_GT(min_arm, 0.012); // 规划只在节点约束距离（目标 20 mm），执行中留 8 mm 余量
  EXPECT_LT(max_tracking, 0.02);
  EXPECT_LT(final_error.head<3>().norm(), 0.002);
  EXPECT_LT(final_error.tail<3>().norm(), 0.01);
}

// 执行层偏差（docs/trajectory_planning.md §11）：规划器以为横梁高 3 cm。
// 只按规划执行会穿透横梁；规划 + CBF + 力矩 QP 保持安全距离且跟踪误差小。
TEST(TrajectoryOptimizer, ExecutionLayerKeepsClearanceUnderPlannerModelError)
{
  auto runWith = [](std::vector<std::string> overrides, double &min_object, double &max_tracking)
  {
    overrides.insert(overrides.end(), {"planner.model_error_body=gate", "planner.model_error_offset=[0,0,0.03]"});
    const SimConfig cfg = gateConfig(overrides);
    SimEnv env(cfg);
    auto model = std::make_shared<MujocoRobotModel>(cfg);
    auto traj =
        std::make_shared<ObjectTrajectory>(cfg.object_trajectory, env.state().object.pose, env.scene().waypoints);
    traj->setDeformation(planFull(cfg, model, traj).deformation);
    QpCoopController controller(model, traj, cfg.coop, cfg.torque_qp, cfg.collision, cfg.timestep);
    controller.reset(env.state());
    CollisionModel monitor(env.scene(), {plantBasePose(cfg, Arm::Left), plantBasePose(cfg, Arm::Right)}, cfg.collision);
    int object_gate = -1;
    for (int g = 0; g < monitor.numGroups(); ++g)
      object_gate = monitor.groupName(g) == "object~gate" ? g : object_gate;
    min_object = 1.0;
    max_tracking = 0.0;
    const int steps = static_cast<int>((traj->endTime() + 1.0) / cfg.timestep);
    for (int k = 0; k < steps; ++k)
    {
      const auto [tau_left, tau_right] = controller.compute(env.state(), env.time());
      env.step(tau_left, tau_right);
      if (k % 10 == 0)
      {
        const DualArmState &s = env.state();
        monitor.query(s.arm(Arm::Left).q, s.arm(Arm::Right).q);
        min_object = std::min(min_object, monitor.groupMinDistance(object_gate));
        // 跟踪误差相对控制器实际跟踪的参考（启用 CBF 时是滤波后的参考）
        max_tracking = std::max(max_tracking, (controller.governedReference().pose.p - s.object.pose.p).norm());
      }
    }
  };
  double plan_only_min = 0.0, plan_only_tracking = 0.0, full_min = 0.0, full_tracking = 0.0;
  runWith({"controller.torque_qp.reference_governor.enabled=false",
           "controller.torque_qp.collision_avoidance_enabled=false"},
          plan_only_min, plan_only_tracking);
  runWith({}, full_min, full_tracking);
  EXPECT_LT(plan_only_min, 0.0);   // 只靠规划：穿透横梁
  EXPECT_GT(full_min, 0.0095);     // CBF + QP：守住 10 mm 安全距离（离散误差 0.5 mm）
  EXPECT_LT(full_tracking, 0.02);  // 且参考被平滑修正，跟踪误差仍小
}
