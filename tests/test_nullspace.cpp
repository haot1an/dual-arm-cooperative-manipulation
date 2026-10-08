#include "dual_arm/nullspace_task.hpp"
#include "dual_arm/mujoco_model.hpp"
#include "dual_arm/object_trajectory.hpp"
#include "dual_arm/qp_coop_controller.hpp"
#include "test_common.hpp"

#include <Eigen/Cholesky>
#include <gtest/gtest.h>

using namespace dual_arm;

TEST(NullspaceTask, DynamicProjectionPreservesTcpAndDiffersFromVelocityProjection) {
  const SimConfig cfg = test::testConfig();
  SimEnv env(cfg);
  MujocoRobotModel model(cfg);
  model.update(env.state());
  for (Arm a : kArms) {
    const Matrix6x7d J = model.jacobian(a);
    const Matrix7d M = model.massMatrix(a);
    int rank = 0;
    const Matrix7d P = NullspaceTask::torqueProjector(J, M, 0.3, 1e-6, &rank);
    const Matrix7d Mi = M.ldlt().solve(Matrix7d::Identity());
    const Matrix7d expected = Matrix7d::Identity() -
        J.transpose() * (J * Mi * J.transpose()).ldlt().solve(J * Mi);
    EXPECT_EQ(rank, 6);
    EXPECT_LT((P - expected).norm(), 1e-10);
    EXPECT_LT((J * Mi * P).norm(), 1e-11);
    EXPECT_LT((P * P - P).norm(), 1e-11);
    EXPECT_LT((P * J.transpose()).norm(), 1e-11);
    const Matrix7d kinematic = Matrix7d::Identity() - J.transpose() * (J * J.transpose()).ldlt().solve(J);
    EXPECT_GT((J * Mi * kinematic).norm(), 1e-2);
  }
}

TEST(NullspaceTask, RankDeficientAndZeroJacobiansRemainFinite) {
  Matrix6x7d J = Matrix6x7d::Zero();
  for (int j = 0; j < 5; ++j) J(j,j) = 1.0;
  int rank = 0;
  const Matrix7d P = NullspaceTask::torqueProjector(J, Matrix7d::Identity(), 0.3, 1e-6, &rank);
  EXPECT_EQ(rank, 5);
  EXPECT_TRUE(P.allFinite());
  EXPECT_LT((J * P).norm(), 1e-12);
  EXPECT_NEAR(P.trace(), 2.0, 1e-12);
  EXPECT_LT((NullspaceTask::torqueProjector(Matrix6x7d::Zero(), Matrix7d::Identity(), 0.3, 1e-6) -
             Matrix7d::Identity()).norm(), 1e-12);
}

TEST(NullspaceTask, LimitPotentialHasCorrectGradientAndDeadZone) {
  const Vector7d lo = Vector7d::Constant(-1.0), hi = Vector7d::Constant(1.0);
  Vector7d q = Vector7d::Zero();
  q[0] = -0.95;
  q[1] = 0.9;
  const Vector7d tau = NullspaceTask::jointLimitTorque(q, lo, hi, 0.3, 20.0);
  EXPECT_NEAR(tau[0], 5.0, 1e-12);
  EXPECT_NEAR(tau[1], -4.0, 1e-12);
  EXPECT_DOUBLE_EQ(tau.tail<5>().norm(), 0.0);
  auto potential = [&](const Vector7d& x) {
    double v = 0;
    for (int k = 0; k < 7; ++k) {
      const double l = std::max(0.0, 0.3 - (x[k] - lo[k]));
      const double u = std::max(0.0, 0.3 - (hi[k] - x[k]));
      v += 10.0 * (l*l + u*u);
    }
    return v;
  };
  for (int k = 0; k < 7; ++k) {
    Vector7d plus = q, minus = q;
    plus[k] += 1e-6;
    minus[k] -= 1e-6;
    EXPECT_NEAR(tau[k], -(potential(plus) - potential(minus)) / 2e-6, 1e-8);
  }
}

TEST(NullspaceTask, UniformTorqueLimitAndRampPreserveDynamicNullspace) {
  SimConfig cfg = test::testConfig();
  cfg.coop.nullspace.enabled = true;
  cfg.coop.nullspace.max_torque = 0.05;
  cfg.coop.nullspace_kp = 1000.0;
  SimEnv env(cfg);
  MujocoRobotModel model(cfg);
  DualArmState state = env.state();
  NullspaceTask task(cfg.coop);
  task.reset(state);
  model.update(state);
  EXPECT_DOUBLE_EQ(task.compute(state, model).norm(), 0.0);
  for (Arm a : kArms) state.arm(a).q += Vector7d::Constant(0.03);
  state.t = 2.0;
  model.update(state);
  const Vector14d tau = task.compute(state, model);
  EXPECT_NEAR(tau.cwiseAbs().maxCoeff(), 0.05, 1e-12);
  EXPECT_LT(task.diagnostics().acceleration_leak, 1e-11);
  cfg.coop.nullspace.enabled = false;
  NullspaceTask disabled(cfg.coop);
  disabled.reset(env.state());
  EXPECT_DOUBLE_EQ(disabled.compute(state, model).norm(), 0.0);
}

TEST(NullspaceTask, KinematicProjectionOptionLeaksIntoTcpAcceleration) {
  // projection: kinematic 只作对照：投影 I − J⁺J 不保证 J M⁻¹ τ_ns = 0。
  EXPECT_THROW(test::testConfig({"controller.coop.nullspace.projection=bogus"}), std::runtime_error);
  double leak[2] = {0.0, 0.0};
  const char* modes[2] = {"controller.coop.nullspace.projection=dynamic",
                          "controller.coop.nullspace.projection=kinematic"};
  for (int k = 0; k < 2; ++k) {
    SimConfig cfg = test::testConfig({modes[k]});
    cfg.coop.nullspace.enabled = true;
    cfg.coop.nullspace_kp = 50.0;
    SimEnv env(cfg);
    MujocoRobotModel model(cfg);
    DualArmState state = env.state();
    NullspaceTask task(cfg.coop);
    task.reset(state);
    for (Arm a : kArms) state.arm(a).q += Vector7d::Constant(0.03);
    state.t = 5.0;
    model.update(state);
    EXPECT_GT(task.compute(state, model).norm(), 1e-3);
    leak[k] = task.diagnostics().acceleration_leak;
  }
  EXPECT_LT(leak[0], 1e-11);
  EXPECT_GT(leak[1], 1e-3);
}

TEST(NullspaceTask, RejectsInvalidParametersAtLoadAndConstruction) {
  for (const char* override : {"controller.coop.nullspace.max_torque=0",
                              "controller.coop.nullspace.rank_tolerance=1",
                              "controller.coop.nullspace.avoidance_gain=-1",
                              "controller.coop.nullspace.rotation_length=.nan"})
    EXPECT_THROW(test::testConfig({override}), std::runtime_error);
  CoopConfig config;
  config.nullspace.ramp_time = -1;
  EXPECT_THROW(NullspaceTask task(config), std::invalid_argument);
}

TEST(NullspaceTask, UsesClosestPairPerGroupAndExcludesObjectDistances) {
  SimConfig cfg = test::testConfig({}, "nullspace");
  cfg.coop.nullspace.ramp_time = 0;
  SimEnv env(cfg);
  MujocoRobotModel model(cfg);
  model.update(env.state());
  NullspaceTask task(cfg.coop);
  task.reset(env.state());
  std::vector<DistanceInfo> distances(3);
  distances[0].group = 1; // left_arm~post_L
  distances[0].distance = 0.04;
  distances[0].jacobian[0] = 0.1;
  distances[1] = distances[0];
  distances[1].distance = 0.06;
  distances[1].jacobian[0] = -100;
  distances[2] = distances[0];
  distances[2].group = 3; // object~post_L
  distances[2].distance = -1;
  distances[2].jacobian[0] = -100;
  const Vector14d actual = task.compute(env.state(), model, &distances);
  EXPECT_EQ(task.diagnostics().active_pairs, 1);
  distances.resize(1);
  const Vector14d single = task.compute(env.state(), model, &distances);
  EXPECT_LT((actual - single).norm(), 1e-12);
  EXPECT_GT(actual.norm(), 0.1);
}

TEST(NullspaceTask, StaticAvoidanceImprovesClearanceWithoutChangingObjectTask) {
  std::array<double, 2> final_clearance{};
  for (int enabled = 0; enabled < 2; ++enabled) {
    const SimConfig cfg = test::testConfig(
        {std::string("controller.coop.nullspace.enabled=") + (enabled ? "true" : "false")}, "nullspace");
    SimEnv env(cfg);
    auto model = std::make_shared<MujocoRobotModel>(cfg);
    auto traj = std::make_shared<ObjectTrajectory>(cfg.object_trajectory, env.state().object.pose, env.scene().waypoints);
    QpCoopController ctrl(model, traj, cfg.coop, cfg.torque_qp, cfg.collision, cfg.timestep);
    ctrl.reset(env.state());
    CollisionModel distances(env.scene(), {env.basePose(Arm::Left), env.basePose(Arm::Right)}, cfg.collision);
    const Pose object_initial = env.state().object.pose;
    const Vector7d q_initial = env.state().arm(Arm::Left).q;
    double max_position = 0, max_rotation = 0, max_leak = 0;
    for (int k = 0; k < 12000; ++k) {
      const auto tau = ctrl.compute(env.state(), env.time());
      ASSERT_TRUE(tau.first.allFinite());
      ASSERT_TRUE(tau.second.allFinite());
      // 固定迭代预算的 ADMM 冷启动可以返回 MaxIterations；检查可行性残差，
      // 并要求终态收敛，避免仅凭求解器标签判断物理行为。
      ASSERT_NE(ctrl.qpStatus(), JointSafetyTorqueQp::Status::InvalidInput);
      ASSERT_NE(ctrl.qpStatus(), JointSafetyTorqueQp::Status::InfeasibleBounds);
      EXPECT_LT(ctrl.maxConstraintViolation(), 0.002);
      EXPECT_LT(ctrl.closedChainAccelerationResidual(), 0.002);
      EXPECT_LT(ctrl.maxCollisionSlack(), 1e-8);
      env.step(tau.first, tau.second);
      const Vector6d e = poseError(object_initial, env.state().object.pose);
      max_position = std::max(max_position, e.head<3>().norm());
      max_rotation = std::max(max_rotation, e.tail<3>().norm());
      max_leak = std::max(max_leak, ctrl.nullspaceDiagnostics().acceleration_leak);
    }
    ASSERT_FALSE(env.diverged());
    EXPECT_EQ(ctrl.qpStatus(), JointSafetyTorqueQp::Status::Solved);
    distances.query(env.state().arm(Arm::Left).q, env.state().arm(Arm::Right).q);
    final_clearance[enabled] = std::min(distances.groupMinDistance(1), distances.groupMinDistance(2));
    EXPECT_LT(max_position, 1.2e-3);
    EXPECT_LT(max_rotation, 0.002);
    EXPECT_LT(max_leak, 1e-10);
    EXPECT_LT((env.state().object.pose.p - object_initial.p).norm(), 1e-5);
    if (enabled) {
      EXPECT_GT(final_clearance[enabled], 0.10);
      EXPECT_GT((env.state().arm(Arm::Left).q - q_initial).norm(), 0.5);
    } else {
      EXPECT_LT((env.state().arm(Arm::Left).q - q_initial).norm(), 0.002);
    }
  }
  EXPECT_GT(final_clearance[1] - final_clearance[0], 0.07);
}
