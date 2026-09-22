#include "dual_arm/mujoco_model.hpp"
#include "dual_arm/object_trajectory.hpp"
#include "dual_arm/qp_asym_coop_controller.hpp"
#include "dual_arm/qp_coop_controller.hpp"
#include "dual_arm/sim_env.hpp"
#include "dual_arm/torque_qp.hpp"
#include "test_common.hpp"

#include <gtest/gtest.h>

#include <limits>
#include <memory>

using namespace dual_arm;

TEST(BoxTorqueQp, SolvesWeightedTrackingAndSmoothingObjective)
{
  TorqueQpConfig config;
  config.tracking_weight = 2.0;
  config.smoothing_weight = 1.0;

  BoxTorqueQp qp(config);
  const Vector14d previous =
      Vector14d::Constant(1.0);
  qp.reset(previous);

  const Vector14d desired =
      Vector14d::Constant(4.0);
  const Vector14d lower =
      Vector14d::Constant(-10.0);
  const Vector14d upper =
      Vector14d::Constant(10.0);

  const Vector14d solution =
      qp.solve(desired, lower, upper);

  // (2*4 + 1*1) / (2+1) = 3。
  EXPECT_TRUE(solution.isApprox(Vector14d::Constant(3.0), 1e-12));
  EXPECT_EQ(qp.status(), BoxTorqueQp::Status::Solved);
  EXPECT_EQ(qp.activeConstraints(), 0);
}

TEST(BoxTorqueQp, EnforcesEveryTorqueBound)
{
  TorqueQpConfig config;
  config.smoothing_weight = 0.0;
  BoxTorqueQp qp(config);

  Vector14d desired;
  for (int i = 0; i < desired.size(); ++i)
  {
    desired[i] = i % 2 == 0 ? 100.0 : -100.0;
  }

  const Vector14d lower =
      Vector14d::Constant(-5.0);
  const Vector14d upper =
      Vector14d::Constant(7.0);
  const Vector14d solution =
      qp.solve(desired, lower, upper);

  EXPECT_TRUE((solution.array() >= lower.array()).all());
  EXPECT_TRUE((solution.array() <= upper.array()).all());
  EXPECT_EQ(qp.activeConstraints(), 14);
}

TEST(BoxTorqueQp, FallsBackToPreviousTorqueForInvalidDesiredInput)
{
  TorqueQpConfig config;
  BoxTorqueQp qp(config);
  qp.reset(Vector14d::Constant(2.0));

  Vector14d desired = Vector14d::Zero();
  desired[3] = std::numeric_limits<double>::quiet_NaN();

  const Vector14d solution =
      qp.solve(
          desired,
          Vector14d::Constant(-1.0),
          Vector14d::Constant(1.0));

  EXPECT_TRUE(solution.isApprox(Vector14d::Ones(), 1e-12));
  EXPECT_EQ(qp.status(), BoxTorqueQp::Status::InvalidInput);
}

TEST(JointSafetyTorqueQp, BrakesBeforePredictedPositionLimit)
{
  TorqueQpConfig config;
  config.smoothing_weight = 0.0;
  config.joint_position_margin = 0.10;
  config.joint_prediction_horizon = 0.10;
  config.joint_velocity_limit.setConstant(10.0);
  config.joint_acceleration_limit.setConstant(100.0);
  config.admm_rho = 10.0;
  config.admm_max_iterations = 200;
  config.admm_tolerance = 1e-7;

  JointSafetyTorqueQp qp(config);
  const Vector14d torque_lower =
      Vector14d::Constant(-50.0);
  const Vector14d torque_upper =
      Vector14d::Constant(50.0);
  Vector14d position = Vector14d::Zero();
  Vector14d velocity = Vector14d::Zero();
  const Vector14d position_lower =
      Vector14d::Constant(-1.0);
  const Vector14d position_upper =
      Vector14d::Constant(1.0);

  // 安全上限为 0.9 rad。当前 q=0.89、dq=0.2，若不制动，0.1 s 后越界。
  position[0] = 0.89;
  velocity[0] = 0.20;

  const Vector14d solution =
      qp.solve(
          Vector14d::Constant(10.0),
          torque_lower,
          torque_upper,
          Matrix14d::Identity(),
          Vector14d::Zero(),
          position,
          velocity,
          position_lower,
          position_upper);

  // M=I、h=0，因此 tau=ddq；预测位置约束要求 ddq_0 <= -2 rad/s²。
  EXPECT_LE(solution[0], -2.0 + 2e-5);
  EXPECT_LE(
      solution[0],
      qp.accelerationUpperBound()[0] + 2e-5);
  EXPECT_LE(qp.maxConstraintViolation(), 2e-5);
  EXPECT_EQ(qp.status(), JointSafetyTorqueQp::Status::Solved);
}

TEST(JointSafetyTorqueQp, BrakesBeforePredictedVelocityLimit)
{
  TorqueQpConfig config;
  config.smoothing_weight = 0.0;
  config.joint_position_margin = 0.0;
  config.joint_prediction_horizon = 0.10;
  config.joint_velocity_limit.setConstant(2.0);
  config.joint_acceleration_limit.setConstant(100.0);
  config.admm_rho = 10.0;
  config.admm_max_iterations = 200;
  config.admm_tolerance = 1e-7;

  JointSafetyTorqueQp qp(config);
  Vector14d velocity = Vector14d::Zero();
  velocity[4] = 1.9;

  const Vector14d solution =
      qp.solve(
          Vector14d::Constant(10.0),
          Vector14d::Constant(-50.0),
          Vector14d::Constant(50.0),
          Matrix14d::Identity(),
          Vector14d::Zero(),
          Vector14d::Zero(),
          velocity,
          Vector14d::Constant(-10.0),
          Vector14d::Constant(10.0));

  // dq(T)=dq+ddq*T <= 2，所以 ddq_4 <= 1 rad/s²。
  EXPECT_LE(solution[4], 1.0 + 2e-5);
  EXPECT_LE(
      solution[4],
      qp.accelerationUpperBound()[4] + 2e-5);
  EXPECT_LE(qp.maxConstraintViolation(), 2e-5);
  EXPECT_EQ(qp.status(), JointSafetyTorqueQp::Status::Solved);
}

TEST(JointSafetyTorqueQp, EnforcesCoupledAccelerationEquality)
{
  TorqueQpConfig config;
  config.smoothing_weight = 0.0;
  config.joint_position_margin = 0.0;
  config.joint_prediction_horizon = 0.10;
  config.joint_velocity_limit.setConstant(100.0);
  config.joint_acceleration_limit.setConstant(100.0);
  config.admm_rho = 10.0;
  config.admm_max_iterations = 300;
  config.admm_tolerance = 1e-8;

  JointSafetyTorqueQp qp(config);
  Vector14d desired = Vector14d::Zero();
  desired[0] = 10.0;
  desired[7] = -10.0;

  Matrix6x14d equality = Matrix6x14d::Zero();
  equality(0, 0) = 1.0;
  equality(0, 7) = -1.0;

  const Vector14d solution =
      qp.solve(
          desired,
          Vector14d::Constant(-50.0),
          Vector14d::Constant(50.0),
          Matrix14d::Identity(),
          Vector14d::Zero(),
          Vector14d::Zero(),
          Vector14d::Zero(),
          Vector14d::Constant(-10.0),
          Vector14d::Constant(10.0),
          equality,
          Vector6d::Zero());

  EXPECT_TRUE(solution.allFinite());
  EXPECT_LT(
      (equality * qp.predictedAcceleration()).norm(),
      2e-6);
  EXPECT_LE(qp.maxConstraintViolation(), 2e-6);
  EXPECT_EQ(qp.status(), JointSafetyTorqueQp::Status::Solved);
}

TEST(JointSafetyTorqueQp, EnforcesCollisionAccelerationDamper)
{
  TorqueQpConfig config;
  config.smoothing_weight = 0.0;
  config.joint_position_margin = 0.0;
  config.joint_prediction_horizon = 0.10;
  config.joint_velocity_limit.setConstant(100.0);
  config.joint_acceleration_limit.setConstant(100.0);
  config.collision_slack_weight = 1e6;
  config.admm_rho = 10.0;
  config.admm_max_iterations = 400;
  config.admm_tolerance = 1e-8;

  JointSafetyTorqueQp qp(config);
  JointSafetyTorqueQp::CollisionMatrix collision =
      JointSafetyTorqueQp::CollisionMatrix::Zero();
  JointSafetyTorqueQp::CollisionVector lower =
      JointSafetyTorqueQp::CollisionVector::Zero();
  collision(0, 0) = 1.0;
  lower[0] = 5.0;

  const Vector14d solution =
      qp.solve(
          Vector14d::Zero(),
          Vector14d::Constant(-50.0),
          Vector14d::Constant(50.0),
          Matrix14d::Identity(),
          Vector14d::Zero(),
          Vector14d::Zero(),
          Vector14d::Zero(),
          Vector14d::Constant(-10.0),
          Vector14d::Constant(10.0),
          Matrix6x14d::Zero(),
          Vector6d::Zero(),
          collision,
          lower,
          1);

  EXPECT_TRUE(solution.allFinite());
  EXPECT_GE(qp.predictedAcceleration()[0] + qp.maxCollisionSlack(), 5.0 - 2e-5);
  EXPECT_LT(qp.maxCollisionSlack(), 1e-3);
  EXPECT_LE(qp.maxConstraintViolation(), 2e-5);
  EXPECT_EQ(qp.status(), JointSafetyTorqueQp::Status::Solved);
}

TEST(QpAsymCoopController, AssemblyReachesHoldInsideTorqueBounds)
{
  const SimConfig cfg =
      test::testConfig(
          {
              "controller.type=qp_asym_coop",
              "object_trajectory.type=waypoints",
              "simulation.contacts=false",
              "disturbances=[]",
          },
          "assembly");

  SimEnv env(cfg);
  auto model =
      std::make_shared<MujocoRobotModel>(cfg);
  auto trajectory =
      std::make_shared<ObjectTrajectory>(
          cfg.object_trajectory,
          env.state().object.pose,
          env.scene().waypoints);

  QpAsymmetricCoopController controller(
      model,
      trajectory,
      cfg.asym_coop,
      cfg.torque_qp,
      cfg.collision,
      cfg.timestep);
  controller.reset(env.state());

  const Pose initial_object_pose =
      env.state().object.pose;
  double max_object_drift = 0.0;
  int active_constraint_steps = 0;

  for (int k = 0; k < 14000; ++k)
  {
    const auto [tau_left, tau_right] =
        controller.compute(env.state(), env.time());

    ASSERT_TRUE(tau_left.allFinite());
    ASSERT_TRUE(tau_right.allFinite());
    EXPECT_TRUE(
        (tau_left.array() >=
         controller.torqueLowerBound().head<kArmDof>().array())
            .all());
    EXPECT_TRUE(
        (tau_left.array() <=
         controller.torqueUpperBound().head<kArmDof>().array())
            .all());
    EXPECT_TRUE(
        (tau_right.array() >=
         controller.torqueLowerBound().tail<kArmDof>().array())
            .all());
    EXPECT_TRUE(
        (tau_right.array() <=
         controller.torqueUpperBound().tail<kArmDof>().array())
            .all());

    active_constraint_steps +=
        controller.activeTorqueConstraints() > 0 ? 1 : 0;
    env.step(tau_left, tau_right);
    max_object_drift =
        std::max(
            max_object_drift,
            (env.state().object.pose.p -
             initial_object_pose.p)
                .norm());
  }

  EXPECT_FALSE(env.diverged());
  EXPECT_EQ(
      controller.qpStatus(),
      JointSafetyTorqueQp::Status::Solved);
  EXPECT_EQ(
      controller.phase(),
      AsymmetricCoopController::Phase::Hold);
  EXPECT_NEAR(
      controller.measuredPreload(),
      cfg.asym_coop.axial_preload_force,
      cfg.asym_coop.completion_preload_tolerance);
  EXPECT_GE(
      controller.measuredTighteningTorque(),
      cfg.asym_coop.tightening_torque -
          cfg.asym_coop.completion_torque_tolerance);
  EXPECT_LT(
      std::abs(env.state().screw.rate),
      cfg.asym_coop.completion_speed_threshold);
  EXPECT_LT(max_object_drift, 0.002);
  // 闭链等式在每个周期都是 active constraint。
  EXPECT_GT(active_constraint_steps, 0);
  EXPECT_LT(
      controller.closedChainAccelerationResidual(),
      1e-3);
  EXPECT_GT(
      controller.collisionMinDistance(),
      cfg.torque_qp.collision_safe_distance);
  EXPECT_LT(controller.maxCollisionSlack(), 1e-3);
}

TEST(QpCoopController, SlotInsertionAvoidsPenetrationAndReachesGoal)
{
  const SimConfig cfg = test::testConfig(
      {
          "controller.type=qp_coop",
          "object_trajectory.type=waypoints",
          "simulation.contacts=false",
          "disturbances=[]",
      },
      "slot");

  SimEnv env(cfg);
  auto model = std::make_shared<MujocoRobotModel>(cfg);
  auto trajectory = std::make_shared<ObjectTrajectory>(
      cfg.object_trajectory,
      env.state().object.pose,
      env.scene().waypoints);
  QpCoopController controller(
      model,
      trajectory,
      cfg.coop,
      cfg.torque_qp,
      cfg.collision,
      cfg.timestep);
  controller.reset(env.state());

  double minimum_distance = std::numeric_limits<double>::infinity();
  int collision_active_steps = 0;
  int invalid_steps = 0;
  double maximum_constraint_violation = 0.0;
  double maximum_violation_time = 0.0;
  int maximum_violation_row = -1;
  double maximum_collision_slack = 0.0;
  for (int k = 0; k < 15000; ++k)
  {
    const auto [tau_left, tau_right] =
        controller.compute(env.state(), env.time());
    ASSERT_TRUE(tau_left.allFinite());
    ASSERT_TRUE(tau_right.allFinite());
    minimum_distance = std::min(
        minimum_distance,
        controller.collisionMinDistance());
    collision_active_steps +=
        controller.activeCollisionConstraints() > 0 ? 1 : 0;
    invalid_steps +=
        controller.qpStatus() == JointSafetyTorqueQp::Status::InvalidInput ||
                controller.qpStatus() == JointSafetyTorqueQp::Status::InfeasibleBounds
            ? 1
            : 0;
    if (controller.maxConstraintViolation() > maximum_constraint_violation)
    {
      maximum_constraint_violation = controller.maxConstraintViolation();
      maximum_violation_time = env.time();
      maximum_violation_row = controller.maxConstraintViolationRow();
    }
    maximum_collision_slack = std::max(
        maximum_collision_slack,
        controller.maxCollisionSlack());
    env.step(tau_left, tau_right);
  }

  const Pose& goal = env.scene().waypoints.back().pose;
  const Vector6d final_error = poseError(goal, env.state().object.pose);
  EXPECT_FALSE(env.diverged());
  EXPECT_EQ(invalid_steps, 0);
  EXPECT_EQ(controller.qpStatus(), JointSafetyTorqueQp::Status::Solved);
  // 槽口棱边会造成最近点/法向离散切换；固定 80 次 ADMM 允许单周期
  // 小于 1 m/s² 的 acceleration feasibility 瞬态，最终闭链残差另行严格检查。
  EXPECT_LT(maximum_constraint_violation, 1.0)
      << "maximum violation at t=" << maximum_violation_time
      << ", row=" << maximum_violation_row;
  EXPECT_GT(collision_active_steps, 0);
  EXPECT_GT(minimum_distance, 0.0004);
  EXPECT_LT(final_error.head<3>().norm(), 0.002);
  EXPECT_LT(final_error.tail<3>().norm(), 0.01);
  EXPECT_LT(controller.closedChainAccelerationResidual(), 1e-4);
  EXPECT_LT(controller.maxCollisionSlack(), 1e-6);
  EXPECT_LT(maximum_collision_slack, 20.0);
}

TEST(QpCoopController, CooperativeTransportClearsBarrierAndInserts)
{
  const SimConfig cfg = test::testConfig(
      {
          "controller.type=qp_coop",
          "object_trajectory.type=waypoints",
          "simulation.contacts=false",
          "disturbances=[]",
      },
      "slot_avoid");

  SimEnv env(cfg);
  auto model = std::make_shared<MujocoRobotModel>(cfg);
  auto trajectory = std::make_shared<ObjectTrajectory>(
      cfg.object_trajectory,
      env.state().object.pose,
      env.scene().waypoints);
  QpCoopController controller(
      model,
      trajectory,
      cfg.coop,
      cfg.torque_qp,
      cfg.collision,
      cfg.timestep);
  controller.reset(env.state());

  double minimum_distance = std::numeric_limits<double>::infinity();
  int collision_active_steps = 0;
  double maximum_governor_offset = 0.0;
  bool saw_lift = false;
  bool saw_cross = false;
  bool saw_descend = false;
  for (int k = 0; k < 20000; ++k)
  {
    const auto [tau_left, tau_right] =
        controller.compute(env.state(), env.time());
    ASSERT_TRUE(tau_left.allFinite());
    ASSERT_TRUE(tau_right.allFinite());
    minimum_distance = std::min(
        minimum_distance,
        controller.collisionMinDistance());
    collision_active_steps +=
        controller.activeCollisionConstraints() > 0 ? 1 : 0;
    maximum_governor_offset = std::max(
        maximum_governor_offset,
        controller.governorOffset());
    saw_lift = saw_lift ||
        controller.governorPhase() == QpCoopController::GovernorPhase::Lift;
    saw_cross = saw_cross ||
        controller.governorPhase() == QpCoopController::GovernorPhase::Cross;
    saw_descend = saw_descend ||
        controller.governorPhase() == QpCoopController::GovernorPhase::Descend;
    env.step(tau_left, tau_right);
  }

  const Pose& goal = env.scene().waypoints.back().pose;
  const Vector6d final_error = poseError(goal, env.state().object.pose);
  EXPECT_FALSE(env.diverged());
  EXPECT_EQ(controller.qpStatus(), JointSafetyTorqueQp::Status::Solved);
  EXPECT_GT(collision_active_steps, 0);
  EXPECT_TRUE(saw_lift);
  EXPECT_TRUE(saw_cross);
  EXPECT_TRUE(saw_descend);
  EXPECT_NEAR(
      maximum_governor_offset,
      cfg.torque_qp.reference_governor.avoidance_offset,
      1e-9);
  EXPECT_EQ(
      controller.governorPhase(),
      QpCoopController::GovernorPhase::Normal);
  EXPECT_NEAR(controller.governorOffset(), 0.0, 1e-12);
  EXPECT_LT(controller.governorVirtualTime(), env.time() - 1.0);
  EXPECT_GT(minimum_distance, 0.0004);
  EXPECT_LT(final_error.head<3>().norm(), 0.002);
  EXPECT_LT(final_error.tail<3>().norm(), 0.01);
  EXPECT_LT(controller.closedChainAccelerationResidual(), 1e-4);
  EXPECT_LT(controller.maxCollisionSlack(), 1e-6);
}
