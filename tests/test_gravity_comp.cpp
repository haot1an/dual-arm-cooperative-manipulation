// 重力补偿与基线控制器（lift 场景）：
//  1) 不夹物体（weld 关闭、物体移到远处地面上）时，纯重力补偿 τ = g(q) 下机械臂基本静止；
//  2) 夹着物体时，GravityCompJointPD 能持住物体（不掉落、不发散）；
//  3) 协同控制器桩（对称 / 非对称）能运行（输出 = 重力补偿）。
#include "dual_arm/asym_coop_controller.hpp"
#include "dual_arm/baseline_controllers.hpp"
#include "dual_arm/coop_controller.hpp"
#include "dual_arm/mujoco_model.hpp"
#include "dual_arm/object_trajectory.hpp"
#include "dual_arm/sim_env.hpp"
#include "test_common.hpp"

#include <gtest/gtest.h>

#include <memory>

using namespace dual_arm;

TEST(GravityComp, ArmsStayStillWithoutObject) {
  const SimConfig cfg = test::testConfig();
  SimEnv env(cfg);
  MujocoRobotModel model(cfg);

  // 断开 weld，把物体移到远处的地面上
  mjData* d = env.data();
  for (Arm a : kArms) d->eq_active[env.indices()[a].weld_eq] = 0;
  d->qpos[env.indices().object_qpos_adr + 0] = 3.0;
  d->qpos[env.indices().object_qpos_adr + 2] = 0.1;
  env.forward();

  const DualArmState s0 = env.state();
  double max_dev = 0.0, max_vel = 0.0;
  for (int k = 0; k < 2000; ++k) {  // 2 s
    const DualArmState& s = env.state();
    model.update(s);
    env.step(model.gravity(Arm::Left), model.gravity(Arm::Right));
    for (Arm a : kArms) {
      max_dev = std::max(max_dev, (env.state().arm(a).q - s0.arm(a).q).cwiseAbs().maxCoeff());
      max_vel = std::max(max_vel, env.state().arm(a).dq.cwiseAbs().maxCoeff());
    }
  }
  EXPECT_FALSE(env.diverged());
  EXPECT_LT(max_dev, 1e-4) << "joint drift under gravity compensation [rad]";
  EXPECT_LT(max_vel, 1e-3) << "joint velocity under gravity compensation [rad/s]";
}

TEST(GravityComp, WithoutCompensationArmsFall) {
  // 对照组：τ = 0 时两臂明显下落，说明上一个测试不是“什么都不做也静止”
  const SimConfig cfg = test::testConfig();
  SimEnv env(cfg);
  mjData* d = env.data();
  for (Arm a : kArms) d->eq_active[env.indices()[a].weld_eq] = 0;
  d->qpos[env.indices().object_qpos_adr + 0] = 3.0;
  d->qpos[env.indices().object_qpos_adr + 2] = 0.1;
  env.forward();
  const Vector7d q0 = env.state().arm(Arm::Left).q;
  for (int k = 0; k < 300; ++k) env.step(Vector7d::Zero(), Vector7d::Zero());
  EXPECT_GT((env.state().arm(Arm::Left).q - q0).cwiseAbs().maxCoeff(), 0.05);
}

TEST(GravityCompJointPD, HoldsTheObject) {
  // 关掉接触：lift 的杆在抓取位姿离台面只有 1 mm，PD 的静差会让它压到台面上，合力就不再只由两个 weld 承担
  const SimConfig cfg = test::testConfig({"simulation.contacts=false"});
  SimEnv env(cfg);
  auto model = std::make_shared<MujocoRobotModel>(cfg);
  GravityCompJointPD ctrl(model, cfg.gravity_pd);
  ctrl.reset(env.state());

  const Vector3d p0 = env.state().object.pose.p;
  double max_drift = 0.0;
  for (int k = 0; k < 3000; ++k) {  // 3 s
    const auto [tl, tr] = ctrl.compute(env.state(), env.time());
    env.step(tl, tr);
    max_drift = std::max(max_drift, (env.state().object.pose.p - p0).norm());
  }
  EXPECT_FALSE(env.diverged());
  EXPECT_LT(max_drift, 0.005) << "object drift [m]";
  // 静止后物体速度≈0
  EXPECT_LT(env.state().object.twist.norm(), 1e-3);
  // 静止后两臂通过 weld 对物体的合力 ≈ 物体重力（向上）
  const double mg = env.scene().object.mass * 9.81;
  const Wrench sum = test::sumWeldWrenchAtObjectCenter(env, env.lastStep());
  EXPECT_NEAR(sum[2], mg, 0.05);
  EXPECT_LT(sum.head<2>().norm(), 0.05);
  EXPECT_LT(sum.tail<3>().norm(), 0.02);
}

TEST(CoopController, HoldsObjectAtInitialPose) {
  const SimConfig cfg = test::testConfig(
      {
          "controller.type=coop",
          "object_trajectory.type=hold",
          "simulation.contacts=false",
          "disturbances=[]",
      },
      "lift");

  SimEnv env(cfg);

  auto model =
      std::make_shared<MujocoRobotModel>(cfg);

  auto trajectory =
      std::make_shared<ObjectTrajectory>(
          cfg.object_trajectory,
          env.state().object.pose,
          env.scene().waypoints);

  CoopController ctrl(
      model,
      trajectory,
      cfg.coop);

  ctrl.reset(env.state());

  const Pose initial_pose =
      env.state().object.pose;

  double max_position_drift = 0.0;

  for (int k = 0; k < 3000; ++k) {
    const auto [tau_left, tau_right] =
        ctrl.compute(
            env.state(),
            env.time());

    ASSERT_TRUE(tau_left.allFinite());
    ASSERT_TRUE(tau_right.allFinite());

    env.step(tau_left, tau_right);

    max_position_drift = std::max(
        max_position_drift,
        (env.state().object.pose.p -
         initial_pose.p)
            .norm());
  }

  EXPECT_FALSE(env.diverged());

  EXPECT_LT(
      max_position_drift,
      1e-3);

  EXPECT_LT(
      env.state().object.twist.norm(),
      1e-3);

  // 稳态时，两臂对物体的合力应抵消物体重力。
  const double object_weight =
      env.scene().object.mass * 9.81;

  const Wrench total_wrench =
      test::sumWeldWrenchAtObjectCenter(
          env,
          env.lastStep());

  EXPECT_NEAR(
      total_wrench[2],
      object_weight,
      0.1);

  EXPECT_LT(
      total_wrench.head<2>().norm(),
      0.1);

  EXPECT_LT(
      total_wrench.tail<3>().norm(),
      0.05);
}

TEST(
    AsymCoopController,
    DerivesInternalForceDimensionAndRuns)
{
  for (const auto& [scene, n_rel] :
       {
           std::pair<std::string, int>{
               "assembly", 1},
           {"assembly_simple", 0},
           {"lift", 0},
       })
  {
    const SimConfig cfg =
        test::testConfig(
            {
                "controller.type=asym_coop",
                "object_trajectory.type=hold",
                "disturbances=[]",
            },
            scene);

    SimEnv env(cfg);

    auto model =
        std::make_shared<MujocoRobotModel>(cfg);

    auto trajectory =
        std::make_shared<ObjectTrajectory>(
            cfg.object_trajectory,
            env.state().object.pose,
            env.scene().waypoints);

    AsymmetricCoopController ctrl(
        model,
        trajectory,
        cfg.asym_coop);

    EXPECT_EQ(
        ctrl.relativeDof(),
        n_rel)
        << scene;

    EXPECT_EQ(
        ctrl.internalForceDim(),
        6 - n_rel)
        << scene;

    EXPECT_EQ(
        ctrl.holdingArm(),
        Arm::Left);

    ctrl.reset(env.state());

    for (int k = 0; k < 200; ++k)
    {
      const auto [tau_left, tau_right] =
          ctrl.compute(
              env.state(),
              env.time());

      ASSERT_TRUE(tau_left.allFinite())
          << scene;

      ASSERT_TRUE(tau_right.allFinite())
          << scene;

      env.step(
          tau_left,
          tau_right);
    }

    EXPECT_FALSE(env.diverged())
        << scene;
  }

}

TEST(
    AsymCoopController,
    ConstraintFeedbackChangesWorkingArmTorque)
{
  const SimConfig cfg =
      test::testConfig(
          {
              "controller.type=asym_coop",
              "object_trajectory.type=hold",
              "disturbances=[]",
          },
          "assembly");

  EXPECT_GT(
      cfg.asym_coop.working_constraint_stiffness.minCoeff(),
      0.0);

  EXPECT_GT(
      cfg.asym_coop.working_constraint_damping.minCoeff(),
      0.0);

  EXPECT_GT(
      cfg.asym_coop.axial_preload_force,
      0.0);

  SimEnv env(cfg);

  const DualArmState initial_state =
      env.state();

  auto trajectory =
      std::make_shared<ObjectTrajectory>(
          cfg.object_trajectory,
          initial_state.object.pose,
          env.scene().waypoints);

  AsymCoopConfig active_params =
      cfg.asym_coop;

  // 关闭螺旋角度任务，只比较五维约束反馈产生的力矩。
  active_params.task_gain = 0.0;
  active_params.task_damping = 0.0;
  active_params.task_torque_limit = 0.0;

  AsymCoopConfig disabled_params =
      active_params;

  disabled_params.working_constraint_stiffness.setZero();
  disabled_params.working_constraint_damping.setZero();

  auto active_model =
      std::make_shared<MujocoRobotModel>(cfg);

  auto disabled_model =
      std::make_shared<MujocoRobotModel>(cfg);

  AsymmetricCoopController active_controller(
      active_model,
      trajectory,
      active_params);

  AsymmetricCoopController disabled_controller(
      disabled_model,
      trajectory,
      disabled_params);

  active_controller.reset(initial_state);
  disabled_controller.reset(initial_state);

  DualArmState perturbed_state =
      initial_state;

  // 在 reset 之后偏移作业臂关节，产生包含五维约束分量的 TCP 位姿误差。
  perturbed_state.arm(Arm::Right).q[0] += 0.01;

  const auto active_tau =
      active_controller.compute(
          perturbed_state,
          0.0);

  const auto disabled_tau =
      disabled_controller.compute(
          perturbed_state,
          0.0);

  EXPECT_GT(
      (active_tau.first - disabled_tau.first).norm(),
      0.1);

  EXPECT_GT(
      (active_tau.second - disabled_tau.second).norm(),
      0.1);
}

TEST(
    AsymCoopController,
    AxialPreloadUsesForceFeedback)
{
  const SimConfig cfg =
      test::testConfig(
          {
              "controller.type=asym_coop",
              "object_trajectory.type=hold",
              "disturbances=[]",
          },
          "assembly");

  SimEnv env(cfg);

  const DualArmState initial_state =
      env.state();

  auto trajectory =
      std::make_shared<ObjectTrajectory>(
          cfg.object_trajectory,
          initial_state.object.pose,
          env.scene().waypoints);

  AsymCoopConfig params =
      cfg.asym_coop;

  params.task_gain = 0.0;
  params.task_damping = 0.0;
  params.task_torque_limit = 0.0;
  params.working_constraint_stiffness.setZero();
  params.working_constraint_damping.setZero();
  params.axial_preload_force = 5.0;
  params.axial_preload_gain = 0.5;
  params.axial_preload_limit = 10.0;
  params.axial_preload_ramp_time = 0.0;
  // 本测试在同一时刻比较两组传感器输入，只验证力反馈映射；关闭时间滤波。
  params.wrench_filter_time_constant = 0.0;

  auto model =
      std::make_shared<MujocoRobotModel>(cfg);

  AsymmetricCoopController controller(
      model,
      trajectory,
      params);

  controller.reset(initial_state);

  DualArmState unloaded_state =
      initial_state;

  unloaded_state.t = 1.0;
  unloaded_state.arm(Arm::Right).ft_ee_world.setZero();

  DualArmState loaded_state =
      unloaded_state;

  const ClosedChainConstraint* screw =
      env.scene().screwConstraint();

  ASSERT_NE(screw, nullptr);

  const Vector3d axis_world =
      loaded_state.object.pose.q *
      screw->axis.normalized();

  // 传感器已经测得目标压紧力时，反馈增量应消失。
  loaded_state.arm(Arm::Right).ft_ee_world.head<3>() =
      -params.axial_preload_force *
      axis_world;

  const auto unloaded_tau =
      controller.compute(
          unloaded_state,
          unloaded_state.t);

  const auto loaded_tau =
      controller.compute(
          loaded_state,
          loaded_state.t);

  EXPECT_GT(
      (unloaded_tau.first - loaded_tau.first).norm(),
      0.01);

  EXPECT_GT(
      (unloaded_tau.second - loaded_tau.second).norm(),
      0.01);
}

TEST(
    AsymCoopController,
    AssemblyStateMachineReachesHold)
{
  const SimConfig cfg =
      test::testConfig(
          {
              "controller.type=asym_coop",
              "object_trajectory.type=waypoints",
              "simulation.contacts=false",
              "disturbances=[]",
          },
          "assembly");

  SimEnv env(cfg);
  const Pose initial_object_pose =
      env.state().object.pose;

  auto model =
      std::make_shared<MujocoRobotModel>(cfg);

  auto trajectory =
      std::make_shared<ObjectTrajectory>(
          cfg.object_trajectory,
          initial_object_pose,
          env.scene().waypoints);

  AsymmetricCoopController controller(
      model,
      trajectory,
      cfg.asym_coop);

  controller.reset(env.state());

  bool saw_angle_tighten = false;
  bool saw_torque_tighten = false;
  bool saw_hold = false;
  double max_object_drift = 0.0;

  // 默认参数约在 12.1 s 进入 HOLD；运行 14 s 给完成判定留出裕量。
  for (int k = 0; k < 14000; ++k)
  {
    const auto [tau_left, tau_right] =
        controller.compute(
            env.state(),
            env.time());

    ASSERT_TRUE(tau_left.allFinite());
    ASSERT_TRUE(tau_right.allFinite());

    env.step(tau_left, tau_right);

    saw_angle_tighten |=
        controller.phase() ==
        AsymmetricCoopController::Phase::AngleTighten;
    saw_torque_tighten |=
        controller.phase() ==
        AsymmetricCoopController::Phase::TorqueTighten;
    saw_hold |=
        controller.phase() ==
        AsymmetricCoopController::Phase::Hold;

    max_object_drift =
        std::max(
            max_object_drift,
            (env.state().object.pose.p -
             initial_object_pose.p)
                .norm());
  }

  EXPECT_FALSE(env.diverged());
  EXPECT_TRUE(saw_angle_tighten);
  EXPECT_TRUE(saw_torque_tighten);
  EXPECT_TRUE(saw_hold);
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
}
