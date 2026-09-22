// 验证 1 kHz 控制环（控制器计算 + 仿真步 + 轨迹 + SceneMonitor（含 CollisionModel 距离查询）+ 日志写入）
// 中没有任何堆分配。
//
// 做法：在本可执行文件中替换 glibc 的 malloc / calloc / realloc（符号插桩，对 libstdc++ 的
// operator new、Eigen、libmujoco 都生效），在计数窗口内统计调用次数。
// 注意 Eigen 的 EIGEN_RUNTIME_NO_MALLOC 在 Release (NDEBUG) 下不起作用，所以这里不用它。
#include "dual_arm/asym_coop_controller.hpp"
#include "dual_arm/baseline_controllers.hpp"
#include "dual_arm/collision_model.hpp"
#include "dual_arm/coop_controller.hpp"
#include "dual_arm/coop_kinematics.hpp"
#include "dual_arm/logger.hpp"
#include "dual_arm/math_utils.hpp"
#include "dual_arm/mujoco_model.hpp"
#include "dual_arm/object_trajectory.hpp"
#include "dual_arm/qp_asym_coop_controller.hpp"
#include "dual_arm/qp_coop_controller.hpp"
#include "dual_arm/scene_monitor.hpp"
#include "dual_arm/sim_env.hpp"
#include "test_common.hpp"

#include <gtest/gtest.h>

#include <atomic>
#include <cstddef>
#include <memory>

extern "C"
{
  void *__libc_malloc(std::size_t size);
  void *__libc_calloc(std::size_t n, std::size_t size);
  void *__libc_realloc(void *ptr, std::size_t size);
}

namespace
{
  std::atomic<bool> g_counting{false};
  std::atomic<long> g_allocations{0};

  inline void countAllocation()
  {
    if (g_counting.load(std::memory_order_relaxed))
      g_allocations.fetch_add(1, std::memory_order_relaxed);
  }
} // namespace

extern "C"
{
  void *malloc(std::size_t size)
  {
    countAllocation();
    return __libc_malloc(size);
  }
  void *calloc(std::size_t n, std::size_t size)
  {
    countAllocation();
    return __libc_calloc(n, size);
  }
  void *realloc(void *ptr, std::size_t size)
  {
    countAllocation();
    return __libc_realloc(ptr, size);
  }
}

using namespace dual_arm;

namespace
{

  long countAllocationsInControlLoop(const std::string &controller, const std::string &scene)
  {
    const SimConfig cfg =
        loadConfig("config/default.yaml", {"log.enabled=false", "controller.type=" + controller}, scene);
    SimEnv env(cfg);
    auto model = std::make_shared<MujocoRobotModel>(cfg);
    auto traj = std::make_shared<ObjectTrajectory>(cfg.object_trajectory, env.state().object.pose, env.scene().waypoints);
    std::unique_ptr<Controller> ctrl;
    if (controller == "coop")
    {
      ctrl = std::make_unique<CoopController>(model, traj, cfg.coop);
    }
    else if (controller == "asym_coop")
    {
      ctrl = std::make_unique<AsymmetricCoopController>(model, traj, cfg.asym_coop);
    }
    else if (controller == "qp_asym_coop")
    {
      ctrl = std::make_unique<QpAsymmetricCoopController>(
          model,
          traj,
          cfg.asym_coop,
          cfg.torque_qp,
          cfg.collision,
          cfg.timestep);
    }
    else if (controller == "qp_coop")
    {
      ctrl = std::make_unique<QpCoopController>(
          model,
          traj,
          cfg.coop,
          cfg.torque_qp,
          cfg.collision,
          cfg.timestep);
    }
    else if (controller == "cartesian_impedance")
    {
      ctrl = std::make_unique<IndependentCartesianImpedance>(
          model,
          traj,
          cfg.cartesian_impedance);
    }
    else
    {
      ctrl = std::make_unique<GravityCompJointPD>(model, cfg.gravity_pd);
    }

    ctrl->reset(env.state());
    SceneMonitor monitor(env);
    CsvLogger logger;
    logger.setExtraColumns(monitor.columns());
    EXPECT_TRUE(logger.open(testing::TempDir() + "no_alloc_" + controller + "_" + scene + ".csv"));
    LogRow row;

    auto cycle = [&]()
    {
      const auto [tl, tr] = ctrl->compute(env.state(), env.time());
      env.step(tl, tr);
      const DualArmState &rec = env.lastStep();
      const ObjectReference ref = traj->evaluate(rec.t);
      monitor.update(rec);
      row.step = &rec;
      row.object_ref = ref.pose;
      row.screw_ref = ref.screw_angle;
      row.object_error = poseError(ref.pose, rec.object.pose);
      row.internal_wrench = coop::internalWrenchForLogging(rec, *model);
      row.extra = monitor.values().data();
      logger.write(row);
    };

    for (int k = 0; k < 20; ++k)
      cycle(); // 预热：一次性的提示输出等
    g_allocations = 0;
    g_counting = true;
    for (int k = 0; k < 3500; ++k)
      cycle(); // 包含 3.0–3.5 s 的扰动段
    g_counting = false;
    return g_allocations.load();
  }

  long countAllocationsInCollisionQueries(const std::string &scene, int threads = 0)
  {
    const SimConfig cfg =
        loadConfig("config/default.yaml", {"log.enabled=false", "collision.threads=" + std::to_string(threads)}, scene);
    CollisionModel cm(cfg.scene, {nominalBasePose(cfg, Arm::Left), nominalBasePose(cfg, Arm::Right)}, cfg.collision);
    Vector7d ql = cfg.scene.q_init[0], qr = cfg.scene.q_init[1];
    cm.query(ql, qr);
    g_allocations = 0;
    g_counting = true;
    double sum = 0.0;
    for (int k = 0; k < 200; ++k)
    {
      ql[3] += 1e-3;
      qr[3] -= 1e-3;
      sum += cm.query(ql, qr).size();
    }
    g_counting = false;
    EXPECT_GT(sum, 0.0);
    return g_allocations.load();
  }

} // namespace

TEST(NoAlloc, SanityCheckCounterWorks)
{
  g_allocations = 0;
  g_counting = true;
  auto *p = new std::vector<double>(100);
  g_counting = false;
  delete p;
  EXPECT_GE(g_allocations.load(), 1);
}

TEST(NoAlloc, GravityPdControlLoopDoesNotAllocate)
{
  for (const char *scene : {"lift", "slot", "assembly", "assembly_simple"})
  {
    EXPECT_EQ(countAllocationsInControlLoop("gravity_pd", scene), 0) << scene;
  }
}
TEST(NoAlloc, CartesianImpedanceControlLoopDoesNotAllocate)
{
  EXPECT_EQ(
      countAllocationsInControlLoop(
          "cartesian_impedance",
          "lift"),
      0);
}

TEST(IndependentCartesianImpedance, HoldsObjectAtInitialPose) {
  const SimConfig cfg = test::testConfig(
      {
          "controller.type=cartesian_impedance",
          "object_trajectory.type=hold",
          "simulation.contacts=false",
          "disturbances=[]",
      },
      "lift");

  SimEnv env(cfg);
  auto model = std::make_shared<MujocoRobotModel>(cfg);
  auto trajectory = std::make_shared<ObjectTrajectory>(
      cfg.object_trajectory,
      env.state().object.pose,
      env.scene().waypoints);

  IndependentCartesianImpedance ctrl(
      model,
      trajectory,
      cfg.cartesian_impedance);

  const Pose initial_pose = env.state().object.pose;
  double max_position_drift = 0.0;

  for (int k = 0; k < 3000; ++k) {
    const auto [tau_left, tau_right] =
        ctrl.compute(env.state(), env.time());

    ASSERT_TRUE(tau_left.allFinite());
    ASSERT_TRUE(tau_right.allFinite());

    env.step(tau_left, tau_right);

    max_position_drift = std::max(
        max_position_drift,
        (env.state().object.pose.p - initial_pose.p).norm());
  }

  EXPECT_FALSE(env.diverged());
  EXPECT_LT(max_position_drift, 1e-3);
  EXPECT_LT(env.state().object.twist.norm(), 1e-3);
}
TEST(NoAlloc, CoopControlLoopDoesNotAllocate)
{
  EXPECT_EQ(countAllocationsInControlLoop("coop", "lift"), 0);
  EXPECT_EQ(countAllocationsInControlLoop("asym_coop", "assembly"), 0);
  EXPECT_EQ(countAllocationsInControlLoop("qp_asym_coop", "assembly"), 0);
  EXPECT_EQ(countAllocationsInControlLoop("qp_coop", "slot"), 0);
  EXPECT_EQ(countAllocationsInControlLoop("qp_coop", "slot_avoid"), 0);
}

TEST(NoAlloc, CollisionQueryDoesNotAllocate)
{
  for (const char *scene : {"lift", "slot", "assembly"})
  {
    EXPECT_EQ(countAllocationsInCollisionQueries(scene), 0) << scene;
  }
}

TEST(NoAlloc, CollisionQueryWithThreadPoolDoesNotAllocate)
{
  EXPECT_EQ(countAllocationsInCollisionQueries("assembly", 4), 0);
}
