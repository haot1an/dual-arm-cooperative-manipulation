// 验证 RobotModel 的坐标约定：J 为 [线速度; 角速度]、世界系、参考点 ee_site；
// 同时检查 J̇·dq、M、h、g 的基本性质，以及控制器模型与 plant 在无标定误差时一致。
#include "dual_arm/math_utils.hpp"
#include "dual_arm/mujoco_model.hpp"
#include "dual_arm/sim_env.hpp"
#include "test_common.hpp"

#include <Eigen/Dense>
#include <gtest/gtest.h>

#include <random>

using namespace dual_arm;

namespace {

DualArmState makeState(const Vector7d& ql, const Vector7d& dql, const Vector7d& qr,
                       const Vector7d& dqr) {
  DualArmState s;
  s.arm(Arm::Left).q = ql;
  s.arm(Arm::Left).dq = dql;
  s.arm(Arm::Right).q = qr;
  s.arm(Arm::Right).dq = dqr;
  return s;
}

}  // namespace

TEST(Jacobian, MatchesFiniteDifferenceOfEePose) {
  const SimConfig cfg = test::testConfig();
  MujocoRobotModel model(cfg);
  std::mt19937 rng(42);
  const double eps = 1e-6;

  for (int trial = 0; trial < 20; ++trial) {
    for (Arm a : kArms) {
      const Vector7d q = test::randomJointConfig(rng, model.jointLowerLimit(a), model.jointUpperLimit(a));
      const Vector7d dq = test::randomVector7(rng, 1.0);
      const Vector7d zero = Vector7d::Zero();

      auto stateWith = [&](const Vector7d& qa) {
        return a == Arm::Left ? makeState(qa, zero, cfg.scene.q_init[1], zero)
                              : makeState(cfg.scene.q_init[0], zero, qa, zero);
      };
      model.update(stateWith(q + eps * dq));
      const Pose Tp = model.eePose(a);
      model.update(stateWith(q - eps * dq));
      const Pose Tm = model.eePose(a);
      model.update(stateWith(q));
      const Matrix6x7d J = model.jacobian(a);

      Twist fd;
      fd.head<3>() = (Tp.p - Tm.p) / (2 * eps);                    // ee_site 原点的线速度
      fd.tail<3>() = rotationError(Tp.R(), Tm.R()) / (2 * eps);     // 世界系角速度
      const Twist Jdq = J * dq;
      EXPECT_LT((Jdq - fd).norm(), 1e-6 * (1.0 + fd.norm()))
          << armName(a) << " trial " << trial << "\nJ*dq = " << Jdq.transpose()
          << "\nfd   = " << fd.transpose();
    }
  }
}

TEST(Jacobian, LinearRowsReferToEeSiteNotBodyOrigin) {
  // 绕 ee z 轴（接近方向）纯转动 joint7 时，ee_site 在 joint7 轴上，线速度应≈0；
  // 若参考点错用了别的点（如 link7 质心），这里会出现非零线速度。
  const SimConfig cfg = test::testConfig();
  MujocoRobotModel model(cfg);
  Vector7d dq = Vector7d::Zero();
  dq[6] = 1.0;
  model.update(makeState(cfg.scene.q_init[0], dq, cfg.scene.q_init[1], dq));
  for (Arm a : kArms) {
    const Twist v = model.eeTwist(a);
    EXPECT_LT(v.head<3>().norm(), 1e-9) << armName(a);
    // 角速度沿 ee_site 的 z 轴（joint7 轴与法兰轴重合）
    const Vector3d z = model.eePose(a).R().col(2);
    EXPECT_NEAR(std::abs(v.tail<3>().dot(z)), 1.0, 1e-9);
  }
}

TEST(Jacobian, JdotTimesQdotMatchesFiniteDifference) {
  const SimConfig cfg = test::testConfig();
  MujocoRobotModel model(cfg);
  std::mt19937 rng(7);
  const double eps = 1e-6;
  for (int trial = 0; trial < 10; ++trial) {
    const Vector7d q = test::randomJointConfig(rng, model.jointLowerLimit(Arm::Left),
                                               model.jointUpperLimit(Arm::Left));
    const Vector7d dq = test::randomVector7(rng, 1.0);
    model.update(makeState(q + eps * dq, dq, cfg.scene.q_init[1], Vector7d::Zero()));
    const Matrix6x7d Jp = model.jacobian(Arm::Left);
    model.update(makeState(q - eps * dq, dq, cfg.scene.q_init[1], Vector7d::Zero()));
    const Matrix6x7d Jm = model.jacobian(Arm::Left);
    model.update(makeState(q, dq, cfg.scene.q_init[1], Vector7d::Zero()));
    const Vector6d fd = (Jp - Jm) / (2 * eps) * dq;
    EXPECT_LT((model.jacobianDotTimesQdot(Arm::Left) - fd).norm(), 1e-5 * (1.0 + fd.norm()));
  }
}

TEST(Dynamics, MassMatrixBiasAndGravityProperties) {
  const SimConfig cfg = test::testConfig();
  MujocoRobotModel model(cfg);
  std::mt19937 rng(3);
  for (int trial = 0; trial < 10; ++trial) {
    for (Arm a : kArms) {
      const Vector7d q = test::randomJointConfig(rng, model.jointLowerLimit(a), model.jointUpperLimit(a));
      const Vector7d dq = test::randomVector7(rng, 1.0);
      auto st = [&](const Vector7d& v) {
        return makeState(q, a == Arm::Left ? v : Vector7d::Zero(), q, a == Arm::Right ? v : Vector7d::Zero());
      };
      model.update(st(Vector7d::Zero()));
      const Matrix7d M = model.massMatrix(a);
      const Vector7d g = model.gravity(a);
      EXPECT_LT((model.bias(a) - g).norm(), 1e-9);  // dq = 0 时 h = g
      EXPECT_LT((M - M.transpose()).norm(), 1e-9);
      EXPECT_GT(Eigen::SelfAdjointEigenSolver<Matrix7d>(M).eigenvalues().minCoeff(), 0.099);  // ≥ armature (0.1)
      model.update(st(dq));
      const Vector7d c1 = model.bias(a) - g;
      model.update(st(2.0 * dq));
      const Vector7d c2 = model.bias(a) - g;
      EXPECT_LT((c2 - 4.0 * c1).norm(), 1e-8 * (1.0 + c1.norm()));  // C(q,dq)dq 关于 dq 二次齐次
      EXPECT_LT((model.gravity(a) - g).norm(), 1e-12);             // g 与 dq 无关
    }
  }
}

TEST(Jacobian, ControllerModelMatchesPlantWithoutCalibrationError) {
  const SimConfig cfg = test::testConfig();
  SimEnv env(cfg);
  MujocoRobotModel model(cfg);
  std::mt19937 rng(11);
  mjData* d = env.data();
  const auto& idx = env.indices();
  // 让 plant 处于随机状态（关掉 weld，避免约束影响速度），比较位姿与 twist
  for (Arm a : kArms) d->eq_active[idx[a].weld_eq] = 0;
  for (Arm a : kArms) {
    const Vector7d q = test::randomJointConfig(rng, model.jointLowerLimit(a), model.jointUpperLimit(a));
    const Vector7d dq = test::randomVector7(rng, 1.0);
    for (int j = 0; j < kArmDof; ++j) {
      d->qpos[idx[a].qpos_adr + j] = q[j];
      d->qvel[idx[a].dof_adr + j] = dq[j];
    }
  }
  env.forward();
  const DualArmState& s = env.state();
  model.update(s);
  for (Arm a : kArms) {
    const Pose Tp = s.arm(a).ee_pose;
    const Pose Tm = model.eePose(a);
    EXPECT_LT((Tp.p - Tm.p).norm(), 1e-12);
    EXPECT_LT(rotationError(Tp.R(), Tm.R()).norm(), 1e-9);
    EXPECT_LT((s.arm(a).ee_twist - model.eeTwist(a)).norm(), 1e-9);
  }
}
