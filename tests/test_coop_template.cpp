// 【测试模板】协同运动学 / 静力学。
//
// 这些测试针对 coop_kinematics.hpp 中的桩函数。桩函数目前返回 NaN，测试检测到 NaN
// 会 GTEST_SKIP()；你实现对应函数后，测试会自动生效，无需修改本文件。
// 需要验证的两条核心性质：
//   (a) G · (I − G⁺G) h = 0 —— 零空间内力不改变物体合 wrench（式 4.3/4.4）
//   (b) 理想闭链下 J_r · dq = 0 —— 相对运动为零（式 3.8）
#include "dual_arm/coop_kinematics.hpp"
#include "dual_arm/math_utils.hpp"
#include "dual_arm/mujoco_model.hpp"
#include "test_common.hpp"

#include <Eigen/Dense>
#include <gtest/gtest.h>

#include <random>

using namespace dual_arm;

#define SKIP_IF_NOT_IMPLEMENTED(x, fn)                                          \
  do {                                                                          \
    if (hasNaN(x)) GTEST_SKIP() << "coop::" fn "() 尚未实现（返回 NaN），跳过"; \
  } while (0)

namespace {

Vector3d randomVec3(std::mt19937& rng, double s) {
  std::uniform_real_distribution<double> u(-s, s);
  return Vector3d(u(rng), u(rng), u(rng));
}

/// 独立实现的 Moore–Penrose 伪逆（用于对照，不依赖被测代码）
Matrix12x6d referencePinv(const Matrix6x12d& G) {
  return G.transpose() * (G * G.transpose()).inverse();
}

}  // namespace

// ---------------------------------------------------------------------------
// §3 / §4 抓取矩阵
// ---------------------------------------------------------------------------
TEST(CoopTemplate, GraspMatrixMapsWrenchToObjectCenter) {
  std::mt19937 rng(1);
  const Vector3d p_o = randomVec3(rng, 1.0);
  const Vector3d p_i = p_o + randomVec3(rng, 0.3);
  const Vector3d r_i = p_o - p_i;  // 约定：抓取点 → 物体中心
  const Matrix6d Gi = coop::graspMatrixArm(r_i);
  SKIP_IF_NOT_IMPLEMENTED(Gi, "graspMatrixArm");

  const Wrench h_i = (Wrench() << randomVec3(rng, 10.0), randomVec3(rng, 1.0)).finished();
  // G_i h_i 应等于“把 h_i 的参考点从 p_i 平移到 p_o”
  EXPECT_LT((Gi * h_i - shiftWrenchRefPoint(h_i, p_i, p_o)).norm(), 1e-12);
  // G_iᵀ ν_o 应等于物体 twist 在抓取点处的值（刚性抓取，式 3.1）
  const Twist v_o = (Twist() << randomVec3(rng, 1.0), randomVec3(rng, 1.0)).finished();
  EXPECT_LT((Gi.transpose() * v_o - shiftTwistRefPoint(v_o, p_o, p_i)).norm(), 1e-12);
}

TEST(CoopTemplate, InternalWrenchLiesInNullSpaceOfG) {
  std::mt19937 rng(2);
  const Vector3d r1 = randomVec3(rng, 0.3), r2 = randomVec3(rng, 0.3);
  const Matrix6x12d G = coop::graspMatrix(r1, r2);
  SKIP_IF_NOT_IMPLEMENTED(G, "graspMatrix");

  // G 行满秩，零空间 6 维
  Eigen::FullPivLU<Matrix6x12d> lu(G);
  EXPECT_EQ(lu.rank(), 6);
  EXPECT_EQ(lu.dimensionOfKernel(), 6);

  for (int k = 0; k < 20; ++k) {
    Vector12d h;
    for (int i = 0; i < 12; ++i) h[i] = std::uniform_real_distribution<double>(-10, 10)(rng);
    const Vector12d h_int = coop::internalWrench(G, h);
    SKIP_IF_NOT_IMPLEMENTED(h_int, "internalWrench");
    EXPECT_LT((G * h_int).norm(), 1e-9) << "(a) G·(I−G⁺G)h 必须为 0";
    // 与独立实现的投影一致
    const Vector12d ref = (Matrix12d::Identity() - referencePinv(G) * G) * h;
    EXPECT_LT((h_int - ref).norm(), 1e-9);
    // 投影幂等
    EXPECT_LT((coop::internalWrench(G, h_int) - h_int).norm(), 1e-9);
  }
}

TEST(CoopTemplate, SqueezingIsPureInternalForce) {
  // 两臂沿连线方向对挤（本场景几何：抓取点在箱子中心两侧 0.17 m、上方 0.04 m）：
  // 左臂在 p1 处向 +x 推，右臂在 p2 处向 −x 推。两个力对 p_o 的力矩大小相等、方向相反。
  const Vector3d p_o(0, 0, 0.885), p1(-0.17, 0, 0.925), p2(0.17, 0, 0.925);
  const Vector3d r1 = p_o - p1, r2 = p_o - p2;
  const Matrix6x12d G = coop::graspMatrix(r1, r2);
  SKIP_IF_NOT_IMPLEMENTED(G, "graspMatrix");
  Vector12d h = Vector12d::Zero();
  h[0] = 30.0;   // f_1x
  h[6] = -30.0;  // f_2x
  EXPECT_LT((G * h).norm(), 1e-12) << "对挤不改变物体合 wrench";
  // 同方向推：物体受到 60 N 的合力
  Vector12d h_push = Vector12d::Zero();
  h_push[0] = 30.0;
  h_push[6] = 30.0;
  EXPECT_NEAR((G * h_push)[0], 60.0, 1e-12);

  const Vector6d h_r = coop::internalWrenchCoordinates(r1, r2, h);
  SKIP_IF_NOT_IMPLEMENTED(h_r, "internalWrenchCoordinates");
  EXPECT_LT(h_r[0], 0.0) << "约定：沿左→右方向，h_r 分量 < 0 表示挤压";
  EXPECT_NEAR(h_r[0], -30.0, 1e-12);
}
TEST(CoopTemplate, InternalCoordinatesProducePureInternalWrench) {
  const Vector3d r_left(
      -0.25,
      0.0,
      -0.01);

  const Vector3d r_right(
      0.25,
      0.0,
      -0.01);

  Vector6d h_r;
  h_r <<
      10.0, 0.0, 0.0,
      0.0, 0.0, 0.0;

  const Matrix6x12d G =
      coop::graspMatrix(
          r_left,
          r_right);

  const Vector12d h_internal =
      coop::internalWrenchFromCoordinates(
          r_left,
          r_right,
          h_r);

  // 纯内部力不能改变物体合 wrench。
  EXPECT_LT(
      (G * h_internal).norm(),
      1e-10);

  // 转换回来后应恢复原始六维坐标。
  const Vector6d recovered =
      coop::internalWrenchCoordinates(
          r_left,
          r_right,
          h_internal);

  EXPECT_LT(
      (recovered - h_r).norm(),
      1e-10);
}
TEST(CoopTemplate, DistributionReproducesObjectWrench) {
  std::mt19937 rng(3);
  const Vector3d r1(0.2, 0.01, -0.02), r2(-0.2, 0.0, 0.01);
  const Matrix6x12d G = coop::graspMatrix(r1, r2);
  SKIP_IF_NOT_IMPLEMENTED(G, "graspMatrix");
  const Wrench w_o = (Wrench() << randomVec3(rng, 20.0), randomVec3(rng, 2.0)).finished();
  Vector12d h_int;
  for (int i = 0; i < 12; ++i) h_int[i] = std::uniform_real_distribution<double>(-10, 10)(rng);

  const Vector12d h = coop::distributeObjectWrench(G, w_o, h_int, Matrix12d::Identity());
  SKIP_IF_NOT_IMPLEMENTED(h, "distributeObjectWrench");
  EXPECT_LT((G * h - w_o).norm(), 1e-9) << "内力项不能改变物体合 wrench";

  // 加权伪逆：左臂 λ1 = 0.8，纯力、r_i = 0 时左臂应承担 80%（式 5.3）
  const Matrix6x12d G0 = coop::graspMatrix(Vector3d::Zero(), Vector3d::Zero());
  Matrix12d W = Matrix12d::Identity();
  W.topLeftCorner<6, 6>() /= 0.8;
  W.bottomRightCorner<6, 6>() /= 0.2;
  const Matrix12x6d Gp = coop::weightedPseudoInverse(G0, W);
  SKIP_IF_NOT_IMPLEMENTED(Gp, "weightedPseudoInverse");
  const Wrench f = (Wrench() << 0, 0, 10, 0, 0, 0).finished();
  const Vector12d hw = Gp * f;
  EXPECT_NEAR(hw[2], 8.0, 1e-9);
  EXPECT_NEAR(hw[8], 2.0, 1e-9);
}
TEST(CoopTemplate, ScrewMotionBasisMatchesScrewKinematics)
{
  const Vector3d axis_world =
      Vector3d::UnitZ();

  const Vector3d axis_point_world =
      Vector3d::Zero();

  const Vector3d reference_point_world(
      1.0,
      0.0,
      0.0);

  const double lead =
      0.01;

  const Twist S =
      coop::screwMotionBasisAtPoint(
          axis_world,
          axis_point_world,
          reference_point_world,
          lead);

  Twist expected =
      Twist::Zero();

  expected.head<3>() <<
      0.0,
      1.0,
      lead / (2.0 * M_PI);

  expected.tail<3>() =
      Vector3d::UnitZ();

  EXPECT_LT(
      (S - expected).norm(),
      1e-12);
}

TEST(CoopTemplate, ScrewConstraintProjectorRemovesAllowedMotion)
{
  const Vector3d axis_world =
      Vector3d(0.2, -0.3, 1.0).normalized();

  const Vector3d axis_point_world(
      -0.1,
      0.2,
      0.8);

  const Vector3d reference_point_world(
      0.15,
      -0.4,
      1.1);

  const Twist S =
      coop::screwMotionBasisAtPoint(
          axis_world,
          axis_point_world,
          reference_point_world,
          0.00125);

  const Matrix6d P =
      coop::screwConstraintProjector(S);

  EXPECT_LT(
      (P * S).norm(),
      1e-12);

  EXPECT_LT(
      (P * P - P).norm(),
      1e-12);

  EXPECT_LT(
      (P.transpose() - P).norm(),
      1e-12);

  const Wrench arbitrary_wrench =
      (Wrench() <<
          10.0, -4.0, 7.0,
          0.8, -0.2, 1.5)
          .finished();

  const Wrench constraint_wrench =
      P * arbitrary_wrench;

  EXPECT_NEAR(
      S.dot(constraint_wrench),
      0.0,
      1e-12);

  Eigen::FullPivLU<Matrix6d> lu(P);

  EXPECT_EQ(
      lu.rank(),
      5);
}
// ---------------------------------------------------------------------------
// §3 绝对 / 相对雅可比与闭链约束
// ---------------------------------------------------------------------------
TEST(CoopTemplate, RelativeJacobianVanishesOnIdealClosedChain) {
  const SimConfig cfg = test::testConfig();
  MujocoRobotModel model(cfg);
  DualArmState s;
  for (Arm a : kArms) s.arm(a).q = cfg.scene.q_init[armIndex(a)];
  model.update(s);

  const Matrix6x7d J1 = model.jacobian(Arm::Left), J2 = model.jacobian(Arm::Right);
  // 物体中心由名义抓取几何得到
  const Pose T_o = model.eePose(Arm::Left) * model.scene().grasp[0].site_in_body.inverse();
  const Vector3d r1 = T_o.p - model.eePose(Arm::Left).p;
  const Vector3d r2 = T_o.p - model.eePose(Arm::Right).p;

  const Matrix6x14d Jr = coop::relativeJacobian(J1, J2, r1, r2);
  const Matrix6x14d Ja = coop::absoluteJacobian(J1, J2, r1, r2);
  SKIP_IF_NOT_IMPLEMENTED(Jr, "relativeJacobian");
  SKIP_IF_NOT_IMPLEMENTED(Ja, "absoluteJacobian");
  const Matrix6d G1 = coop::graspMatrixArm(r1), G2 = coop::graspMatrixArm(r2);
  SKIP_IF_NOT_IMPLEMENTED(G1, "graspMatrixArm");

  std::mt19937 rng(4);
  for (int k = 0; k < 10; ++k) {
    // 任取物体 twist ν_o → 两臂末端 twist ν_i = G_iᵀ ν_o → 关节速度（7 自由度，取最小范数解）
    const Twist v_o = (Twist() << randomVec3(rng, 0.2), randomVec3(rng, 0.5)).finished();
    Vector14d dq;
    dq.head<7>() = J1.completeOrthogonalDecomposition().solve(Vector6d(G1.transpose() * v_o));
    dq.tail<7>() = J2.completeOrthogonalDecomposition().solve(Vector6d(G2.transpose() * v_o));
    EXPECT_LT((Jr * dq).norm(), 1e-9) << "(b) 理想闭链下 J_r·dq 必须为 0";
    EXPECT_LT((Ja * dq - v_o).norm(), 1e-9) << "理想闭链下 J_a·dq = 物体 twist";
  }
  // 反例：只动一只手 → 相对 twist 非零
  Vector14d dq_bad = Vector14d::Zero();
  dq_bad[0] = 0.1;
  EXPECT_GT((Jr * dq_bad).norm(), 1e-3);
}

TEST(CoopTemplate, EeReferencesConsistentWithObjectReference) {
  const SimConfig cfg = test::testConfig();
  MujocoRobotModel model(cfg);
  const std::array<Pose, kNumArms> grasp{model.scene().grasp[0].site_in_body, model.scene().grasp[1].site_in_body};
  ObjectReference ref;
  ref.pose.p = Vector3d(0.05, -0.02, 0.95);
  ref.pose.q = quatFromRpyDeg(Vector3d(3, -2, 10));
  ref.twist << 0.1, -0.05, 0.02, 0.2, -0.1, 0.3;
  ref.accel << 0.3, 0.1, -0.2, 0.5, 0.2, -0.4;

  const auto ee = coop::eeReferencesFromObject(ref, grasp);
  SKIP_IF_NOT_IMPLEMENTED(ee[0].twist, "eeReferencesFromObject");
  for (Arm a : kArms) {
    const int i = armIndex(a);
    const Pose expected = ref.pose * grasp[i];
    EXPECT_LT((ee[i].pose.p - expected.p).norm(), 1e-12);
    EXPECT_LT(rotationError(ee[i].pose.R(), expected.R()).norm(), 1e-12);
    // twist：刚体上 p_i 点的速度
    EXPECT_LT((ee[i].twist - shiftTwistRefPoint(ref.twist, ref.pose.p, expected.p)).norm(), 1e-12);
    // 加速度：a_i = a_o + α×(p_i−p_o) + ω×(ω×(p_i−p_o))
    const Vector3d d = expected.p - ref.pose.p;
    const Vector3d w = ref.twist.tail<3>();
    const Vector3d a_i = ref.accel.head<3>() + ref.accel.tail<3>().cross(d) + w.cross(w.cross(d));
    EXPECT_LT((ee[i].accel.head<3>() - a_i).norm(), 1e-12);
    EXPECT_LT((ee[i].accel.tail<3>() - ref.accel.tail<3>()).norm(), 1e-12);
  }
}
