#include "dual_arm/math_utils.hpp"

#include <gtest/gtest.h>

#include <random>

using namespace dual_arm;

TEST(MathUtils, SkewIsCrossProduct) {
  const Vector3d a(0.3, -1.2, 2.0), b(-0.7, 0.4, 1.1);
  EXPECT_LT((skew(a) * b - a.cross(b)).norm(), 1e-15);
  EXPECT_LT((skew(a) + skew(a).transpose()).norm(), 1e-15);
}

TEST(MathUtils, RpyConvention) {
  // yaw 90°：x 轴 → y 轴
  const Quaterniond q = quatFromRpyDeg(Vector3d(0, 0, 90));
  EXPECT_LT((q * Vector3d::UnitX() - Vector3d::UnitY()).norm(), 1e-12);
  // R = Rz Ry Rx
  const Vector3d rpy(10, -20, 30);
  const Matrix3d R = (Eigen::AngleAxisd(30 * M_PI / 180, Vector3d::UnitZ()) *
                      Eigen::AngleAxisd(-20 * M_PI / 180, Vector3d::UnitY()) *
                      Eigen::AngleAxisd(10 * M_PI / 180, Vector3d::UnitX()))
                         .toRotationMatrix();
  EXPECT_LT((quatFromRpyDeg(rpy).toRotationMatrix() - R).norm(), 1e-12);
}

TEST(MathUtils, RotationErrorIsWorldFrameRotationVector) {
  const Matrix3d R = quatFromRpyDeg(Vector3d(5, 10, -40)).toRotationMatrix();
  const Vector3d w(0.1, -0.3, 0.2);
  const Matrix3d R_des = Eigen::AngleAxisd(w.norm(), w.normalized()).toRotationMatrix() * R;
  EXPECT_LT((rotationError(R_des, R) - w).norm(), 1e-12);
  // 接近 π 也稳定
  const Vector3d w_big = Vector3d(0, 0, 1) * (M_PI - 1e-6);
  const Matrix3d R2 = Eigen::AngleAxisd(w_big.norm(), w_big.normalized()).toRotationMatrix() * R;
  EXPECT_LT((rotationError(R2, R) - w_big).norm(), 1e-6);
}

TEST(MathUtils, WrenchAndTwistShiftPreservePower) {
  std::mt19937 rng(0);
  std::uniform_real_distribution<double> u(-1, 1);
  for (int k = 0; k < 20; ++k) {
    const Vector3d a(u(rng), u(rng), u(rng)), b(u(rng), u(rng), u(rng));
    Wrench h;
    Twist v;
    for (int i = 0; i < 6; ++i) {
      h[i] = u(rng);
      v[i] = u(rng);
    }
    // 功率 hᵀν 与参考点无关
    EXPECT_NEAR(shiftWrenchRefPoint(h, a, b).dot(shiftTwistRefPoint(v, a, b)), h.dot(v), 1e-12);
    // 平移再平移回来得到原值
    EXPECT_LT((shiftWrenchRefPoint(shiftWrenchRefPoint(h, a, b), b, a) - h).norm(), 1e-12);
  }
}

TEST(MathUtils, PoseComposeInverse) {
  Pose T;
  T.p = Vector3d(0.1, -0.2, 0.3);
  T.q = quatFromRpyDeg(Vector3d(10, 20, 30));
  const Pose I = T * T.inverse();
  EXPECT_LT(I.p.norm(), 1e-12);
  EXPECT_LT(rotationError(I.R(), Matrix3d::Identity()).norm(), 1e-12);
  const Vector3d x(0.5, 0.4, -0.1);
  EXPECT_LT((T.inverse().transformPoint(T.transformPoint(x)) - x).norm(), 1e-12);
}
