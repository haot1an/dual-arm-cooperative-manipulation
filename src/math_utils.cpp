#include "dual_arm/math_utils.hpp"

#include <Eigen/SVD>

#include <algorithm>
#include <cmath>

namespace dual_arm {

Matrix3d skew(const Vector3d& v) {
  Matrix3d S;
  S << 0.0, -v.z(), v.y(),
       v.z(), 0.0, -v.x(),
       -v.y(), v.x(), 0.0;
  return S;
}

Quaterniond quatFromRpyDeg(const Vector3d& rpy_deg) {
  const Vector3d r = rpy_deg * M_PI / 180.0;
  return (Eigen::AngleAxisd(r.z(), Vector3d::UnitZ()) *
          Eigen::AngleAxisd(r.y(), Vector3d::UnitY()) *
          Eigen::AngleAxisd(r.x(), Vector3d::UnitX()))
      .normalized();
}

Vector3d rotationError(const Matrix3d& R_des, const Matrix3d& R) {
  // 通过四元数取对数，数值上在 0 和 π 附近都稳定
  Quaterniond dq(R_des * R.transpose());
  dq.normalize();
  if (dq.w() < 0.0) dq.coeffs() *= -1.0;  // 取最短旋转
  const Vector3d v = dq.vec();
  const double s = v.norm();
  if (s < 1e-12) return 2.0 * v;  // 小角度：log ≈ 2·vec
  const double angle = 2.0 * std::atan2(s, dq.w());
  return v * (angle / s);
}

Vector6d poseError(const Pose& des, const Pose& cur) {
  Vector6d e;
  e.head<3>() = des.p - cur.p;
  e.tail<3>() = rotationError(des.R(), cur.R());
  return e;
}

Vector6d decomposePoseDrift(const Pose& T0, const Pose& T, const Matrix3d& R_f) {
  Vector6d out;
  out.head<3>() = R_f.transpose() * (T.p - T0.p);
  out.tail<3>() = R_f.transpose() * rotationError(T.R(), T0.R());
  return out;
}

JacobianMetrics jacobianMetrics(const Matrix6x7d& J) {
  JacobianMetrics out;
  const Eigen::JacobiSVD<Matrix6x7d> svd(J);
  const Vector6d s = svd.singularValues();
  out.sigma_min = s.minCoeff();
  out.cond = s.maxCoeff() / std::max(out.sigma_min, 1e-300);
  out.manipulability = s.prod();
  const Eigen::Matrix<double, 3, 7> Jv = J.topRows<3>();
  const Eigen::JacobiSVD<Eigen::Matrix<double, 3, 7>> svd_v(Jv);
  const Eigen::Vector3d sv = svd_v.singularValues();
  out.sigma_min_linear = sv.minCoeff();
  out.cond_linear = sv.maxCoeff() / std::max(out.sigma_min_linear, 1e-300);
  return out;
}

Wrench shiftWrenchRefPoint(const Wrench& w_at_a, const Vector3d& a, const Vector3d& b) {
  Wrench out;
  out.head<3>() = w_at_a.head<3>();
  out.tail<3>() = w_at_a.tail<3>() + (a - b).cross(w_at_a.head<3>());
  return out;
}

Twist shiftTwistRefPoint(const Twist& v_at_a, const Vector3d& a, const Vector3d& b) {
  Twist out;
  out.tail<3>() = v_at_a.tail<3>();
  out.head<3>() = v_at_a.head<3>() + v_at_a.tail<3>().cross(b - a);
  return out;
}

}  // namespace dual_arm
