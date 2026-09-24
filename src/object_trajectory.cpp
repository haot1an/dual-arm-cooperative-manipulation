#include "dual_arm/object_trajectory.hpp"

#include "dual_arm/math_utils.hpp"
#include "dual_arm/path_deformation.hpp"

#include <algorithm>
#include <cmath>

namespace dual_arm {

// ---------------------------------------------------------------------------
// 梯形速度剖面
// ---------------------------------------------------------------------------
TrapezoidProfile TrapezoidProfile::make(double L, double v_max, double a_max) {
  TrapezoidProfile p;
  p.L = std::max(L, 0.0);
  p.a = a_max;
  if (p.L <= 0.0) return p;
  if (v_max * v_max / a_max >= p.L) {  // 三角形：到不了 v_max
    p.v = std::sqrt(p.L * a_max);
    p.t_acc = p.v / a_max;
    p.t_flat = 0.0;
  } else {
    p.v = v_max;
    p.t_acc = v_max / a_max;
    p.t_flat = (p.L - v_max * p.t_acc) / v_max;
  }
  p.T = 2.0 * p.t_acc + p.t_flat;
  return p;
}

void TrapezoidProfile::sample(double t, double& s, double& ds, double& dds) const {
  if (L <= 0.0 || T <= 0.0 || t <= 0.0) {
    s = 0.0;
    ds = dds = 0.0;
    return;
  }
  if (t >= T) {
    s = L;
    ds = dds = 0.0;
    return;
  }
  if (t < t_acc) {
    s = 0.5 * a * t * t;
    ds = a * t;
    dds = a;
  } else if (t < t_acc + t_flat) {
    s = 0.5 * a * t_acc * t_acc + v * (t - t_acc);
    ds = v;
    dds = 0.0;
  } else {
    const double r = T - t;
    s = L - 0.5 * a * r * r;
    ds = a * r;
    dds = -a;
  }
}

// ---------------------------------------------------------------------------
// 物体轨迹
// ---------------------------------------------------------------------------
ObjectTrajectory::ObjectTrajectory(const ObjectTrajectoryConfig& cfg, const Pose& initial_pose,
                                   const std::vector<Waypoint>& waypoints)
    : cfg_(cfg), p0_(initial_pose) {
  t_end_ = cfg_.waypoints.t_start;
  if (cfg_.type != ObjectTrajectoryConfig::Type::Waypoints || waypoints.size() < 2) return;
  const auto& w = cfg_.waypoints;
  psi_init_ = waypoints.front().has_screw_angle ? waypoints.front().screw_angle : 0.0;
  Pose cur = p0_;  // 第一个航点用实际初始位姿（与 reset 后的物体一致）
  double psi = psi_init_;
  double t = w.t_start;
  // 螺钉角速度 / 角加速度上限沿用转动上限 w_max / alpha_max
  for (std::size_t k = 1; k < waypoints.size(); ++k) {
    Segment s;
    s.from = waypoints[k - 1].name;
    s.to = waypoints[k].name;
    s.t0 = t;
    s.start = cur;
    s.dp = waypoints[k].pose.p - cur.p;
    const Eigen::AngleAxisd aa((waypoints[k].pose.q * cur.q.conjugate()).normalized());
    s.angle = aa.angle();
    s.axis = s.angle > 1e-12 ? Vector3d(aa.axis()) : Vector3d::UnitZ();
    if (s.angle > M_PI) {  // 取短弧
      s.angle = 2.0 * M_PI - s.angle;
      s.axis = -s.axis;
    }
    s.psi0 = psi;
    s.dpsi = waypoints[k].has_screw_angle ? waypoints[k].screw_angle - psi : 0.0;
    // 三个量（平移距离、转角、螺钉角）由同一个归一化参数 u ∈ [0, 1] 驱动：量 X 的速度 = L_X·u̇。
    // 取 u̇ 上限 = min_X v_max,X / L_X、ü 上限 = min_X a_max,X / L_X，三者同步开始、同步结束且都不超限。
    double v_u = 1e9, a_u = 1e9;
    const double lens[3] = {s.dp.norm(), s.angle, std::abs(s.dpsi)};
    const double vmax[3] = {w.v_max, w.w_max, w.w_max};
    const double amax[3] = {w.a_max, w.alpha_max, w.alpha_max};
    for (int i = 0; i < 3; ++i) {
      if (lens[i] < 1e-12) continue;
      v_u = std::min(v_u, vmax[i] / lens[i]);
      a_u = std::min(a_u, amax[i] / lens[i]);
    }
    if (v_u >= 1e9) {  // 与上一航点完全相同：只停留
      v_u = a_u = 1.0;
      s.prof = TrapezoidProfile{};
    } else {
      s.prof = TrapezoidProfile::make(1.0, v_u, a_u);
    }
    const double T_seg = s.prof.T;
    s.T = T_seg;
    segs_.push_back(s);
    t += T_seg + w.dwell;
    cur = waypoints[k].pose;
    psi += s.dpsi;
  }
  t_end_ = t;
}

double ObjectTrajectory::endTime() const {
  return deformation_ ? deformation_->executionTime(t_end_) : t_end_;
}

ObjectReference ObjectTrajectory::evaluate(double t) const {
  if (!deformation_) return evaluateNominal(t);
  // 变形 + 重新计时（公式见 path_deformation.hpp）
  double tau, tau_dot, tau_ddot;
  deformation_->timeMap(t, tau, tau_dot, tau_ddot);
  ObjectReference ref = evaluateNominal(tau);
  Vector3d d, d_prime, d_second;
  deformation_->offset(tau, d, d_prime, d_second);
  const Vector3d v_path = ref.twist.head<3>() + d_prime;
  const Vector3d w_path = ref.twist.tail<3>();
  const Vector3d alpha_nom = ref.accel.tail<3>();
  ref.pose.p += d;
  ref.accel.head<3>() = tau_dot * tau_dot * (ref.accel.head<3>() + d_second) + tau_ddot * v_path;
  ref.accel.tail<3>() = tau_dot * tau_dot * ref.accel.tail<3>() + tau_ddot * w_path;
  ref.twist.head<3>() = tau_dot * v_path;
  ref.twist.tail<3>() = tau_dot * w_path;
  if (deformation_->hasRotation()) {
    // 姿态偏移（世界系左乘）：R = Exp(φ) R_nom，ω = τ̇ (J_l(φ) φ' + Exp(φ) ω_nom)；
    // α ≈ τ̈ (·) + τ̇² (J_l φ'' + [J_l φ']× Exp(φ) ω_nom + Exp(φ) α_nom)（忽略 dJ_l/dτ，属二阶小量）
    Vector3d phi, phi_prime, phi_second;
    deformation_->rotationOffset(tau, phi, phi_prime, phi_second);
    const double angle = phi.norm();
    Matrix3d Rphi = Matrix3d::Identity();
    Matrix3d Jl = Matrix3d::Identity();
    if (angle > 1e-12) {
      const Vector3d axis = phi / angle;
      Rphi = Eigen::AngleAxisd(angle, axis).toRotationMatrix();
      const Matrix3d K = skew(phi);
      Jl += (1.0 - std::cos(angle)) / (angle * angle) * K + (angle - std::sin(angle)) / (angle * angle * angle) * K * K;
    }
    const Vector3d w_offset = Jl * phi_prime;
    const Vector3d w_total = w_offset + Rphi * w_path;  // w_path：名义角速度（对 τ）
    ref.pose.q = (Quaterniond(Rphi) * ref.pose.q).normalized();
    ref.twist.tail<3>() = tau_dot * w_total;
    ref.accel.tail<3>() = tau_ddot * w_total +
                          tau_dot * tau_dot * (Jl * phi_second + w_offset.cross(Rphi * w_path) + Rphi * alpha_nom);
  }
  ref.screw_accel = tau_dot * tau_dot * ref.screw_accel + tau_ddot * ref.screw_rate;
  ref.screw_rate *= tau_dot;
  return ref;
}

ObjectReference ObjectTrajectory::evaluateNominal(double t) const {
  ObjectReference ref;
  ref.pose = p0_;

  switch (cfg_.type) {
    case ObjectTrajectoryConfig::Type::Hold:
      break;

    case ObjectTrajectoryConfig::Type::Waypoints: {
      ref.screw_angle = psi_init_;
      if (segs_.empty() || t < segs_.front().t0) break;
      // 找到 t 所在的段（段数很少，线性查找）
      std::size_t k = 0;
      while (k + 1 < segs_.size() && t >= segs_[k + 1].t0) ++k;
      const Segment& s = segs_[k];
      double u, du, ddu;
      s.prof.sample(t - s.t0, u, du, ddu);
      ref.segment = static_cast<int>(k);
      ref.pose.p = s.start.p + u * s.dp;
      ref.pose.q = (Quaterniond(Eigen::AngleAxisd(u * s.angle, s.axis)) * s.start.q).normalized();
      ref.twist << du * s.dp, (du * s.angle) * s.axis;
      ref.accel << ddu * s.dp, (ddu * s.angle) * s.axis;
      ref.screw_angle = s.psi0 + u * s.dpsi;
      ref.screw_rate = du * s.dpsi;
      ref.screw_accel = ddu * s.dpsi;
      break;
    }

    case ObjectTrajectoryConfig::Type::MinJerk: {
      // s(τ) = 10τ³ − 15τ⁴ + 6τ⁵，τ = (t − t0)/T ∈ [0, 1]
      const auto& c = cfg_.min_jerk;
      const double T = std::max(c.t_end - c.t_start, 1e-6);
      const double tau = std::clamp((t - c.t_start) / T, 0.0, 1.0);
      const double s = tau * tau * tau * (10.0 - 15.0 * tau + 6.0 * tau * tau);
      const bool moving = (t > c.t_start && t < c.t_end);
      const double ds = moving ? 30.0 * tau * tau * (1.0 - tau) * (1.0 - tau) / T : 0.0;
      const double dds = moving ? 60.0 * tau * (1.0 - tau) * (1.0 - 2.0 * tau) / (T * T) : 0.0;

      // 姿态偏移用旋转向量 φ（世界系）表示：R(t) = Exp(s·φ) R0，ω = ds·φ，α = dds·φ
      const Eigen::AngleAxisd aa(quatFromRpyDeg(c.offset_rpy_deg));
      const Vector3d phi = aa.axis() * aa.angle();
      ref.pose.p = p0_.p + s * c.offset_xyz;
      ref.pose.q = (Quaterniond(Eigen::AngleAxisd(s * aa.angle(), aa.axis())) * p0_.q).normalized();
      ref.twist << ds * c.offset_xyz, ds * phi;
      ref.accel << dds * c.offset_xyz, dds * phi;
      break;
    }

    case ObjectTrajectoryConfig::Type::Sine: {
      const auto& c = cfg_.sine;
      if (t <= c.t_start) break;
      const double w = 2.0 * M_PI * c.frequency;
      const double ph = w * (t - c.t_start);
      const Vector3d axis = c.axis.norm() > 1e-9 ? Vector3d(c.axis.normalized()) : Vector3d::UnitZ();
      const double A = c.amplitude_deg * M_PI / 180.0;
      ref.pose.p = p0_.p + c.amplitude_xyz * std::sin(ph);
      ref.pose.q = (Quaterniond(Eigen::AngleAxisd(A * std::sin(ph), axis)) * p0_.q).normalized();
      ref.twist << c.amplitude_xyz * (w * std::cos(ph)), axis * (A * w * std::cos(ph));
      ref.accel << c.amplitude_xyz * (-w * w * std::sin(ph)), axis * (-A * w * w * std::sin(ph));
      break;
    }
  }
  return ref;
}

}  // namespace dual_arm
