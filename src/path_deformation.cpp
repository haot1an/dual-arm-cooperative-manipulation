// 公式见 include/dual_arm/path_deformation.hpp 与 docs/trajectory_planning.md §5–§6。
#include "dual_arm/path_deformation.hpp"

#include "dual_arm/object_trajectory.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace dual_arm
{

  PathDeformation::PathDeformation(std::vector<double> knot_times, std::vector<Vector3d> offsets)
      : knot_times_(std::move(knot_times)), offsets_(std::move(offsets))
  {
    const std::size_t n = knot_times_.size();
    if (n < 2 || offsets_.size() != n)
      throw std::invalid_argument("PathDeformation: need >= 2 knots and matching offsets");
    h_ = (knot_times_.back() - knot_times_.front()) / static_cast<double>(n - 1);
    if (!(h_ > 0.0))
      throw std::invalid_argument("PathDeformation: knot times must increase");
    for (std::size_t i = 1; i < n; ++i)
    {
      if (std::abs(knot_times_[i] - knot_times_[i - 1] - h_) > 1e-9 * std::max(1.0, h_))
        throw std::invalid_argument("PathDeformation: knots must be uniform");
    }

    second_derivative_ = splineSecondDerivatives(offsets_, h_);
  }

  std::vector<Vector3d> PathDeformation::splineSecondDerivatives(const std::vector<Vector3d> &values, double h)
  {
    // 两端一阶导为 0 的三次样条：解三对角方程得节点二阶导 M_i（Thomas 算法，三个分量一起算）
    const std::size_t n = values.size();
    std::vector<double> sub(n, h), diag(n, 4.0 * h), sup(n, h);
    std::vector<Vector3d> rhs(n);
    diag.front() = diag.back() = 2.0 * h;
    rhs.front() = 6.0 * (values[1] - values[0]) / h;
    rhs.back() = -6.0 * (values[n - 1] - values[n - 2]) / h;
    for (std::size_t i = 1; i + 1 < n; ++i)
      rhs[i] = 6.0 * (values[i + 1] - 2.0 * values[i] + values[i - 1]) / h;
    for (std::size_t i = 1; i < n; ++i)
    {
      const double w = sub[i] / diag[i - 1];
      diag[i] -= w * sup[i - 1];
      rhs[i] -= w * rhs[i - 1];
    }
    std::vector<Vector3d> second(n, Vector3d::Zero());
    second[n - 1] = rhs[n - 1] / diag[n - 1];
    for (std::size_t i = n - 1; i-- > 0;)
      second[i] = (rhs[i] - sup[i] * second[i + 1]) / diag[i];
    return second;
  }

  void PathDeformation::setRotationOffsets(std::vector<Vector3d> rotation_offsets)
  {
    if (rotation_offsets.size() != knot_times_.size())
      throw std::invalid_argument("PathDeformation: rotation offsets must match knots");
    rotation_offsets_ = std::move(rotation_offsets);
    rotation_second_derivative_ = splineSecondDerivatives(rotation_offsets_, h_);
  }

  void PathDeformation::setKnotExecutionTimes(std::vector<double> knot_execution_times)
  {
    const std::size_t n = knot_times_.size();
    if (knot_execution_times.size() != n)
      throw std::invalid_argument("PathDeformation: execution times must match knots");
    for (std::size_t i = 1; i < n; ++i)
      if (!(knot_execution_times[i] > knot_execution_times[i - 1]))
        throw std::invalid_argument("PathDeformation: execution times must increase");
    knot_exec_t_ = std::move(knot_execution_times);
    // 单调三次 Hermite（Fritsch–Carlson）：割线斜率全为正，节点斜率取相邻割线平均再做单调性修正
    std::vector<double> secant(n - 1);
    for (std::size_t i = 0; i + 1 < n; ++i)
      secant[i] = (knot_times_[i + 1] - knot_times_[i]) / (knot_exec_t_[i + 1] - knot_exec_t_[i]);
    knot_slope_.assign(n, 0.0);
    knot_slope_.front() = secant.front();
    knot_slope_.back() = secant.back();
    for (std::size_t i = 1; i + 1 < n; ++i)
      knot_slope_[i] = 0.5 * (secant[i - 1] + secant[i]);
    for (std::size_t i = 0; i + 1 < n; ++i)
    {
      const double a = knot_slope_[i] / secant[i], b = knot_slope_[i + 1] / secant[i];
      const double r = a * a + b * b;
      if (r > 9.0)
      {
        const double t = 3.0 / std::sqrt(r);
        knot_slope_[i] = t * a * secant[i];
        knot_slope_[i + 1] = t * b * secant[i];
      }
    }
    grid_tau_.clear();
    grid_t_.clear();
    grid_k_.clear();
  }

  void PathDeformation::evaluateSpline(const std::vector<Vector3d> &values, const std::vector<Vector3d> &second,
                                       double tau, Vector3d &v, Vector3d &v_prime, Vector3d &v_second) const
  {
    const double t0 = knot_times_.front();
    const double t1 = knot_times_.back();
    if (tau <= t0 || tau >= t1)
    {
      v = tau <= t0 ? values.front() : values.back();
      v_prime.setZero();
      v_second.setZero();
      return;
    }
    const std::size_t i = std::min(static_cast<std::size_t>((tau - t0) / h_), knot_times_.size() - 2);
    const double a = (knot_times_[i + 1] - tau) / h_;
    const double b = 1.0 - a;
    const Vector3d &y0 = values[i], &y1 = values[i + 1];
    const Vector3d &m0 = second[i], &m1 = second[i + 1];
    v = a * y0 + b * y1 + ((a * a * a - a) * m0 + (b * b * b - b) * m1) * (h_ * h_ / 6.0);
    v_prime = (y1 - y0) / h_ - (3.0 * a * a - 1.0) / 6.0 * h_ * m0 + (3.0 * b * b - 1.0) / 6.0 * h_ * m1;
    v_second = a * m0 + b * m1;
  }

  void PathDeformation::rotationOffset(double tau, Vector3d &phi, Vector3d &phi_prime, Vector3d &phi_second) const
  {
    if (rotation_offsets_.empty())
    {
      phi.setZero();
      phi_prime.setZero();
      phi_second.setZero();
      return;
    }
    evaluateSpline(rotation_offsets_, rotation_second_derivative_, tau, phi, phi_prime, phi_second);
  }

  void PathDeformation::offset(double tau, Vector3d &d, Vector3d &d_prime, Vector3d &d_second) const
  {
    evaluateSpline(offsets_, second_derivative_, tau, d, d_prime, d_second);
  }

  void PathDeformation::retime(const ObjectTrajectory &nominal, double max_speed, double max_accel,
                               double smoothing_window)
  {
    const double tau_end = knot_times_.back();
    grid_step_ = 0.005;
    const int n = static_cast<int>(std::ceil(tau_end / grid_step_)) + 1;
    grid_tau_.resize(n);
    std::vector<double> raw(n);
    for (int i = 0; i < n; ++i)
    {
      const double tau = std::min(i * grid_step_, tau_end);
      grid_tau_[i] = tau;
      const ObjectReference ref = nominal.evaluateNominal(tau);
      Vector3d d, dp, dpp;
      offset(tau, d, dp, dpp);
      const double speed = (ref.twist.head<3>() + dp).norm();
      const double accel = (ref.accel.head<3>() + dpp).norm();
      raw[i] = std::max({1.0, speed / max_speed, std::sqrt(accel / max_accel)});
    }
    // 滑动最大（窗口内任一点的需求都被覆盖）后滑动平均（使 k 连续）；两步之后 k ≥ 原值
    const int w = std::max(1, static_cast<int>(smoothing_window / grid_step_));
    std::vector<double> peak(n);
    for (int i = 0; i < n; ++i)
    {
      double m = 1.0;
      for (int j = std::max(0, i - w); j <= std::min(n - 1, i + w); ++j)
        m = std::max(m, raw[j]);
      peak[i] = m;
    }
    grid_k_.resize(n);
    for (int i = 0; i < n; ++i)
    {
      double sum = 0.0;
      int count = 0;
      for (int j = std::max(0, i - w); j <= std::min(n - 1, i + w); ++j)
      {
        sum += peak[j];
        ++count;
      }
      grid_k_[i] = sum / count;
    }
    grid_t_.resize(n);
    grid_t_[0] = grid_tau_[0];
    for (int i = 1; i < n; ++i)
      grid_t_[i] = grid_t_[i - 1] + 0.5 * (grid_k_[i] + grid_k_[i - 1]) * (grid_tau_[i] - grid_tau_[i - 1]);
  }

  void PathDeformation::timeMap(double t, double &tau, double &tau_dot, double &tau_ddot) const
  {
    tau_dot = 1.0;
    tau_ddot = 0.0;
    if (!knot_exec_t_.empty())
    {
      if (t <= knot_exec_t_.front())
      {
        tau = knot_times_.front() + (t - knot_exec_t_.front());
        return;
      }
      if (t >= knot_exec_t_.back())
      {
        tau = knot_times_.back() + (t - knot_exec_t_.back());
        return;
      }
      const auto it = std::upper_bound(knot_exec_t_.begin(), knot_exec_t_.end(), t);
      const std::size_t i = static_cast<std::size_t>(it - knot_exec_t_.begin()) - 1;
      const double hh = knot_exec_t_[i + 1] - knot_exec_t_[i];
      const double s = (t - knot_exec_t_[i]) / hh;
      const double y0 = knot_times_[i], y1 = knot_times_[i + 1];
      const double m0 = knot_slope_[i] * hh, m1 = knot_slope_[i + 1] * hh;
      tau = (2 * s * s * s - 3 * s * s + 1) * y0 + (s * s * s - 2 * s * s + s) * m0 + (-2 * s * s * s + 3 * s * s) * y1 +
            (s * s * s - s * s) * m1;
      tau_dot = ((6 * s * s - 6 * s) * y0 + (3 * s * s - 4 * s + 1) * m0 + (-6 * s * s + 6 * s) * y1 +
                 (3 * s * s - 2 * s) * m1) / hh;
      tau_ddot = ((12 * s - 6) * y0 + (6 * s - 4) * m0 + (-12 * s + 6) * y1 + (6 * s - 2) * m1) / (hh * hh);
      return;
    }
    if (grid_t_.empty() || t <= grid_t_.front())
    {
      tau = t;
      return;
    }
    if (t >= grid_t_.back())
    {
      tau = grid_tau_.back() + (t - grid_t_.back());
      return;
    }
    const auto it = std::upper_bound(grid_t_.begin(), grid_t_.end(), t);
    const std::size_t i = static_cast<std::size_t>(it - grid_t_.begin()) - 1;
    const double s = (t - grid_t_[i]) / (grid_t_[i + 1] - grid_t_[i]);
    tau = grid_tau_[i] + s * (grid_tau_[i + 1] - grid_tau_[i]);
    const double k = grid_k_[i] + s * (grid_k_[i + 1] - grid_k_[i]);
    const double dk = (grid_k_[i + 1] - grid_k_[i]) / (grid_tau_[i + 1] - grid_tau_[i]);
    tau_dot = 1.0 / k;
    tau_ddot = -dk / (k * k * k);
  }

  double PathDeformation::executionTime(double tau) const
  {
    if (!knot_exec_t_.empty())
    {
      // 报告用：节点之间线性插值（节点处精确）
      if (tau <= knot_times_.front())
        return knot_exec_t_.front() + (tau - knot_times_.front());
      if (tau >= knot_times_.back())
        return knot_exec_t_.back() + (tau - knot_times_.back());
      const std::size_t i = std::min(static_cast<std::size_t>((tau - knot_times_.front()) / h_), knot_times_.size() - 2);
      const double s = (tau - knot_times_[i]) / h_;
      return knot_exec_t_[i] + s * (knot_exec_t_[i + 1] - knot_exec_t_[i]);
    }
    if (grid_t_.empty() || tau <= grid_tau_.front())
      return tau;
    if (tau >= grid_tau_.back())
      return grid_t_.back() + (tau - grid_tau_.back());
    const auto it = std::upper_bound(grid_tau_.begin(), grid_tau_.end(), tau);
    const std::size_t i = static_cast<std::size_t>(it - grid_tau_.begin()) - 1;
    const double s = (tau - grid_tau_[i]) / (grid_tau_[i + 1] - grid_tau_[i]);
    return grid_t_[i] + s * (grid_t_[i + 1] - grid_t_[i]);
  }

  double PathDeformation::addedDuration() const
  {
    if (!knot_exec_t_.empty())
      return knot_exec_t_.back() - knot_times_.back();
    return grid_t_.empty() ? 0.0 : grid_t_.back() - grid_tau_.back();
  }

} // namespace dual_arm
