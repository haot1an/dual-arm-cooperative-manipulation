// 公式见 include/dual_arm/cbf_reference_filter.hpp 与 docs/cbf_reference_governor.md。
#include "dual_arm/cbf_reference_filter.hpp"

#include "dual_arm/small_qp.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

namespace dual_arm
{
  namespace
  {
    /// 名义路径速度低于该值时不施加分工约束、不逃逸（航点停顿、纯转动段）[m/s]
    constexpr double kMinPathSpeed = 1e-3;
  } // namespace

  CbfReferenceFilter::CbfReferenceFilter(const Config &config, double timestep)
      : config_(config), timestep_(timestep)
  {
    if (!(timestep_ > 0.0))
      throw std::invalid_argument("CbfReferenceFilter: timestep must be positive");
    if (config_.alpha * timestep_ >= 1.0)
      throw std::invalid_argument("CbfReferenceFilter: alpha * timestep must be < 1 (discrete CBF)");
    reset();
  }

  void CbfReferenceFilter::reset(double virtual_time)
  {
    virtual_time_ = virtual_time;
    offset_.setZero();
    decision_ = Decision{};
    previous_decision_ = Decision{};
    reference_ = ObjectReference{};
    status_ = Status::Solved;
    min_barrier_ = std::numeric_limits<double>::infinity();
    active_constraints_ = 0;
  }

  bool CbfReferenceFilter::engaged() const
  {
    return offset_.norm() > 1e-4 || decision_.time_rate < 0.999;
  }

  const ObjectReference &CbfReferenceFilter::update(const ObjectReference &nominal, const Pose &actual_object,
                                                    const Obstacles &obstacles, int count)
  {
    count = std::clamp(count, 0, kMaxObstacles);
    min_barrier_ = std::numeric_limits<double>::infinity();
    for (int j = 0; j < count; ++j)
      min_barrier_ = std::min(min_barrier_, obstacles[j].distance - config_.safe_distance);

    Decision decision = previous_decision_;
    active_constraints_ = 0;
    status_ = solveCbfQp(nominal, actual_object, obstacles, count, decision);
    if (status_ == Status::Infeasible || !decision.offset_rate.allFinite() || !std::isfinite(decision.time_rate))
    {
      // 无可用解：参考停在当前位置（δ̇ = 0, ṡ = 0），由下层 QP 的距离约束兜底。
      status_ = Status::Infeasible;
      decision.offset_rate.setZero();
      decision.time_rate = 0.0;
    }
    decision_ = decision;

    // 参考合成（δ_k, u_k）；加速度用 δ̇ 的差分，忽略 s̈ 项。
    const double s_dot = decision_.time_rate;
    reference_ = nominal;
    reference_.pose.p = nominal.pose.p + offset_;
    reference_.twist.head<3>() = s_dot * nominal.twist.head<3>() + decision_.offset_rate;
    reference_.twist.tail<3>() = s_dot * nominal.twist.tail<3>();
    reference_.accel.head<3>() = s_dot * s_dot * nominal.accel.head<3>() +
                                 (decision_.offset_rate - previous_decision_.offset_rate) / timestep_;
    reference_.accel.tail<3>() = s_dot * s_dot * nominal.accel.tail<3>();
    reference_.screw_rate = s_dot * nominal.screw_rate;
    reference_.screw_accel = s_dot * s_dot * nominal.screw_accel;

    // 积分到下一周期
    offset_ += decision_.offset_rate * timestep_;
    virtual_time_ += s_dot * timestep_;
    previous_decision_ = decision_;
    return reference_;
  }

  CbfReferenceFilter::Status CbfReferenceFilter::solveCbfQp(const ObjectReference &nominal,
                                                            const Pose &actual_object,
                                                            const Obstacles &obstacles, int count,
                                                            Decision &decision)
  {
    using small_qp::kMaxRows;
    using small_qp::kMaxVars;
    using XRow = Eigen::Matrix<double, 1, 4>;
    const Config &c = config_;
    const double dt = timestep_;
    const Vector3d v_nom = nominal.twist.head<3>();
    const Vector3d w_nom = nominal.twist.tail<3>();

    // ---- 名义偏移速度：回复项 + 逃逸项（文档 §5）----
    double min_h = std::numeric_limits<double>::infinity();
    Vector3d n_min = Vector3d::UnitX();
    for (int j = 0; j < count; ++j)
    {
      const double h = obstacles[j].distance - c.safe_distance;
      if (h < min_h)
      {
        min_h = h;
        n_min = obstacles[j].normal;
      }
    }
    double sigma = 0.0;
    Vector3d escape = Vector3d::Zero();
    if (count > 0 && c.escape_activation > 0.0 && v_nom.norm() > kMinPathSpeed)
    {
      // σ_h：离障碍越近越强；σ_a：只在参考正朝障碍运动时逃逸
      const double sigma_h = std::clamp(1.0 - min_h / c.escape_activation, 0.0, 1.0);
      const double sigma_a = std::clamp(n_min.dot(v_nom.normalized()), 0.0, 1.0);
      sigma = sigma_h * sigma_a;
      const Vector3d tangential = c.escape_direction - c.escape_direction.dot(n_min) * n_min;
      if (tangential.norm() > 1e-6)
        escape = c.escape_gain * tangential.normalized();
    }
    Eigen::Vector4d target_x;
    target_x << (1.0 - sigma) * (-c.return_gain * offset_) + sigma * escape, 1.0;

    // ---- 分工约束 v̂ᵀδ̇ = −k_r v̂ᵀδ：δ̇ = c_v v̂ + B z，决策变量 y = [z; ṡ]（文档 §4）----
    Eigen::Matrix<double, 4, kMaxVars> T = Eigen::Matrix<double, 4, kMaxVars>::Zero();
    Eigen::Vector4d x0 = Eigen::Vector4d::Zero();
    small_qp::VarVector weight = small_qp::VarVector::Ones();
    int n = 4;
    if (v_nom.norm() > kMinPathSpeed)
    {
      const Vector3d v_hat = v_nom.normalized();
      const Vector3d b1 = v_hat.unitOrthogonal();
      const Vector3d b2 = v_hat.cross(b1);
      const double along = std::clamp(-c.return_gain * v_hat.dot(offset_), -0.5 * c.offset_speed_max,
                                      0.5 * c.offset_speed_max);
      x0.head<3>() = along * v_hat;
      T.block<3, 1>(0, 0) = b1;
      T.block<3, 1>(0, 1) = b2;
      T(3, 2) = 1.0;
      n = 3;
      weight << c.offset_weight, c.offset_weight, c.time_rate_weight, 1.0;
    }
    else
    {
      T.topLeftCorner<4, 4>().setIdentity();
      weight << c.offset_weight, c.offset_weight, c.offset_weight, c.time_rate_weight;
    }
    // T 的列在 δ̇ 块内正交归一且与 x0 正交，故 y 空间代价仍是对角的，目标为 Tᵀ(t_x − x0)。
    const small_qp::VarVector target_y = T.transpose() * (target_x - x0);

    small_qp::RowMatrix A = small_qp::RowMatrix::Zero();
    small_qp::RowVector b = small_qp::RowVector::Zero();
    int m = 0;
    auto addRow = [&](const XRow &row, double bound)
    {
      if (m >= kMaxRows)
        return;
      A.row(m) = row * T;
      b[m] = bound - row.dot(x0.transpose());
      ++m;
    };

    // 分级松弛：0 = 全部约束；1 = 去掉变化率约束；2 = 再去掉偏移上限屏障。CBF 行从不松弛。
    for (int stage = 0; stage < 3; ++stage)
    {
      m = 0;
      for (int j = 0; j < count; ++j)
      {
        // n_jᵀ δ̇ + (n_jᵀ w_j) ṡ ≤ α h_j，w_j = v_nom + ω_nom × (p_j − p_o)（文档 §3）
        const CbfObstacle &o = obstacles[j];
        const Vector3d w_j = v_nom + w_nom.cross(o.point - actual_object.p);
        XRow row;
        row << o.normal.transpose(), o.normal.dot(w_j);
        addRow(row, c.alpha * (o.distance - c.safe_distance));
      }
      if (stage < 2)
      {
        XRow row;
        row << 2.0 * offset_.transpose(), 0.0;
        addRow(row, c.alpha * (c.offset_max * c.offset_max - offset_.squaredNorm()));
      }
      const bool rate_limited = stage == 0;
      for (int i = 0; i < 4; ++i)
      {
        const bool time = i == 3;
        const double previous = time ? previous_decision_.time_rate : previous_decision_.offset_rate[i];
        const double step = (time ? c.time_rate_accel_max : c.offset_accel_max) * dt;
        double upper = time ? 1.0 : c.offset_speed_max;
        double lower = time ? 0.0 : -c.offset_speed_max;
        if (rate_limited)
        {
          upper = std::min(upper, previous + step);
          lower = std::max(lower, previous - step);
        }
        XRow row = XRow::Zero();
        row[i] = 1.0;
        addRow(row, upper);
        addRow(-row, -lower);
      }

      small_qp::VarVector y;
      const small_qp::Result result = small_qp::solveDiagonalQp(A, b, m, n, weight, target_y, y);
      if (!result.feasible)
        continue;
      const Eigen::Vector4d x = T * y + x0;
      decision.offset_rate = x.head<3>();
      decision.time_rate = x[3];
      active_constraints_ = result.active;
      return stage == 0 ? Status::Solved : Status::Relaxed;
    }
    active_constraints_ = 0;
    return Status::Infeasible;
  }

} // namespace dual_arm
