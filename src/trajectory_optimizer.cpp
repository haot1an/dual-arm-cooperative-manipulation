// 公式见 include/dual_arm/trajectory_optimizer.hpp 与 docs/trajectory_planning.md §10。
#include "dual_arm/trajectory_optimizer.hpp"

#include "dual_arm/closed_chain_ik.hpp"
#include "dual_arm/math_utils.hpp"
#include "dual_arm/path_deformation.hpp"
#include "dual_arm/sparse_qp.hpp"

#include <Eigen/SparseCore>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <stdexcept>

namespace dual_arm
{
  namespace
  {
    constexpr double kPenetratingGroup = -0.002; ///< 名义最小距离低于它的障碍组必须被修正 [m]
    constexpr double kSuccessTolerance = 5e-4;   ///< 约束允许的残余违反（米 / 弧度 / 米每秒 量级）
    using Triplet = Eigen::Triplet<double>;

    /// SO(3) 左雅可比 J_l(φ)：Exp(φ + dφ) ≈ Exp(J_l dφ) Exp(φ)
    Matrix3d leftJacobian(const Vector3d &phi)
    {
      const double angle = phi.norm();
      if (angle < 1e-9)
        return Matrix3d::Identity();
      const Matrix3d K = skew(phi);
      return Matrix3d::Identity() + (1.0 - std::cos(angle)) / (angle * angle) * K +
             (angle - std::sin(angle)) / (angle * angle * angle) * K * K;
    }

    Quaterniond expQuat(const Vector3d &phi)
    {
      const double angle = phi.norm();
      if (angle < 1e-12)
        return Quaterniond::Identity();
      return Quaterniond(Eigen::AngleAxisd(angle, phi / angle));
    }
  } // namespace

  ObjectTrajectoryOptimizer::ObjectTrajectoryOptimizer(const PlannerConfig &config, double default_safe_distance,
                                                       const CollisionConfig &collision_config,
                                                       std::shared_ptr<RobotModel> model,
                                                       std::shared_ptr<const ObjectTrajectory> nominal)
      : config_(config), default_safe_distance_(default_safe_distance), model_(std::move(model)),
        nominal_(std::move(nominal))
  {
    if (!model_ || !nominal_)
      throw std::invalid_argument("ObjectTrajectoryOptimizer: model and nominal trajectory are required");
    const SceneSpec &scene = model_->scene();
    if (scene.obstacles.empty())
      throw std::invalid_argument("ObjectTrajectoryOptimizer: scene has no obstacle pairs");
    double max_target = 0.0;
    for (int g = 0; g < static_cast<int>(scene.obstacles.size()); ++g)
      max_target = std::max(max_target, targetDistance(g));
    CollisionConfig cc = collision_config;
    cc.margin = max_target + config_.activation_distance + 0.02;
    if (!config_.model_error_body.empty())
      cc.body_offsets.emplace_back(config_.model_error_body, config_.model_error_offset);
    collision_ = std::make_unique<CollisionModel>(
        scene, std::array<Pose, kNumArms>{model_->basePose(Arm::Left), model_->basePose(Arm::Right)}, cc);

    for (int a = 0; a < 3; ++a)
      if (config_.rotation_axes[a] > 0.5)
        axes_.push_back(Vector3d::Unit(a));

    // 与 ObjectPathPlanner 相同的均匀节点（便于用其结果作初值）
    const double T = nominal_->nominalEndTime();
    const int N = std::max(2, static_cast<int>(std::ceil(T / config_.knot_dt)));
    n_knots_ = N + 1;
    h_ = T / N;
    // 最后一段（插槽）运动从 last_start 开始：之后的节点固定为名义值
    double last_start = T - config_.pin_end;
    if (!nominal_->segments().empty())
      last_start = std::min(last_start, nominal_->segments().back().t0);
    tau_.resize(n_knots_);
    reference_.resize(n_knots_);
    pinned_.assign(n_knots_, true);
    pos_index_.assign(n_knots_, -1);
    rot_index_.assign(n_knots_, -1);
    for (int k = 0; k < n_knots_; ++k)
    {
      tau_[k] = k * h_;
      reference_[k] = nominal_->evaluateNominal(tau_[k]);
      pinned_[k] = k == 0 || k == N || tau_[k] <= config_.pin_start + 1e-9 || tau_[k] >= last_start - 1e-9;
      if (pinned_[k])
        continue;
      pos_index_[k] = num_vars_;
      num_vars_ += 3;
      if (!axes_.empty())
      {
        rot_index_[k] = num_vars_;
        num_vars_ += static_cast<int>(axes_.size());
      }
    }
    dt_index_.assign(N, -1);
    for (int i = 0; i < N; ++i)
      if (!(pinned_[i] && pinned_[i + 1]))
        dt_index_[i] = num_vars_++;
    if (num_vars_ == 0)
      throw std::invalid_argument("ObjectTrajectoryOptimizer: no free knots");
  }

  double ObjectTrajectoryOptimizer::targetDistance(int group) const
  {
    const double safe = model_->scene().obstacles[group].safe_distance;
    return (safe >= 0.0 ? safe : default_safe_distance_) + config_.safety_margin;
  }

  Vector3d ObjectTrajectoryOptimizer::position(int k, const Eigen::VectorXd &x) const
  {
    return pos_index_[k] < 0 ? reference_[k].pose.p : Vector3d(x.segment<3>(pos_index_[k]));
  }

  Vector3d ObjectTrajectoryOptimizer::rotationOffset(int k, const Eigen::VectorXd &x) const
  {
    Vector3d phi = Vector3d::Zero();
    if (rot_index_[k] >= 0)
      for (std::size_t j = 0; j < axes_.size(); ++j)
        phi += axes_[j] * x[rot_index_[k] + static_cast<int>(j)];
    return phi;
  }

  double ObjectTrajectoryOptimizer::intervalTime(int i, const Eigen::VectorXd &x) const
  {
    return dt_index_[i] < 0 ? h_ : x[dt_index_[i]];
  }

  Pose ObjectTrajectoryOptimizer::knotPose(int k, const Eigen::VectorXd &x) const
  {
    Pose pose;
    pose.p = position(k, x);
    pose.q = (expQuat(rotationOffset(k, x)) * reference_[k].pose.q).normalized();
    return pose;
  }

  void ObjectTrajectoryOptimizer::evaluateKnots(const Eigen::VectorXd &x, std::vector<KnotEval> &knots) const
  {
    knots.resize(n_knots_);
    const SceneSpec &scene = model_->scene();
    std::array<Vector7d, kNumArms> seed{scene.q_init[0], scene.q_init[1]};
    for (int k = 0; k < n_knots_; ++k)
    {
      KnotEval &kn = knots[k];
      const Pose object = knotPose(k, x);
      kn.q = seed;
      kn.ik_ok = solveClosedChainIk(*model_, object, kn.q);
      seed = kn.q;
      DualArmState state;
      for (Arm a : kArms)
      {
        state.arm(a).q = kn.q[armIndex(a)];
        state.arm(a).dq.setZero();
      }
      model_->update(state);
      for (Arm a : kArms)
        kn.dq_dxi[armIndex(a)] = jointRateFromObjectTwist(*model_, a, object.p);
      kn.contacts.clear();
      for (const DistanceInfo &info : collision_->query(kn.q[0], kn.q[1]))
      {
        Contact c;
        c.pair = info.pair;
        c.group = info.group;
        c.distance = info.distance;
        c.gradient = (info.jacobian.head<7>() * kn.dq_dxi[0] + info.jacobian.tail<7>() * kn.dq_dxi[1]).transpose();
        kn.contacts.push_back(c);
      }
    }
  }

  void ObjectTrajectoryOptimizer::addKnotGradient(int k, const Eigen::VectorXd &x, const Vector6d &d_dxi, double scale,
                                                  std::vector<std::pair<int, double>> &out) const
  {
    if (pos_index_[k] >= 0)
      for (int j = 0; j < 3; ++j)
        out.emplace_back(pos_index_[k] + j, scale * d_dxi[j]);
    if (rot_index_[k] >= 0)
    {
      // ξ 的转动部分 = J_l(φ) dφ，dφ = Σ_j axis_j dz_j
      const Vector3d w = leftJacobian(rotationOffset(k, x)).transpose() * d_dxi.tail<3>();
      for (std::size_t j = 0; j < axes_.size(); ++j)
        out.emplace_back(rot_index_[k] + static_cast<int>(j), scale * w.dot(axes_[j]));
    }
  }

  void ObjectTrajectoryOptimizer::buildRows(const Eigen::VectorXd &x, const std::vector<KnotEval> &knots,
                                            std::vector<Row> &rows) const
  {
    rows.clear();
    const int N = n_knots_ - 1;

    // ---- 碰撞与关节余量（自由节点）----
    for (int k = 0; k < n_knots_; ++k)
    {
      if (pinned_[k])
        continue;
      const KnotEval &kn = knots[k];
      if (!kn.ik_ok)
      {
        Row row; // IK 不可达：不可线性化的大违反
        row.g = 1.0;
        row.linearize = false;
        rows.push_back(row);
      }
      for (const Contact &c : kn.contacts)
      {
        const double stored = required_[k][c.pair];
        const double r = std::isnan(stored) ? targetDistance(c.group) : stored;
        Row row;
        row.g = r - c.distance;
        row.linearize = c.distance < r + config_.activation_distance;
        addKnotGradient(k, x, c.gradient, -1.0, row.gradient);
        rows.push_back(std::move(row));
      }
      for (Arm a : kArms)
      {
        const Vector7d lo = model_->jointLowerLimit(a), hi = model_->jointUpperLimit(a);
        const Vector7d &q = kn.q[armIndex(a)];
        for (int j = 0; j < kArmDof; ++j)
        {
          const Vector6d dq = kn.dq_dxi[armIndex(a)].row(j).transpose();
          for (int side = 0; side < 2; ++side)
          {
            const double g = side == 0 ? (lo[j] + config_.joint_margin) - q[j] : q[j] - (hi[j] - config_.joint_margin);
            if (g < -(config_.joint_activation - config_.joint_margin))
              continue;
            Row row;
            row.g = g;
            addKnotGradient(k, x, dq, side == 0 ? -1.0 : 1.0, row.gradient);
            rows.push_back(std::move(row));
          }
        }
      }
    }

    // ---- 平移速度：‖p_{i+1} − p_i‖ − v_max Δt_i ≤ 0 ----
    for (int i = 0; i < N; ++i)
    {
      if (pos_index_[i] < 0 && pos_index_[i + 1] < 0 && dt_index_[i] < 0)
        continue;
      const Vector3d dp = position(i + 1, x) - position(i, x);
      const double len = dp.norm();
      Row row;
      row.g = len - config_.max_speed * intervalTime(i, x);
      if (len > 1e-9)
      {
        const Vector3d u = dp / len;
        for (int j = 0; j < 3; ++j)
        {
          if (pos_index_[i + 1] >= 0)
            row.gradient.emplace_back(pos_index_[i + 1] + j, u[j]);
          if (pos_index_[i] >= 0)
            row.gradient.emplace_back(pos_index_[i] + j, -u[j]);
        }
      }
      if (dt_index_[i] >= 0)
        row.gradient.emplace_back(dt_index_[i], -config_.max_speed);
      rows.push_back(std::move(row));
    }

    // ---- 平移加速度：a_k = (Δp_k/Δt_k − Δp_{k−1}/Δt_{k−1}) / S，S = (Δt_{k−1} + Δt_k)/2 ----
    for (int k = 1; k < N; ++k)
    {
      const bool involved = pos_index_[k - 1] >= 0 || pos_index_[k] >= 0 || pos_index_[k + 1] >= 0 ||
                            dt_index_[k - 1] >= 0 || dt_index_[k] >= 0;
      if (!involved)
        continue;
      const double t0 = intervalTime(k - 1, x), t1 = intervalTime(k, x);
      const double S = 0.5 * (t0 + t1);
      const Vector3d d0 = position(k, x) - position(k - 1, x);
      const Vector3d d1 = position(k + 1, x) - position(k, x);
      const Vector3d a = (d1 / t1 - d0 / t0) / S;
      const double norm = a.norm();
      Row row;
      row.g = norm - config_.max_accel;
      if (norm > 1e-9)
      {
        const Vector3d u = a / norm;
        auto addPos = [&](int knot, double coefficient)
        {
          if (pos_index_[knot] >= 0)
            for (int j = 0; j < 3; ++j)
              row.gradient.emplace_back(pos_index_[knot] + j, coefficient * u[j]);
        };
        addPos(k + 1, 1.0 / (t1 * S));
        addPos(k, -1.0 / (t1 * S) - 1.0 / (t0 * S));
        addPos(k - 1, 1.0 / (t0 * S));
        if (dt_index_[k] >= 0)
          row.gradient.emplace_back(dt_index_[k], u.dot(-d1 / (t1 * t1 * S) - a / (2.0 * S)));
        if (dt_index_[k - 1] >= 0)
          row.gradient.emplace_back(dt_index_[k - 1], u.dot(d0 / (t0 * t0 * S) - a / (2.0 * S)));
      }
      row.linearize = row.g > -0.5 * config_.max_accel;
      rows.push_back(std::move(row));
    }

    // ---- 角速度：‖Log(R_{i+1} R_iᵀ)‖ − ω_max Δt_i ≤ 0（dρ ≈ J_l(φ_{i+1}) dφ_{i+1} − J_l(φ_i) dφ_i）----
    for (int i = 0; i < N; ++i)
    {
      if (rot_index_[i] < 0 && rot_index_[i + 1] < 0 && dt_index_[i] < 0)
        continue;
      const Eigen::AngleAxisd rel(knotPose(i + 1, x).q * knotPose(i, x).q.conjugate());
      const double angle = std::abs(rel.angle());
      Row row;
      row.g = angle - config_.max_angular_speed * intervalTime(i, x);
      if (angle > 1e-9)
      {
        const Vector3d u = rel.angle() >= 0.0 ? Vector3d(rel.axis()) : Vector3d(-rel.axis());
        auto addRot = [&](int knot, double sign)
        {
          if (rot_index_[knot] < 0)
            return;
          const Vector3d w = leftJacobian(rotationOffset(knot, x)).transpose() * u;
          for (std::size_t j = 0; j < axes_.size(); ++j)
            row.gradient.emplace_back(rot_index_[knot] + static_cast<int>(j), sign * w.dot(axes_[j]));
        };
        addRot(i + 1, 1.0);
        addRot(i, -1.0);
      }
      if (dt_index_[i] >= 0)
        row.gradient.emplace_back(dt_index_[i], -config_.max_angular_speed);
      row.linearize = row.g > -0.5 * config_.max_angular_speed * h_;
      rows.push_back(std::move(row));
    }
  }

  bool ObjectTrajectoryOptimizer::lateRotationStart(const Eigen::VectorXd &x_nominal, Eigen::VectorXd &x0) const
  {
    int first = -1, last = -1;
    for (int k = 0; k < n_knots_; ++k)
      if (!pinned_[k])
      {
        first = first < 0 ? k : first;
        last = k;
      }
    if (first <= 0 || last + 1 >= n_knots_)
      return false;
    const Quaterniond Ra = reference_[first - 1].pose.q, Rb = reference_[last + 1].pose.q;
    const Eigen::AngleAxisd rel(Rb * Ra.conjugate());
    const Vector3d rho = rel.angle() * rel.axis();
    if (rho.norm() < 1e-3)
      return false;
    Vector3d projected = Vector3d::Zero();
    for (const Vector3d &axis : axes_)
      projected += axis * axis.dot(rho);
    if ((rho - projected).norm() > 1e-3)
      return false; // 所需转动不在允许转轴上
    // 名义轨迹在窗口内真正在转的时长
    double rotating = 0.0;
    for (int k = first; k <= last + 1; ++k)
      if (reference_[k].twist.tail<3>().norm() > 1e-6)
        rotating += h_;
    rotating = std::max(rotating, 2.0 * h_);
    const double end = tau_[last + 1];
    x0 = x_nominal;
    for (int k = first; k <= last; ++k)
    {
      const double u = std::clamp((tau_[k] - (end - rotating)) / rotating, 0.0, 1.0);
      const double s = u * u * (3.0 - 2.0 * u);
      const Quaterniond R = (expQuat(s * rho) * Ra).normalized();
      const Eigen::AngleAxisd offset(R * reference_[k].pose.q.conjugate());
      const Vector3d phi = offset.angle() * offset.axis();
      for (std::size_t j = 0; j < axes_.size(); ++j)
      {
        const double value = axes_[j].dot(phi);
        if (std::abs(value) > config_.max_rotation)
          return false;
        x0[rot_index_[k] + static_cast<int>(j)] = value;
      }
    }
    return true;
  }

  double ObjectTrajectoryOptimizer::violation(const std::vector<Row> &rows, double *worst)
  {
    double sum = 0.0, w = -std::numeric_limits<double>::infinity();
    for (const Row &row : rows)
    {
      sum += std::max(0.0, row.g);
      w = std::max(w, row.g);
    }
    if (worst)
      *worst = rows.empty() ? 0.0 : w;
    return sum;
  }

  PlanResult ObjectTrajectoryOptimizer::optimize(const PlanResult *warm_start)
  {
    const auto t_start = std::chrono::steady_clock::now();
    PlanResult result;
    result.formulation = "full";
    const int N = n_knots_ - 1;
    const int n = num_vars_;

    // ---- 名义初值与要求距离（“不比名义轨迹更差”，与 ObjectPathPlanner 相同）----
    Eigen::VectorXd x_nominal = Eigen::VectorXd::Zero(n);
    for (int k = 0; k < n_knots_; ++k)
      if (pos_index_[k] >= 0)
        x_nominal.segment<3>(pos_index_[k]) = reference_[k].pose.p;
    for (int i = 0; i < N; ++i)
      if (dt_index_[i] >= 0)
        x_nominal[dt_index_[i]] = h_;
    std::vector<KnotEval> nominal_knots;
    evaluateKnots(x_nominal, nominal_knots);
    const int num_groups = collision_->numGroups();
    std::vector<double> nominal_min(num_groups, std::numeric_limits<double>::infinity());
    for (const KnotEval &kn : nominal_knots)
      for (const Contact &c : kn.contacts)
        nominal_min[c.group] = std::min(nominal_min[c.group], c.distance);
    required_.assign(n_knots_, std::vector<double>(collision_->numPairs(), std::numeric_limits<double>::quiet_NaN()));
    for (int k = 0; k < n_knots_; ++k)
      for (const Contact &c : nominal_knots[k].contacts)
      {
        const double target = targetDistance(c.group);
        required_[k][c.pair] = nominal_min[c.group] < kPenetratingGroup ? target : std::min(target, c.distance);
      }

    // ---- 固定二次代价 ½xᵀPx + qᵀx ----
    std::vector<Triplet> sp, sr, st;
    Eigen::VectorXd cp = Eigen::VectorXd::Zero(3 * n_knots_), ct = Eigen::VectorXd::Zero(N);
    for (int k = 0; k < n_knots_; ++k)
    {
      for (int j = 0; j < 3; ++j)
      {
        if (pos_index_[k] >= 0)
          sp.emplace_back(3 * k + j, pos_index_[k] + j, 1.0);
        else
          cp[3 * k + j] = reference_[k].pose.p[j];
      }
      if (rot_index_[k] >= 0)
        for (std::size_t a = 0; a < axes_.size(); ++a)
          for (int j = 0; j < 3; ++j)
            if (axes_[a][j] != 0.0)
              sr.emplace_back(3 * k + j, rot_index_[k] + static_cast<int>(a), axes_[a][j]);
    }
    for (int i = 0; i < N; ++i)
    {
      if (dt_index_[i] >= 0)
        st.emplace_back(i, dt_index_[i], 1.0);
      else
        ct[i] = h_;
    }
    Eigen::SparseMatrix<double> Sp(3 * n_knots_, n), Sr(3 * n_knots_, n), St(N, n);
    Sp.setFromTriplets(sp.begin(), sp.end());
    Sr.setFromTriplets(sr.begin(), sr.end());
    St.setFromTriplets(st.begin(), st.end());
    std::vector<Triplet> d2, d1;
    for (int k = 1; k < N; ++k)
      for (int j = 0; j < 3; ++j)
      {
        d2.emplace_back(3 * (k - 1) + j, 3 * (k - 1) + j, 1.0 / (h_ * h_));
        d2.emplace_back(3 * (k - 1) + j, 3 * k + j, -2.0 / (h_ * h_));
        d2.emplace_back(3 * (k - 1) + j, 3 * (k + 1) + j, 1.0 / (h_ * h_));
      }
    for (int i = 0; i + 1 < N; ++i)
    {
      d1.emplace_back(i, i, -1.0);
      d1.emplace_back(i, i + 1, 1.0);
    }
    Eigen::SparseMatrix<double> D2(3 * (N - 1), 3 * n_knots_), D1(N - 1, N);
    D2.setFromTriplets(d2.begin(), d2.end());
    D1.setFromTriplets(d1.begin(), d1.end());
    const double wa = config_.accel_weight * h_;
    const double wr = config_.accel_weight * config_.rotation_length * config_.rotation_length * h_;
    const Eigen::SparseMatrix<double> DpS = D2 * Sp, DrS = D2 * Sr, D1S = D1 * St;
    Eigen::SparseMatrix<double> P = 2.0 * wa * DpS.transpose() * DpS + 2.0 * wr * DrS.transpose() * DrS +
                                    2.0 * config_.rotation_offset_weight * h_ * Sr.transpose() * Sr +
                                    2.0 * config_.time_smooth_weight * D1S.transpose() * D1S;
    Eigen::VectorXd q = 2.0 * wa * DpS.transpose() * (D2 * cp) +
                        2.0 * config_.time_smooth_weight * D1S.transpose() * (D1 * ct) +
                        config_.time_weight * St.transpose() * Eigen::VectorXd::Ones(N);
    auto cost = [&](const Eigen::VectorXd &x) { return 0.5 * x.dot(P * x) + q.dot(x); };

    // ---- 初值候选 ----
    //   lateral：只平移规划的结果（下压穿梁）；late-rotation：保持起始姿态穿过障碍，在插槽段前才完成滚转。
    std::vector<std::pair<std::string, Eigen::VectorXd>> starts;
    {
      Eigen::VectorXd x0 = x_nominal;
      std::string label = "nominal";
      if (warm_start && static_cast<int>(warm_start->offsets.size()) == n_knots_)
      {
        for (int k = 0; k < n_knots_; ++k)
          if (pos_index_[k] >= 0)
            x0.segment<3>(pos_index_[k]) += warm_start->offsets[k];
        label = "lateral:" + warm_start->initialization;
      }
      starts.emplace_back(label, x0);
    }
    if (!axes_.empty())
    {
      Eigen::VectorXd x0;
      if (lateRotationStart(x_nominal, x0))
        starts.emplace_back("late-rotation", x0);
    }

    struct Attempt
    {
      Eigen::VectorXd x;
      std::vector<KnotEval> knots;
      std::vector<Row> rows;
      double mu = 0.0, cost = 0.0, worst = 0.0;
      bool ik_ok = true;
      std::string label;
    };
    const double h_min = config_.min_dt_ratio * h_, h_max = config_.max_dt_ratio * h_;
    sparse_qp::Settings qp_settings;
    auto runScp = [&](Eigen::VectorXd x, const std::string &label) -> Attempt
    {
      std::vector<KnotEval> knots;
      evaluateKnots(x, knots);
      std::vector<Row> rows;
      buildRows(x, knots, rows);

      // ---- 序列凸规划 ----
      double mu = config_.penalty;
      double delta = config_.trust_region;
      double merit_current = cost(x) + mu * violation(rows);
      auto raisePenaltyOrStop = [&]() -> bool
      {
        double worst = 0.0;
        violation(rows, &worst);
        if (worst <= kSuccessTolerance)
          return true;
        mu *= 10.0;
        delta = config_.trust_region;
        merit_current = cost(x) + mu * violation(rows);
        return mu > config_.max_penalty;
      };

      for (int iteration = 0; iteration < config_.full_max_iterations; ++iteration)
      {
        std::vector<int> active;
        double constant_violation = 0.0;
        for (int r = 0; r < static_cast<int>(rows.size()); ++r)
        {
          if (rows[r].linearize && !rows[r].gradient.empty())
            active.push_back(r);
          else
            constant_violation += std::max(0.0, rows[r].g);
        }
        const int ns = static_cast<int>(active.size());
        const int nx = n + ns;
        std::vector<Triplet> p_triplets;
        for (int j = 0; j < P.outerSize(); ++j)
          for (Eigen::SparseMatrix<double>::InnerIterator it(P, j); it; ++it)
            p_triplets.emplace_back(it.row(), it.col(), it.value());
        for (int s = 0; s < ns; ++s)
          p_triplets.emplace_back(n + s, n + s, 1e-8);
        Eigen::SparseMatrix<double> Pqp(nx, nx);
        Pqp.setFromTriplets(p_triplets.begin(), p_triplets.end());
        Eigen::VectorXd qqp = Eigen::VectorXd::Zero(nx);
        qqp.head(n) = q;
        qqp.tail(ns).setConstant(mu);

        const int m = 2 * ns + n;
        std::vector<Triplet> a_triplets;
        Eigen::VectorXd lower(m), upper(m);
        const double inf = std::numeric_limits<double>::infinity();
        for (int s = 0; s < ns; ++s)
        {
          const Row &row = rows[active[s]];
          double rhs = -row.g; // ∇gᵀx − s ≤ ∇gᵀx̄ − g
          for (const auto &[index, value] : row.gradient)
          {
            a_triplets.emplace_back(s, index, value);
            rhs += value * x[index];
          }
          a_triplets.emplace_back(s, n + s, -1.0);
          lower[s] = -inf;
          upper[s] = rhs;
          a_triplets.emplace_back(ns + s, n + s, 1.0);
          lower[ns + s] = 0.0;
          upper[ns + s] = inf;
        }
        // 盒约束：信赖域 ∩ 物理上下限（位置相对名义 ≤ max_offset、|φ| ≤ max_rotation、Δt ∈ [h_min, h_max]）
        Eigen::VectorXd box_lo(n), box_hi(n);
        const double dt_trust = h_ * std::min(0.5, delta / 0.02 * 0.5);
        for (int k = 0; k < n_knots_; ++k)
        {
          if (pos_index_[k] >= 0)
            for (int j = 0; j < 3; ++j)
            {
              const int v = pos_index_[k] + j;
              box_lo[v] = std::max(x[v] - delta, reference_[k].pose.p[j] - config_.max_offset);
              box_hi[v] = std::min(x[v] + delta, reference_[k].pose.p[j] + config_.max_offset);
            }
          if (rot_index_[k] >= 0)
            for (std::size_t a = 0; a < axes_.size(); ++a)
            {
              const int v = rot_index_[k] + static_cast<int>(a);
              box_lo[v] = std::max(x[v] - delta / config_.rotation_length, -config_.max_rotation);
              box_hi[v] = std::min(x[v] + delta / config_.rotation_length, config_.max_rotation);
            }
        }
        for (int i = 0; i < N; ++i)
          if (dt_index_[i] >= 0)
          {
            const int v = dt_index_[i];
            box_lo[v] = std::max(x[v] - dt_trust, h_min);
            box_hi[v] = std::min(x[v] + dt_trust, h_max);
          }
        for (int v = 0; v < n; ++v)
        {
          a_triplets.emplace_back(2 * ns + v, v, 1.0);
          lower[2 * ns + v] = std::min(box_lo[v], box_hi[v]);
          upper[2 * ns + v] = std::max(box_lo[v], box_hi[v]);
        }
        Eigen::SparseMatrix<double> A(m, nx);
        A.setFromTriplets(a_triplets.begin(), a_triplets.end());
        Eigen::VectorXd xs(nx);
        xs.head(n) = x;
        xs.tail(ns).setZero();
        sparse_qp::solve(Pqp, qqp, A, lower, upper, xs, qp_settings);
        ++result.iterations;
        Eigen::VectorXd x_new = xs.head(n);
        for (int v = 0; v < n; ++v)
          x_new[v] = std::clamp(x_new[v], lower[2 * ns + v], upper[2 * ns + v]);

        double model_violation = constant_violation;
        for (int s = 0; s < ns; ++s)
        {
          const Row &row = rows[active[s]];
          double g_lin = row.g;
          for (const auto &[index, value] : row.gradient)
            g_lin += value * (x_new[index] - x[index]);
          model_violation += std::max(0.0, g_lin);
        }
        const double predicted = merit_current - (cost(x_new) + mu * model_violation);
        if (predicted <= 1e-9 * std::max(1.0, std::abs(merit_current)))
        {
          if (raisePenaltyOrStop())
            break;
          continue;
        }
        std::vector<KnotEval> knots_new;
        std::vector<Row> rows_new;
        evaluateKnots(x_new, knots_new);
        buildRows(x_new, knots_new, rows_new);
        const double merit_new = cost(x_new) + mu * violation(rows_new);
        const double ratio = (merit_current - merit_new) / predicted;
        if (ratio > 0.1)
        {
          const double step = (x_new - x).cwiseAbs().maxCoeff();
          x = x_new;
          knots = std::move(knots_new);
          rows = std::move(rows_new);
          merit_current = merit_new;
          ++result.accepted_steps;
          if (ratio > 0.75)
            delta = std::min(2.0 * delta, config_.max_offset);
          if (step < 1e-6 && raisePenaltyOrStop())
            break;
        }
        else
        {
          delta *= 0.5;
          if (delta < config_.min_trust_region && raisePenaltyOrStop())
            break;
        }
      }
      Attempt attempt;
      violation(rows, &attempt.worst);
      attempt.cost = cost(x);
      attempt.mu = mu;
      for (const KnotEval &kn : knots)
        attempt.ik_ok &= kn.ik_ok;
      attempt.x = std::move(x);
      attempt.knots = std::move(knots);
      attempt.rows = std::move(rows);
      attempt.label = label;
      return attempt;
    };
    auto feasible = [](const Attempt &a) { return a.ik_ok && a.worst <= kSuccessTolerance; };
    Attempt best;
    bool have_best = false;
    for (const auto &[label, x0] : starts)
    {
      Attempt attempt = runScp(x0, label);
      ++result.starts;
      const bool better = !have_best || (feasible(attempt) != feasible(best)
                                             ? feasible(attempt)
                                             : (feasible(attempt) ? attempt.cost < best.cost : attempt.worst < best.worst));
      if (better)
      {
        best = std::move(attempt);
        have_best = true;
      }
    }
    const Eigen::VectorXd &x = best.x;
    const std::vector<KnotEval> &knots = best.knots;
    const std::vector<Row> &rows = best.rows;
    const double mu = best.mu;
    result.initialization = best.label;

    // ---- 结果 ----
    double worst = 0.0;
    violation(rows, &worst);
    bool ik_ok = true;
    for (const KnotEval &kn : knots)
      ik_ok &= kn.ik_ok;
    result.worst_violation = worst;
    result.final_penalty = mu;
    result.success = ik_ok && worst <= kSuccessTolerance;
    result.message = !ik_ok ? "closed-chain IK failed at some knot"
                     : result.success ? "ok"
                                      : "constraints still violated (local minimum or penalty limit)";
    result.knot_times = tau_;
    result.offsets.resize(n_knots_);
    result.rotation_offsets.resize(n_knots_);
    result.knot_min_margin.assign(n_knots_, std::numeric_limits<double>::infinity());
    std::vector<double> exec_times(n_knots_, 0.0);
    for (int k = 0; k < n_knots_; ++k)
    {
      result.offsets[k] = position(k, x) - reference_[k].pose.p;
      result.rotation_offsets[k] = rotationOffset(k, x);
      result.max_offset = std::max(result.max_offset, result.offsets[k].norm());
      result.max_rotation = std::max(result.max_rotation, result.rotation_offsets[k].norm());
      for (const Contact &c : knots[k].contacts)
      {
        const double stored = required_[k][c.pair];
        const double r = std::isnan(stored) ? targetDistance(c.group) : stored;
        result.knot_min_margin[k] = std::min(result.knot_min_margin[k], c.distance - r);
      }
    }
    exec_times[0] = tau_[0];
    for (int k = 1; k < n_knots_; ++k)
      exec_times[k] = exec_times[k - 1] + intervalTime(k - 1, x);
    result.deformation = std::make_shared<PathDeformation>(result.knot_times, result.offsets);
    result.deformation->setRotationOffsets(result.rotation_offsets);
    result.deformation->setKnotExecutionTimes(exec_times);
    result.added_duration = result.deformation->addedDuration();
    result.planning_time_s = std::chrono::duration<double>(std::chrono::steady_clock::now() - t_start).count();
    return result;
  }

} // namespace dual_arm
