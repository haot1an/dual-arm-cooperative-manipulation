// 公式见 include/dual_arm/trajectory_planner.hpp 与 docs/trajectory_planning.md。
#include "dual_arm/trajectory_planner.hpp"

#include "dual_arm/closed_chain_ik.hpp"
#include "dual_arm/math_utils.hpp"
#include "dual_arm/sparse_qp.hpp"

#include <Eigen/Cholesky>
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
    constexpr double kMovingSpeed = 1e-3;       ///< 名义速度高于它的节点只允许侧向偏移 [m/s]
    constexpr double kPenetratingGroup = -0.002; ///< 名义最小距离低于它的障碍组必须被修正 [m]
    constexpr double kSuccessTolerance = 5e-4;   ///< 节点处允许的距离不足 [m]
    using Triplet = Eigen::Triplet<double>;
  } // namespace

  ObjectPathPlanner::ObjectPathPlanner(const PlannerConfig &config, double default_safe_distance,
                                       const CollisionConfig &collision_config, std::shared_ptr<RobotModel> model,
                                       std::shared_ptr<const ObjectTrajectory> nominal)
      : config_(config), default_safe_distance_(default_safe_distance), model_(std::move(model)),
        nominal_(std::move(nominal))
  {
    if (!model_ || !nominal_)
      throw std::invalid_argument("ObjectPathPlanner: model and nominal trajectory are required");
    const SceneSpec &scene = model_->scene();
    if (scene.obstacles.empty())
      throw std::invalid_argument("ObjectPathPlanner: scene has no obstacle pairs");

    double max_target = 0.0;
    for (int g = 0; g < static_cast<int>(scene.obstacles.size()); ++g)
      max_target = std::max(max_target, targetDistance(g));
    // 查询范围只取需要的（约 0.09 m），不继承更大的全局 margin（见 CollisionConfig::margin 的说明）
    CollisionConfig cc = collision_config;
    cc.margin = max_target + config_.activation_distance + 0.02;
    if (!config_.model_error_body.empty())
      cc.body_offsets.emplace_back(config_.model_error_body, config_.model_error_offset);
    collision_ = std::make_unique<CollisionModel>(
        scene, std::array<Pose, kNumArms>{model_->basePose(Arm::Left), model_->basePose(Arm::Right)}, cc);

    // 均匀节点
    const double T = nominal_->nominalEndTime();
    const int N = std::max(2, static_cast<int>(std::ceil(T / config_.knot_dt)));
    const double h = T / N;
    tau_.resize(N + 1);
    reference_.resize(N + 1);
    var_index_.assign(N + 1, -1);
    var_dim_.assign(N + 1, 0);
    basis_.assign(N + 1, Eigen::Matrix3d::Identity());
    for (int k = 0; k <= N; ++k)
    {
      tau_[k] = k * h;
      reference_[k] = nominal_->evaluateNominal(tau_[k]);
      const bool free = k > 0 && k < N && tau_[k] > config_.pin_start + 1e-9 && tau_[k] < T - config_.pin_end - 1e-9;
      if (!free)
        continue;
      const Vector3d v = reference_[k].twist.head<3>();
      if (v.norm() > kMovingSpeed)
      {
        // 运动中：δ 只沿垂直于名义速度的两个方向（沿路径的进度交给时间参数化）
        const Vector3d v_hat = v.normalized();
        const Vector3d b1 = v_hat.unitOrthogonal();
        basis_[k].col(0) = b1;
        basis_[k].col(1) = v_hat.cross(b1);
        basis_[k].col(2).setZero();
        var_dim_[k] = 2;
      }
      else
      {
        var_dim_[k] = 3;
      }
      var_index_[k] = num_vars_;
      num_vars_ += var_dim_[k];
    }
    if (num_vars_ == 0)
      throw std::invalid_argument("ObjectPathPlanner: no free knots (check pin_start / pin_end)");
  }

  double ObjectPathPlanner::targetDistance(int group) const
  {
    const double safe = model_->scene().obstacles[group].safe_distance;
    return (safe >= 0.0 ? safe : default_safe_distance_) + config_.safety_margin;
  }

  Vector3d ObjectPathPlanner::offsetAt(int knot, const Eigen::VectorXd &z) const
  {
    if (var_index_[knot] < 0)
      return Vector3d::Zero();
    Vector3d d = Vector3d::Zero();
    for (int c = 0; c < var_dim_[knot]; ++c)
      d += basis_[knot].col(c) * z[var_index_[knot] + c];
    return d;
  }

  bool ObjectPathPlanner::solveArms(const Pose &object, std::array<Vector7d, kNumArms> &q) const
  {
    return solveClosedChainIk(*model_, object, q);
  }

  void ObjectPathPlanner::evaluate(const Eigen::VectorXd &z, std::vector<KnotState> &states) const
  {
    const int n = static_cast<int>(tau_.size());
    states.resize(n);
    const SceneSpec &scene = model_->scene();
    std::array<Vector7d, kNumArms> seed{scene.q_init[0], scene.q_init[1]};
    for (int k = 0; k < n; ++k)
    {
      KnotState &st = states[k];
      Pose object = reference_[k].pose;
      object.p += offsetAt(k, z);
      st.q = seed; // 顺序热启动：上一节点的解
      st.ik_ok = solveArms(object, st.q);
      seed = st.q;
      st.contacts.clear();

      // 物体平移 δ 时两臂的关节增量：dq_i = J_i⁺ [δ; 0]（与 IK 同一阻尼伪逆）
      DualArmState state;
      for (Arm a : kArms)
      {
        state.arm(a).q = st.q[armIndex(a)];
        state.arm(a).dq.setZero();
      }
      model_->update(state);
      std::array<Eigen::Matrix<double, 7, 3>, kNumArms> dq_ddelta;
      for (Arm a : kArms)
        dq_ddelta[armIndex(a)] = jointRateFromObjectTwist(*model_, a, object.p).leftCols<3>();
      for (const DistanceInfo &info : collision_->query(st.q[0], st.q[1]))
      {
        Contact c;
        c.pair = info.pair;
        c.group = info.group;
        c.distance = info.distance;
        c.gradient = (info.jacobian.head<7>() * dq_ddelta[0] + info.jacobian.tail<7>() * dq_ddelta[1]).transpose();
        st.contacts.push_back(c);
      }
    }
  }

  double ObjectPathPlanner::smoothCost(const Eigen::VectorXd &z) const
  {
    const int n = static_cast<int>(tau_.size());
    const double h = tau_[1] - tau_[0];
    double cost = 0.0;
    for (int k = 0; k < n; ++k)
    {
      const Vector3d d = offsetAt(k, z);
      cost += config_.offset_weight * h * d.squaredNorm();
      if (k > 0 && k + 1 < n)
        cost += config_.accel_weight * h *
                ((offsetAt(k + 1, z) - 2.0 * d + offsetAt(k - 1, z)) / (h * h)).squaredNorm();
    }
    return cost;
  }

  double ObjectPathPlanner::violation(const std::vector<KnotState> &states, double *worst) const
  {
    double sum = 0.0;
    double w = -std::numeric_limits<double>::infinity();
    for (int k = 0; k < static_cast<int>(states.size()); ++k)
    {
      if (var_index_[k] < 0)
        continue;
      if (!states[k].ik_ok)
      {
        sum += 1.0; // IK 不可达：视为大违反（1 m）
        w = std::max(w, 1.0);
      }
      for (const Contact &c : states[k].contacts)
      {
        const double stored = required_[k][c.pair];
        const double gap = (std::isnan(stored) ? targetDistance(c.group) : stored) - c.distance;
        sum += std::max(0.0, gap);
        w = std::max(w, gap);
      }
    }
    if (worst)
      *worst = w;
    return sum;
  }

  PlanResult ObjectPathPlanner::plan()
  {
    const auto t0 = std::chrono::steady_clock::now();
    PlanResult result;
    const int n = static_cast<int>(tau_.size());
    const double h = tau_[1] - tau_[0];
    const int num_groups = collision_->numGroups();

    const Eigen::VectorXd z_nominal = Eigen::VectorXd::Zero(num_vars_);
    std::vector<KnotState> nominal_states;
    evaluate(z_nominal, nominal_states);

    // 要求距离：有穿透的组取目标值；其余“不比名义轨迹更差”
    std::vector<double> nominal_min(num_groups, std::numeric_limits<double>::infinity());
    for (const KnotState &st : nominal_states)
      for (const Contact &c : st.contacts)
        nominal_min[c.group] = std::min(nominal_min[c.group], c.distance);
    required_.assign(n, std::vector<double>(collision_->numPairs(), 0.0));
    for (int k = 0; k < n; ++k)
    {
      for (int p = 0; p < collision_->numPairs(); ++p)
        required_[k][p] = std::numeric_limits<double>::quiet_NaN();
      for (const Contact &c : nominal_states[k].contacts)
      {
        const double target = targetDistance(c.group);
        required_[k][c.pair] = nominal_min[c.group] < kPenetratingGroup ? target : std::min(target, c.distance);
      }
    }
    // 名义轨迹上不在查询范围内的 pair（NaN）：首次出现时取目标值
    auto requiredDistance = [&](int k, const Contact &c)
    {
      double &r = required_[k][c.pair];
      if (std::isnan(r))
        r = targetDistance(c.group);
      return r;
    };
    for (int k = 0; k < n; ++k)
      for (const Contact &c : nominal_states[k].contacts)
        requiredDistance(k, c);

    // 固定的二次代价：P_δ = 2 w_a h D2ᵀD2 + 2 w_o h I，P_z = Mᵀ P_δ M（M：z → 全部节点 δ）
    std::vector<Triplet> m_triplets;
    for (int k = 0; k < n; ++k)
      for (int c = 0; c < var_dim_[k]; ++c)
        for (int r = 0; r < 3; ++r)
          m_triplets.emplace_back(3 * k + r, var_index_[k] + c, basis_[k](r, c));
    Eigen::SparseMatrix<double> M(3 * n, num_vars_);
    M.setFromTriplets(m_triplets.begin(), m_triplets.end());
    std::vector<Triplet> d_triplets;
    for (int k = 1; k + 1 < n; ++k)
      for (int r = 0; r < 3; ++r)
      {
        d_triplets.emplace_back(3 * (k - 1) + r, 3 * (k - 1) + r, 1.0 / (h * h));
        d_triplets.emplace_back(3 * (k - 1) + r, 3 * k + r, -2.0 / (h * h));
        d_triplets.emplace_back(3 * (k - 1) + r, 3 * (k + 1) + r, 1.0 / (h * h));
      }
    Eigen::SparseMatrix<double> D2(3 * (n - 2), 3 * n);
    D2.setFromTriplets(d_triplets.begin(), d_triplets.end());
    Eigen::SparseMatrix<double> I3n(3 * n, 3 * n);
    I3n.setIdentity();
    const Eigen::SparseMatrix<double> P_delta =
        2.0 * config_.accel_weight * h * (D2.transpose() * D2) + 2.0 * config_.offset_weight * h * I3n;
    const Eigen::SparseMatrix<double> P_z = M.transpose() * P_delta * M;

    // 一次序列凸规划：从初值 z（及其评估 states）出发，返回收敛结果
    struct Attempt
    {
      Eigen::VectorXd z;
      std::vector<KnotState> states;
      double mu = 0.0, worst = 0.0, cost = 0.0;
      bool ik_ok = true;
      std::string label;
    };
    sparse_qp::Settings qp_settings;
    auto runScp = [&](Eigen::VectorXd z, std::vector<KnotState> states, const std::string &label) -> Attempt
    {
        double mu = config_.penalty;
      double delta = config_.trust_region;
      auto merit = [&](const Eigen::VectorXd &zz, const std::vector<KnotState> &ss)
      { return smoothCost(zz) + mu * violation(ss); };
      double merit_current = merit(z, states);

      for (int iteration = 0; iteration < config_.max_iterations; ++iteration)
      {
        // ---- 线性化约束行：d̄ + gᵀB(z − z̄) + s ≥ r，只取 d < r + activation 的 pair ----
        struct Row
        {
          int knot;
          double required, distance;
          Eigen::Vector3d g_z; // gᵀ B（前 var_dim 项有效）
        };
        std::vector<Row> rows;
        for (int k = 0; k < n; ++k)
        {
          if (var_index_[k] < 0)
            continue;
          for (const Contact &c : states[k].contacts)
          {
            const double r = requiredDistance(k, c);
            if (c.distance >= r + config_.activation_distance)
              continue;
            Row row{k, r, c.distance, Eigen::Vector3d::Zero()};
            for (int col = 0; col < var_dim_[k]; ++col)
              row.g_z[col] = c.gradient.dot(basis_[k].col(col));
            rows.push_back(row);
          }
        }
        const int ns = static_cast<int>(rows.size());
        const int nx = num_vars_ + ns;

        std::vector<Triplet> p_triplets;
        for (int j = 0; j < P_z.outerSize(); ++j)
          for (Eigen::SparseMatrix<double>::InnerIterator it(P_z, j); it; ++it)
            p_triplets.emplace_back(it.row(), it.col(), it.value());
        for (int s = 0; s < ns; ++s)
          p_triplets.emplace_back(num_vars_ + s, num_vars_ + s, 1e-8);
        Eigen::SparseMatrix<double> P(nx, nx);
        P.setFromTriplets(p_triplets.begin(), p_triplets.end());
        Eigen::VectorXd q = Eigen::VectorXd::Zero(nx);
        q.tail(ns).setConstant(mu);

        const int m = ns + ns + num_vars_;
        std::vector<Triplet> a_triplets;
        Eigen::VectorXd lower(m), upper(m);
        const double inf = std::numeric_limits<double>::infinity();
        for (int s = 0; s < ns; ++s)
        {
          const Row &row = rows[s];
          const int base = var_index_[row.knot];
          double rhs = row.required - row.distance;
          for (int col = 0; col < var_dim_[row.knot]; ++col)
          {
            a_triplets.emplace_back(s, base + col, row.g_z[col]);
            rhs += row.g_z[col] * z[base + col];
          }
          a_triplets.emplace_back(s, num_vars_ + s, 1.0);
          lower[s] = rhs;
          upper[s] = inf;
          a_triplets.emplace_back(ns + s, num_vars_ + s, 1.0); // s ≥ 0
          lower[ns + s] = 0.0;
          upper[ns + s] = inf;
        }
        for (int i = 0; i < num_vars_; ++i) // 信赖域 ∩ 偏移上限
        {
          a_triplets.emplace_back(2 * ns + i, i, 1.0);
          lower[2 * ns + i] = std::max(z[i] - delta, -config_.max_offset);
          upper[2 * ns + i] = std::min(z[i] + delta, config_.max_offset);
        }
        Eigen::SparseMatrix<double> A(m, nx);
        A.setFromTriplets(a_triplets.begin(), a_triplets.end());

        Eigen::VectorXd x(nx);
        x.head(num_vars_) = z;
        x.tail(ns).setZero();
        sparse_qp::solve(P, q, A, lower, upper, x, qp_settings);
        ++result.iterations;
        Eigen::VectorXd z_new = x.head(num_vars_);
        for (int i = 0; i < num_vars_; ++i) // ADMM 解可能略出界：投影回信赖域
          z_new[i] = std::clamp(z_new[i], lower[2 * ns + i], upper[2 * ns + i]);

        // 模型（线性化）merit
        double model_violation = 0.0;
        for (const Row &row : rows)
        {
          double d_lin = row.distance;
          for (int col = 0; col < var_dim_[row.knot]; ++col)
            d_lin += row.g_z[col] * (z_new[var_index_[row.knot] + col] - z[var_index_[row.knot] + col]);
          model_violation += std::max(0.0, row.required - d_lin);
        }
        const double model_merit = smoothCost(z_new) + mu * model_violation;
        const double predicted = merit_current - model_merit;

        auto raisePenaltyOrStop = [&]() -> bool
        {
          double worst = 0.0;
          violation(states, &worst);
          if (worst <= kSuccessTolerance)
            return true; // 收敛且满足约束
          mu *= 10.0;
          delta = config_.trust_region;
          merit_current = merit(z, states);
          return mu > config_.max_penalty;
        };

        if (predicted <= 1e-9 * std::max(1.0, merit_current))
        {
          if (raisePenaltyOrStop())
            break;
          continue;
        }
        std::vector<KnotState> candidate;
        evaluate(z_new, candidate);
        const double merit_new = merit(z_new, candidate);
        const double ratio = (merit_current - merit_new) / predicted;
        if (ratio > 0.1)
        {
          const double step = (z_new - z).cwiseAbs().maxCoeff();
          z = z_new;
          states = std::move(candidate);
          merit_current = merit_new;
          ++result.accepted_steps;
          if (ratio > 0.75)
            delta = std::min(2.0 * delta, config_.max_offset);
          if (step < 1e-5)
          {
            if (raisePenaltyOrStop())
              break;
          }
        }
        else
        {
          delta *= 0.5;
          if (delta < config_.min_trust_region && raisePenaltyOrStop())
            break;
        }
      }

      Attempt attempt;
      violation(states, &attempt.worst);
      attempt.cost = smoothCost(z);
      attempt.mu = mu;
      for (const KnotState &st : states)
        attempt.ik_ok &= st.ik_ok;
      attempt.z = std::move(z);
      attempt.states = std::move(states);
      attempt.label = label;
      return attempt;
    };
    auto better = [](const Attempt &a, const Attempt &b)
    {
      const bool fa = a.ik_ok && a.worst <= kSuccessTolerance, fb = b.ik_ok && b.worst <= kSuccessTolerance;
      if (fa != fb)
        return fa;
      return fa ? a.cost < b.cost : a.worst < b.worst;
    };

    Attempt best = runScp(z_nominal, nominal_states, "nominal");
    ++result.starts;
    const bool nominal_ok = best.ik_ok && best.worst <= kSuccessTolerance;
    if (!nominal_ok && config_.multi_start)
    {
      // 多初值：在名义轨迹违反要求距离的节点区间（两侧各加 initial_ramp 的余弦过渡）上，
      // 预置沿候选方向、幅值 initial_offset 的平滑“鼓包”，各自跑一次序列凸规划，取可行且代价最小者。
      int lo = n, hi = -1;
      for (int k = 0; k < n; ++k)
      {
        if (var_index_[k] < 0)
          continue;
        for (const Contact &c : nominal_states[k].contacts)
          if (c.distance < requiredDistance(k, c))
          {
            lo = std::min(lo, k);
            hi = std::max(hi, k);
          }
      }
      if (hi >= lo)
      {
        const int pad = std::max(1, static_cast<int>(std::round(config_.initial_ramp / h)));
        const std::array<std::pair<Vector3d, const char *>, 4> directions{{{-Vector3d::UnitZ(), "down"},
                                                                          {Vector3d::UnitZ(), "up"},
                                                                          {-Vector3d::UnitX(), "-x"},
                                                                          {Vector3d::UnitX(), "+x"}}};
        for (const auto &[direction, name] : directions)
        {
          Eigen::VectorXd z0 = Eigen::VectorXd::Zero(num_vars_);
          for (int k = std::max(0, lo - pad); k <= std::min(n - 1, hi + pad); ++k)
          {
            if (var_index_[k] < 0)
              continue;
            double w = 1.0;
            if (k < lo)
              w = 0.5 - 0.5 * std::cos(M_PI * (k - (lo - pad)) / pad);
            else if (k > hi)
              w = 0.5 - 0.5 * std::cos(M_PI * ((hi + pad) - k) / pad);
            const Vector3d d0 = config_.initial_offset * w * direction;
            for (int col = 0; col < var_dim_[k]; ++col)
              z0[var_index_[k] + col] = basis_[k].col(col).dot(d0); // 投影到该节点允许的偏移方向
          }
          std::vector<KnotState> states0;
          evaluate(z0, states0);
          Attempt attempt = runScp(z0, states0, name);
          ++result.starts;
          if (better(attempt, best))
            best = std::move(attempt);
        }
      }
    }
    const Eigen::VectorXd &z_best = best.z;
    const std::vector<KnotState> &states = best.states;
    const double mu = best.mu;
    result.initialization = best.label;

    // ---- 结果 ----
    double worst = 0.0;
    violation(states, &worst);
    result.worst_violation = worst;
    result.final_penalty = mu;
    result.knot_times = tau_;
    result.offsets.resize(n);
    result.knot_min_margin.assign(n, std::numeric_limits<double>::infinity());
    for (int k = 0; k < n; ++k)
    {
      result.offsets[k] = offsetAt(k, z_best);
      result.max_offset = std::max(result.max_offset, result.offsets[k].norm());
      for (const Contact &c : states[k].contacts)
        result.knot_min_margin[k] = std::min(result.knot_min_margin[k], c.distance - requiredDistance(k, c));
    }
    bool ik_ok = true;
    for (const KnotState &st : states)
      ik_ok &= st.ik_ok;
    result.success = ik_ok && worst <= kSuccessTolerance;
    result.message = !ik_ok ? "closed-chain IK failed at some knot"
                     : result.success ? "ok"
                                      : "constraints still violated (local minimum or penalty limit)";
    result.deformation = std::make_shared<PathDeformation>(result.knot_times, result.offsets);
    result.deformation->retime(*nominal_, config_.max_speed, config_.max_accel);
    result.added_duration = result.deformation->addedDuration();
    result.planning_time_s = std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
    return result;
  }

} // namespace dual_arm
