#include "dual_arm/torque_qp.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

namespace dual_arm
{

  BoxTorqueQp::BoxTorqueQp(const TorqueQpConfig &config)
      : config_(config)
  {
    if (!(config_.tracking_weight > 0.0) ||
        config_.smoothing_weight < 0.0 ||
        config_.torque_margin < 0.0)
    {
      throw std::invalid_argument(
          "BoxTorqueQp: invalid weights or torque margin");
    }
  }

  void BoxTorqueQp::reset(const Vector14d &initial_torque)
  {
    previous_torque_ =
        initial_torque.allFinite()
            ? initial_torque
            : Vector14d::Zero();
    solution_ = previous_torque_;
    status_ = Status::Solved;
    active_constraints_ = 0;
  }

  const Vector14d &BoxTorqueQp::solve(
      const Vector14d &desired_torque,
      const Vector14d &lower_bound,
      const Vector14d &upper_bound)
  {
    const bool bounds_valid =
        lower_bound.allFinite() &&
        upper_bound.allFinite() &&
        (lower_bound.array() <= upper_bound.array()).all();

    if (!desired_torque.allFinite() || !bounds_valid)
    {
      // 实时安全回退：保持上一周期有限力矩，并在合法 bounds 可用时重新投影。
      solution_ = previous_torque_;
      if (bounds_valid)
      {
        solution_ =
            solution_.cwiseMax(lower_bound)
                .cwiseMin(upper_bound);
      }
      status_ = Status::InvalidInput;
      active_constraints_ = 0;
      previous_torque_ = solution_;
      return solution_;
    }

    const double denominator =
        config_.tracking_weight +
        config_.smoothing_weight;

    solution_.noalias() =
        (config_.tracking_weight * desired_torque +
         config_.smoothing_weight * previous_torque_) /
        denominator;

    active_constraints_ = 0;
    for (int i = 0; i < solution_.size(); ++i)
    {
      const double unconstrained = solution_[i];
      solution_[i] = std::clamp(
          unconstrained,
          lower_bound[i],
          upper_bound[i]);
      if (solution_[i] != unconstrained)
      {
        ++active_constraints_;
      }
    }

    previous_torque_ = solution_;
    status_ = Status::Solved;
    return solution_;
  }

  JointSafetyTorqueQp::JointSafetyTorqueQp(
      const TorqueQpConfig &config)
      : config_(config)
  {
    if (!(config_.tracking_weight > 0.0) ||
        config_.smoothing_weight < 0.0 ||
        config_.torque_margin < 0.0 ||
        config_.joint_position_margin < 0.0 ||
        !(config_.joint_prediction_horizon > 0.0) ||
        (config_.joint_velocity_limit.array() <= 0.0).any() ||
        (config_.joint_acceleration_limit.array() <= 0.0).any() ||
        config_.closed_chain_velocity_damping < 0.0 ||
        !(config_.constraint_wrench_regularization > 0.0) ||
        config_.collision_safe_distance < 0.0 ||
        config_.collision_influence_distance <=
            config_.collision_safe_distance ||
        !(config_.collision_max_approach_speed > 0.0) ||
        !(config_.collision_slack_weight > 0.0) ||
        config_.collision_max_constraints < 0 ||
        config_.collision_max_constraints >
            kMaxCollisionConstraints ||
        !(config_.admm_rho > 0.0) ||
        config_.admm_max_iterations <= 0 ||
        !(config_.admm_tolerance > 0.0))
    {
      throw std::invalid_argument(
          "JointSafetyTorqueQp: invalid configuration");
    }
  }

  void JointSafetyTorqueQp::reset(
      const Vector14d &initial_torque)
  {
    previous_torque_ =
        initial_torque.allFinite()
            ? initial_torque
            : Vector14d::Zero();
    solution_ = previous_torque_;
    acceleration_lower_.setZero();
    acceleration_upper_.setZero();
    predicted_acceleration_.setZero();
    constraint_wrench_.setZero();
    collision_slack_.setZero();
    max_collision_slack_ = 0.0;
    decision_.setZero();
    z_.setZero();
    y_.setZero();
    z_previous_.setZero();
    status_ = Status::Solved;
    iterations_ = 0;
    active_constraints_ = 0;
    max_constraint_violation_ = 0.0;
    max_constraint_violation_row_ = -1;
  }

  const Vector14d &JointSafetyTorqueQp::solve(
      const Vector14d &desired_torque,
      const Vector14d &torque_lower,
      const Vector14d &torque_upper,
      const Matrix14d &mass_matrix,
      const Vector14d &bias,
      const Vector14d &position,
      const Vector14d &velocity,
      const Vector14d &position_lower,
      const Vector14d &position_upper)
  {
    return solve(
        desired_torque,
        torque_lower,
        torque_upper,
        mass_matrix,
        bias,
        position,
        velocity,
        position_lower,
        position_upper,
        Matrix6x14d::Zero(),
        Vector6d::Zero());
  }

  const Vector14d &JointSafetyTorqueQp::solve(
      const Vector14d &desired_torque,
      const Vector14d &torque_lower,
      const Vector14d &torque_upper,
      const Matrix14d &mass_matrix,
      const Vector14d &bias,
      const Vector14d &position,
      const Vector14d &velocity,
      const Vector14d &position_lower,
      const Vector14d &position_upper,
      const Matrix6x14d &acceleration_equality_matrix,
      const Vector6d &acceleration_equality_target)
  {
    return solve(
        desired_torque,
        torque_lower,
        torque_upper,
        mass_matrix,
        bias,
        position,
        velocity,
        position_lower,
        position_upper,
        acceleration_equality_matrix,
        acceleration_equality_target,
        CollisionMatrix::Zero(),
        CollisionVector::Zero(),
        0);
  }

  const Vector14d &JointSafetyTorqueQp::solve(
      const Vector14d &desired_torque,
      const Vector14d &torque_lower,
      const Vector14d &torque_upper,
      const Matrix14d &mass_matrix,
      const Vector14d &bias,
      const Vector14d &position,
      const Vector14d &velocity,
      const Vector14d &position_lower,
      const Vector14d &position_upper,
      const Matrix6x14d &acceleration_equality_matrix,
      const Vector6d &acceleration_equality_target,
      const CollisionMatrix &collision_acceleration_matrix,
      const CollisionVector &collision_acceleration_lower,
      int active_collision_constraints)
  {
    const bool finite =
        desired_torque.allFinite() &&
        torque_lower.allFinite() &&
        torque_upper.allFinite() &&
        mass_matrix.allFinite() &&
        bias.allFinite() &&
        position.allFinite() &&
        velocity.allFinite() &&
        position_lower.allFinite() &&
        position_upper.allFinite() &&
        acceleration_equality_matrix.allFinite() &&
        acceleration_equality_target.allFinite() &&
        collision_acceleration_matrix.allFinite() &&
        collision_acceleration_lower.allFinite();

    const bool static_bounds_valid =
        (torque_lower.array() <= torque_upper.array()).all() &&
        (position_lower.array() < position_upper.array()).all() &&
        active_collision_constraints >= 0 &&
        active_collision_constraints <= kMaxCollisionConstraints;

    if (!finite || !static_bounds_valid)
    {
      solution_ = previous_torque_;
      if (torque_lower.allFinite() &&
          torque_upper.allFinite() &&
          (torque_lower.array() <= torque_upper.array()).all())
      {
        solution_ =
            solution_.cwiseMax(torque_lower)
                .cwiseMin(torque_upper);
      }
      previous_torque_ = solution_;
      status_ = Status::InvalidInput;
      iterations_ = 0;
      active_constraints_ = 0;
      max_constraint_violation_ = 0.0;
      return solution_;
    }

    const double horizon =
        config_.joint_prediction_horizon;
    const double horizon_squared =
        horizon * horizon;

    for (int arm_index = 0;
         arm_index < kNumArms;
         ++arm_index)
    {
      for (int joint = 0;
           joint < kArmDof;
           ++joint)
      {
        const int index =
            arm_index * kArmDof + joint;
        const double velocity_limit =
            config_.joint_velocity_limit[joint];
        const double acceleration_limit =
            config_.joint_acceleration_limit[joint];

        double lower = -acceleration_limit;
        double upper = acceleration_limit;

        // dq(T) = dq + ddq*T。
        lower = std::max(
            lower,
            (-velocity_limit - velocity[index]) /
                horizon);
        upper = std::min(
            upper,
            (velocity_limit - velocity[index]) /
                horizon);

        // q(T) = q + dq*T + 0.5*ddq*T^2。
        const double safe_lower =
            position_lower[index] +
            config_.joint_position_margin;
        const double safe_upper =
            position_upper[index] -
            config_.joint_position_margin;

        lower = std::max(
            lower,
            2.0 *
                (safe_lower - position[index] -
                 velocity[index] * horizon) /
                horizon_squared);
        upper = std::min(
            upper,
            2.0 *
                (safe_upper - position[index] -
                 velocity[index] * horizon) /
                horizon_squared);

        acceleration_lower_[index] = lower;
        acceleration_upper_[index] = upper;
      }
    }

    if ((acceleration_lower_.array() >
         acceleration_upper_.array())
            .any())
    {
      solution_ =
          previous_torque_.cwiseMax(torque_lower)
              .cwiseMin(torque_upper);
      previous_torque_ = solution_;
      status_ = Status::InfeasibleBounds;
      iterations_ = 0;
      active_constraints_ = 0;
      max_constraint_violation_ = 0.0;
      return solution_;
    }

    Eigen::LDLT<Matrix14d> mass_ldlt;
    mass_ldlt.compute(mass_matrix);
    if (mass_ldlt.info() != Eigen::Success)
    {
      solution_ =
          previous_torque_.cwiseMax(torque_lower)
              .cwiseMin(torque_upper);
      previous_torque_ = solution_;
      status_ = Status::InvalidInput;
      iterations_ = 0;
      active_constraints_ = 0;
      max_constraint_violation_ = 0.0;
      return solution_;
    }

    // 消去 tau 后，以 y=[ddq; lambda] 为决策变量：
    // tau = M*ddq + h - C^T*lambda。
    torque_map_.setZero();
    torque_map_.leftCols<kVariables>() =
        mass_matrix;
    torque_map_
        .block<kVariables, kEqualityConstraints>(
            0,
            kVariables) =
        -acceleration_equality_matrix.transpose();

    constraint_matrix_.setZero();
    constraint_lower_.setConstant(
        -std::numeric_limits<double>::infinity());
    constraint_upper_.setConstant(
        std::numeric_limits<double>::infinity());
    constraint_matrix_.topRows<kVariables>() =
        torque_map_;
    constraint_matrix_
        .middleRows<kVariables>(kVariables)
        .leftCols<kVariables>()
        .setIdentity();
    constraint_matrix_
        .block<kEqualityConstraints, kVariables>(
            2 * kVariables,
            0)
        .leftCols<kVariables>() =
        acceleration_equality_matrix;

    constraint_lower_.head<kVariables>() =
        torque_lower - bias;
    constraint_upper_.head<kVariables>() =
        torque_upper - bias;
    constraint_lower_.segment<kVariables>(kVariables) =
        acceleration_lower_;
    constraint_upper_.segment<kVariables>(kVariables) =
        acceleration_upper_;
    constraint_lower_.segment<kEqualityConstraints>(
        2 * kVariables) =
        acceleration_equality_target;
    constraint_upper_.segment<kEqualityConstraints>(
        2 * kVariables) =
        acceleration_equality_target;

    constexpr int collision_row =
        2 * kVariables + kEqualityConstraints;
    constexpr int slack_row =
        collision_row + kMaxCollisionConstraints;
    constexpr int slack_column =
        kVariables + kEqualityConstraints;
    for (int i = 0;
         i < kMaxCollisionConstraints;
         ++i)
    {
      if (i < active_collision_constraints)
      {
        constraint_matrix_
            .block<1, kVariables>(
                collision_row + i,
                0) =
            collision_acceleration_matrix.row(i);
        constraint_matrix_(
            collision_row + i,
            slack_column + i) = 1.0;
        constraint_lower_[collision_row + i] =
            collision_acceleration_lower[i];
      }

      // slack_i >= 0；未使用的 slack 由代价压到 0。
      constraint_matrix_(
          slack_row + i,
          slack_column + i) = 1.0;
      constraint_lower_[slack_row + i] = 0.0;
    }

    const double objective_weight =
        config_.tracking_weight +
        config_.smoothing_weight;
    DecisionMatrix objective_hessian;
    objective_hessian.noalias() =
        objective_weight *
        torque_map_.transpose() *
        torque_map_;
    objective_hessian
        .block<kEqualityConstraints,
               kEqualityConstraints>(
            kVariables,
            kVariables)
        .diagonal()
        .array() +=
        config_.constraint_wrench_regularization;
    objective_hessian
        .bottomRightCorner<kMaxCollisionConstraints,
                           kMaxCollisionConstraints>()
        .diagonal()
        .array() +=
        config_.collision_slack_weight;

    DecisionMatrix system_matrix;
    system_matrix.noalias() =
        objective_hessian +
        config_.admm_rho *
        constraint_matrix_.transpose() *
        constraint_matrix_;

    system_ldlt_.compute(system_matrix);
    if (system_ldlt_.info() != Eigen::Success)
    {
      solution_ =
          previous_torque_.cwiseMax(torque_lower)
              .cwiseMin(torque_upper);
      previous_torque_ = solution_;
      status_ = Status::InvalidInput;
      iterations_ = 0;
      active_constraints_ = 0;
      max_constraint_violation_ = 0.0;
      return solution_;
    }

    // 约束随 q 改变但周期之间连续，沿用 z/y 作为 warm start。
    DecisionVector objective_rhs;
    objective_rhs.noalias() =
        torque_map_.transpose() *
        (config_.tracking_weight *
             (desired_torque - bias) +
         config_.smoothing_weight *
             (previous_torque_ - bias));

    // 最近碰撞 pair 的集合和排序会在相邻周期切换。同一个约束行此时可能代表
    // 完全不同的几何 pair，不能沿用上一周期的 collision dual warm start。
    // torque / joint / closed-chain 行仍连续，保留它们的 warm start。
    for (int row = collision_row; row < kConstraints; ++row)
    {
      y_[row] = 0.0;
      const double ax = constraint_matrix_.row(row).dot(decision_);
      z_[row] = std::max(
          constraint_lower_[row],
          std::min(constraint_upper_[row], ax));
    }

    status_ = Status::MaxIterations;
    for (int iteration = 0;
         iteration < config_.admm_max_iterations;
         ++iteration)
    {
      DecisionVector rhs;
      rhs.noalias() =
          objective_rhs +
          config_.admm_rho *
              constraint_matrix_.transpose() *
              (z_ - y_);
      decision_ = system_ldlt_.solve(rhs);

      const ConstraintVector ax =
          constraint_matrix_ * decision_;
      z_previous_ = z_;
      z_ = ax + y_;
      z_ =
          z_.cwiseMax(constraint_lower_)
              .cwiseMin(constraint_upper_);
      y_ += ax - z_;

      const double primal_residual =
          (ax - z_).lpNorm<Eigen::Infinity>();
      const DecisionVector dual_vector =
          config_.admm_rho *
          constraint_matrix_.transpose() *
          (z_ - z_previous_);
      const double dual_residual =
          dual_vector.lpNorm<Eigen::Infinity>();

      iterations_ = iteration + 1;
      if (primal_residual <= config_.admm_tolerance &&
          dual_residual <= config_.admm_tolerance)
      {
        status_ = Status::Solved;
        break;
      }
    }

    predicted_acceleration_ =
        decision_.head<kVariables>();
    constraint_wrench_ =
        decision_.segment<kEqualityConstraints>(
            kVariables);
    // 最近点在棱边切换时 collision row 可能离散变化。固定迭代 ADMM 后，
    // 将每个独立的非负 slack 投影到当前 ddq 的最小可行值；这不会修改
    // ddq、lambda 或任何硬约束，并由 slack 数值保留软约束放松量。
    for (int i = 0; i < kMaxCollisionConstraints; ++i)
    {
      decision_[slack_column + i] =
          i < active_collision_constraints
              ? std::max(
                    0.0,
                    collision_acceleration_lower[i] -
                        collision_acceleration_matrix.row(i).dot(
                            predicted_acceleration_))
              : 0.0;
    }
    collision_slack_ =
        decision_.tail<kMaxCollisionConstraints>();
    max_collision_slack_ =
        active_collision_constraints > 0
            ? collision_slack_
                  .head(active_collision_constraints)
                  .maxCoeff()
            : 0.0;
    solution_.noalias() =
        torque_map_ * decision_ + bias;

    // 无论 ADMM 是否完全收敛，执行器硬边界必须严格满足。
    solution_ =
        solution_.cwiseMax(torque_lower)
            .cwiseMin(torque_upper);

    ConstraintVector constraint_value =
        constraint_matrix_ * decision_;
    // 诊断使用真正发给执行器的力矩，而不是 ADMM 的未截断值。
    constraint_value.head<kVariables>() =
        solution_ - bias;
    max_constraint_violation_ = 0.0;
    active_constraints_ = 0;
    for (int row = 0;
         row < kConstraints;
         ++row)
    {
      const double row_violation =
          std::max(
              constraint_lower_[row] - constraint_value[row],
              constraint_value[row] - constraint_upper_[row]);
      if (row_violation > max_constraint_violation_)
      {
        max_constraint_violation_ = row_violation;
        max_constraint_violation_row_ = row;
      }

      const double active_tolerance =
          10.0 * config_.admm_tolerance;
      if (std::abs(
              constraint_value[row] -
              constraint_lower_[row]) <=
              active_tolerance ||
          std::abs(
              constraint_value[row] -
              constraint_upper_[row]) <=
              active_tolerance)
      {
        ++active_constraints_;
      }
    }
    max_constraint_violation_ =
        std::max(0.0, max_constraint_violation_);

    if (max_constraint_violation_ <=
        10.0 * config_.admm_tolerance)
    {
      status_ = Status::Solved;
    }

    previous_torque_ = solution_;
    return solution_;
  }

} // namespace dual_arm
