#pragma once
/**
 * @file torque_qp.hpp
 * @brief 固定尺寸 14 维 box-constrained torque QP。
 *
 * 求解：
 *   min_tau  0.5*w_track*||tau - tau_des||^2
 *          + 0.5*w_smooth*||tau - tau_prev||^2
 *   s.t.    tau_lower <= tau <= tau_upper
 *
 * Hessian 为正定对角阵，因此精确解是无约束加权最优值在 box 上的逐元素投影。
 * 控制周期内不进行动态内存分配；这是后续加入耦合约束前的 QP baseline。
 */
#include "dual_arm/config.hpp"
#include "dual_arm/types.hpp"

#include <Eigen/Cholesky>

namespace dual_arm
{

  class BoxTorqueQp
  {
  public:
    enum class Status : int
    {
      Solved = 0,
      InvalidInput = 1,
    };

    explicit BoxTorqueQp(const TorqueQpConfig &config);

    void reset(const Vector14d &initial_torque = Vector14d::Zero());

    const Vector14d &solve(
        const Vector14d &desired_torque,
        const Vector14d &lower_bound,
        const Vector14d &upper_bound);

    const Vector14d &solution() const { return solution_; }
    Status status() const { return status_; }
    int activeConstraints() const { return active_constraints_; }

  private:
    TorqueQpConfig config_;
    Vector14d previous_torque_ = Vector14d::Zero();
    Vector14d solution_ = Vector14d::Zero();
    Status status_ = Status::Solved;
    int active_constraints_ = 0;
  };

  /**
   * 闭链关节安全 QP。决策变量 y=[ddq(14); lambda(6); collision_slack(8)]，
   * 固定约束矩阵最多 50 行：
   *   tau = [M, -C^T] y + h in [tau_min, tau_max]
   *   ddq in [ddq_min, ddq_max]
   *   C ddq = b（最多 6 条闭链相对加速度等式）
   *   g_d ddq + slack >= b_d，slack >= 0（最多 8 条碰撞速度阻尼器）
   * 动力学满足 M ddq + h = tau + C^T lambda，lambda 是闭链约束 wrench。
   * 其中 ddq bounds 同时考虑加速度硬限制、预测速度与预测位置安全区。
   */
  class JointSafetyTorqueQp
  {
  public:
    static constexpr int kVariables = 2 * kArmDof;
    static constexpr int kEqualityConstraints = 6;
    static constexpr int kMaxCollisionConstraints = 8;
    static constexpr int kDecisionVariables =
        kVariables + kEqualityConstraints +
        kMaxCollisionConstraints;
    static constexpr int kConstraints =
        2 * kVariables + kEqualityConstraints +
        2 * kMaxCollisionConstraints;
    using ConstraintVector =
        Eigen::Matrix<double, kConstraints, 1>;
    using ConstraintMatrix =
        Eigen::Matrix<double, kConstraints, kDecisionVariables>;
    using DecisionVector =
        Eigen::Matrix<double, kDecisionVariables, 1>;
    using DecisionMatrix =
        Eigen::Matrix<double, kDecisionVariables, kDecisionVariables>;
    using TorqueMap =
        Eigen::Matrix<double, kVariables, kDecisionVariables>;
    using CollisionMatrix =
        Eigen::Matrix<double, kMaxCollisionConstraints, kVariables>;
    using CollisionVector =
        Eigen::Matrix<double, kMaxCollisionConstraints, 1>;

    enum class Status : int
    {
      Solved = 0,
      MaxIterations = 1,
      InvalidInput = 2,
      InfeasibleBounds = 3,
    };

    explicit JointSafetyTorqueQp(const TorqueQpConfig &config);
    void reset(const Vector14d &initial_torque = Vector14d::Zero());

    const Vector14d &solve(
        const Vector14d &desired_torque,
        const Vector14d &torque_lower,
        const Vector14d &torque_upper,
        const Matrix14d &mass_matrix,
        const Vector14d &bias,
        const Vector14d &position,
        const Vector14d &velocity,
        const Vector14d &position_lower,
        const Vector14d &position_upper);

    const Vector14d &solve(
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
        const Vector6d &acceleration_equality_target);

    const Vector14d &solve(
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
        int active_collision_constraints);

    const Vector14d &solution() const { return solution_; }
    const Vector14d &accelerationLowerBound() const { return acceleration_lower_; }
    const Vector14d &accelerationUpperBound() const { return acceleration_upper_; }
    const Vector14d &predictedAcceleration() const { return predicted_acceleration_; }
    const Vector6d &constraintWrench() const { return constraint_wrench_; }
    double maxCollisionSlack() const { return max_collision_slack_; }
    Status status() const { return status_; }
    int iterations() const { return iterations_; }
    int activeConstraints() const { return active_constraints_; }
    double maxConstraintViolation() const { return max_constraint_violation_; }
    int maxConstraintViolationRow() const { return max_constraint_violation_row_; }

  private:
    TorqueQpConfig config_;
    Vector14d previous_torque_ = Vector14d::Zero();
    Vector14d solution_ = Vector14d::Zero();
    Vector14d acceleration_lower_ = Vector14d::Zero();
    Vector14d acceleration_upper_ = Vector14d::Zero();
    Vector14d predicted_acceleration_ = Vector14d::Zero();
    Vector6d constraint_wrench_ = Vector6d::Zero();
    CollisionVector collision_slack_ = CollisionVector::Zero();
    double max_collision_slack_ = 0.0;
    DecisionVector decision_ = DecisionVector::Zero();
    TorqueMap torque_map_ = TorqueMap::Zero();
    ConstraintMatrix constraint_matrix_ = ConstraintMatrix::Zero();
    ConstraintVector constraint_lower_ = ConstraintVector::Zero();
    ConstraintVector constraint_upper_ = ConstraintVector::Zero();
    ConstraintVector z_ = ConstraintVector::Zero();
    ConstraintVector y_ = ConstraintVector::Zero();
    ConstraintVector z_previous_ = ConstraintVector::Zero();
    Eigen::LDLT<DecisionMatrix> system_ldlt_;
    Status status_ = Status::Solved;
    int iterations_ = 0;
    int active_constraints_ = 0;
    double max_constraint_violation_ = 0.0;
    int max_constraint_violation_row_ = -1;
  };

} // namespace dual_arm
