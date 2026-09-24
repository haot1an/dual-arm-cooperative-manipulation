#pragma once
/**
 * @file trajectory_optimizer.hpp
 * @brief 物体 SE(3) + 时间的轨迹联合优化（直接配点 + 序列凸规划；docs/trajectory_planning.md §10）。
 *
 * 决策变量（起始保持段与最后一段插槽运动固定为名义值）：
 *   p_k ∈ R³        节点位置（完整三维，不再限制侧向）
 *   φ_k             节点姿态偏移，R_k = Exp(φ_k) R̄_k，只在 rotation_axes 选中的世界系转轴上取值
 *   Δt_i            区间 [k_i, k_i+1] 的执行时长（节点时刻 t_k = Σ Δt）
 *
 *   min  w_a h Σ‖Δ²p‖²/h⁴ + w_a L² h Σ‖Δ²φ‖²/h⁴ + w_φ h Σ‖φ‖² + w_T Σ Δt + w_Δ Σ (Δt_{i+1} − Δt_i)²
 *   s.t. d_kj(p_k, R_k) ≥ r_kj                        （物体 + 两臂经闭链 IK；r 同 ObjectPathPlanner）
 *        q_min + ε ≤ q_i(p_k, R_k) ≤ q_max − ε         （关节余量 = 可达性）
 *        ‖p_{i+1} − p_i‖ ≤ v_max Δt_i,  ‖a_k‖ ≤ a_max, ‖Log(R_{i+1} R_iᵀ)‖ ≤ ω_max Δt_i
 *        Δt_i ∈ [min_dt_ratio, max_dt_ratio]·h,  |φ| ≤ max_rotation
 *
 * 梯度：物体 twist ξ = [dp; J_l(φ) dφ] → 两臂关节增量 dq_i = J_i⁺ G_iᵀ ξ（closed_chain_ik.hpp），
 *       距离 ∂d/∂ξ = Σ_i (∂d/∂q_i) J_i⁺ G_iᵀ；速度 / 加速度 / 角速度的梯度解析给出。
 * 全部约束写成 g(x) ≤ 0，每次迭代线性化 + ℓ1 罚 + 信赖域，解稀疏 QP（sparse_qp）。
 * 输出 PathDeformation：δ_k = p_k − p_nom(τ_k)、φ_k、节点执行时刻 t_k。离线运行，会分配内存。
 */
#include "dual_arm/collision_model.hpp"
#include "dual_arm/config.hpp"
#include "dual_arm/object_trajectory.hpp"
#include "dual_arm/robot_model.hpp"
#include "dual_arm/trajectory_planner.hpp"

#include <memory>
#include <vector>

namespace dual_arm
{

  class ObjectTrajectoryOptimizer
  {
  public:
    ObjectTrajectoryOptimizer(const PlannerConfig &config, double default_safe_distance,
                              const CollisionConfig &collision_config, std::shared_ptr<RobotModel> model,
                              std::shared_ptr<const ObjectTrajectory> nominal);

    /// warm_start：只平移规划器（同一节点网格）的结果，用其偏移作初值；nullptr 时从名义轨迹出发。
    PlanResult optimize(const PlanResult *warm_start = nullptr);

    int numVariables() const { return num_vars_; }

  private:
    struct Contact
    {
      int pair = -1, group = -1;
      double distance = 0.0;
      Vector6d gradient = Vector6d::Zero(); ///< ∂d/∂ξ（物体 twist，世界系，参考点物体原点）
    };
    struct KnotEval
    {
      std::array<Vector7d, kNumArms> q;
      bool ik_ok = true;
      std::vector<Contact> contacts;
      std::array<Eigen::Matrix<double, 7, 6>, kNumArms> dq_dxi; ///< dq_i/dξ
    };
    /// 非线性约束 g(x) ≤ 0 在当前点的值与稀疏梯度
    struct Row
    {
      double g = 0.0;
      std::vector<std::pair<int, double>> gradient;
      bool linearize = true; ///< 是否进入本次 QP（离可行边界足够远的行只计入 merit）
    };

    PlannerConfig config_;
    double default_safe_distance_;
    std::shared_ptr<RobotModel> model_;
    std::shared_ptr<const ObjectTrajectory> nominal_;
    std::unique_ptr<CollisionModel> collision_;

    int n_knots_ = 0;
    double h_ = 0.0;
    std::vector<double> tau_;
    std::vector<ObjectReference> reference_;
    std::vector<bool> pinned_;
    std::vector<Vector3d> axes_;              ///< 允许的姿态偏移转轴（世界系单位向量）
    std::vector<int> pos_index_, rot_index_;  ///< 节点 → 变量下标（−1 = 固定）
    std::vector<int> dt_index_;               ///< 区间 → 变量下标（−1 = 固定为 h）
    int num_vars_ = 0;
    std::vector<std::vector<double>> required_; ///< [节点][pair]

    Vector3d position(int k, const Eigen::VectorXd &x) const;
    Vector3d rotationOffset(int k, const Eigen::VectorXd &x) const;
    double intervalTime(int i, const Eigen::VectorXd &x) const;
    Pose knotPose(int k, const Eigen::VectorXd &x) const;
    double targetDistance(int group) const;

    void evaluateKnots(const Eigen::VectorXd &x, std::vector<KnotEval> &knots) const;
    /// 全部约束行（碰撞、关节、速度、加速度、角速度）
    void buildRows(const Eigen::VectorXd &x, const std::vector<KnotEval> &knots, std::vector<Row> &rows) const;
    /// ξ 的梯度 → 节点 k 的决策变量梯度
    void addKnotGradient(int k, const Eigen::VectorXd &x, const Vector6d &d_dxi, double scale,
                         std::vector<std::pair<int, double>> &out) const;
    static double violation(const std::vector<Row> &rows, double *worst = nullptr);
    /// “推迟转动”初值：自由窗口内保持起始姿态、在窗口末尾按名义转动时长完成姿态变化；位置取名义值。
    /// 所需转动不在允许转轴上或超出 max_rotation 时返回 false。
    bool lateRotationStart(const Eigen::VectorXd &x_nominal, Eigen::VectorXd &x0) const;
  };

} // namespace dual_arm
