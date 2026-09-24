#pragma once
/**
 * @file trajectory_planner.hpp
 * @brief 物体空间轨迹优化：TrajOpt 风格的序列凸规划（docs/trajectory_planning.md）。
 *
 * 在名义物体轨迹上叠加平移偏移 δ(τ)（姿态不变），使物体与两臂（经闭链 IK）对障碍保持距离：
 *
 *   min_δ  w_a ∫‖δ̈‖² dτ + w_o ∫‖δ‖² dτ + μ Σ_{k,j} max(0, r_kj − d_kj(δ_k))
 *
 * - 决策变量：均匀节点 τ_k 上的 δ_k；运动中只允许垂直于名义速度的两个方向（δ_k = B_k z_k），静止节点三维；
 *   起始 pin_start、结束 pin_end 内的节点固定为 0。
 * - d_kj：第 k 个节点、第 j 个障碍对的有符号距离（CollisionModel，两臂构型由闭链 IK 求得）；
 *   ∂d/∂δ = Σ_i (∂d/∂q_i) J_i⁺ [I₃; 0]。
 * - 要求距离 r_kj：该障碍组的名义轨迹有穿透（< −2 mm）时取 d_safe + safety_margin，
 *   否则取 min(d_safe + safety_margin, 名义距离)——“不比名义轨迹更差”，允许起点贴着支撑块、插槽 1 mm 间隙等设计。
 * - 每次迭代把距离线性化、加信赖域，解一个稀疏 QP（sparse_qp，ADMM）；按实际 / 预测下降比接受或拒绝，
 *   约束仍违反则增大罚系数 μ。
 * 规划在任务开始前离线运行一次（会分配内存）；结果是 PathDeformation，交给 ObjectTrajectory::setDeformation。
 */
#include "dual_arm/collision_model.hpp"
#include "dual_arm/config.hpp"
#include "dual_arm/object_trajectory.hpp"
#include "dual_arm/path_deformation.hpp"
#include "dual_arm/robot_model.hpp"

#include <memory>
#include <string>
#include <vector>

namespace dual_arm
{

  struct PlanResult
  {
    bool success = false;           ///< 所有节点满足要求距离（容差 0.5 mm）
    std::string message;
    int iterations = 0;             ///< QP 求解次数
    int accepted_steps = 0;
    int starts = 0;                 ///< 序列凸规划的运行次数（1 = 只从名义轨迹出发）
    std::string initialization;     ///< 最终解来自哪个初值：nominal / down / up / -x / +x
    double planning_time_s = 0.0;
    double final_penalty = 0.0;
    double max_offset = 0.0;        ///< max ‖δ_k‖ [m]
    double worst_violation = 0.0;   ///< max (r − d) over 节点与障碍对（≤ 0 表示全部满足）[m]
    double added_duration = 0.0;    ///< 执行时长 − 名义时长 [s]（负值 = 更快）
    std::string formulation = "lateral";
    double max_rotation = 0.0;      ///< max ‖φ_k‖ [rad]（仅 full）
    std::vector<Vector3d> rotation_offsets; ///< 节点姿态偏移（仅 full）
    std::vector<double> knot_times;
    std::vector<Vector3d> offsets;
    std::vector<double> knot_min_margin; ///< 每个节点 min_j (d − r)
    std::shared_ptr<PathDeformation> deformation;
  };

  class ObjectPathPlanner
  {
  public:
    /// model：控制器用的名义机器人模型（IK 与雅可比）；collision_config.margin 会按需要放大。
    ObjectPathPlanner(const PlannerConfig &config, double default_safe_distance,
                      const CollisionConfig &collision_config, std::shared_ptr<RobotModel> model,
                      std::shared_ptr<const ObjectTrajectory> nominal);

    PlanResult plan();

    /// 障碍组 group 的目标距离 d_safe + safety_margin
    double targetDistance(int group) const;
    const CollisionModel &collisionModel() const { return *collision_; }

  private:
    struct Contact
    {
      int pair = -1;
      int group = -1;
      double distance = 0.0;
      Vector3d gradient = Vector3d::Zero(); ///< ∂d/∂δ（世界系）
    };
    struct KnotState
    {
      std::array<Vector7d, kNumArms> q;
      bool ik_ok = true;
      std::vector<Contact> contacts;
    };

    PlannerConfig config_;
    double default_safe_distance_;
    std::shared_ptr<RobotModel> model_;
    std::shared_ptr<const ObjectTrajectory> nominal_;
    std::unique_ptr<CollisionModel> collision_;

    std::vector<double> tau_;                 ///< 节点名义时间
    std::vector<ObjectReference> reference_;  ///< 节点处的名义参考
    std::vector<int> var_index_;              ///< 节点 → 第一个决策变量下标（−1 = 固定）
    std::vector<int> var_dim_;                ///< 节点的自由度（0 / 2 / 3）
    std::vector<Eigen::Matrix3d> basis_;      ///< 节点的偏移基（前 var_dim 列有效）
    int num_vars_ = 0;
    std::vector<std::vector<double>> required_; ///< [节点][pair] 要求距离

    Vector3d offsetAt(int knot, const Eigen::VectorXd &z) const;
    bool solveArms(const Pose &object, std::array<Vector7d, kNumArms> &q) const;
    void evaluate(const Eigen::VectorXd &z, std::vector<KnotState> &states) const;
    double smoothCost(const Eigen::VectorXd &z) const;
    double violation(const std::vector<KnotState> &states, double *worst = nullptr) const;
  };

} // namespace dual_arm
