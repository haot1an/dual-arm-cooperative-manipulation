#pragma once
/**
 * @file closed_chain_ik.hpp
 * @brief 双臂闭链 IK：给定物体位姿，两臂 TCP 分别到达 T_obj · T_{obj, g_i}（阻尼最小二乘，每臂独立）。
 * 轨迹规划器 / 轨迹优化器共用；会调用 model.update，因此不是 const。
 */
#include "dual_arm/robot_model.hpp"

#include <array>

namespace dual_arm
{
  /// q：输入为热启动，输出为解（失败时为最后一次迭代值）。收敛：位置 < 1e-7 m、姿态 < 1e-6 rad。
  bool solveClosedChainIk(RobotModel &model, const Pose &object, std::array<Vector7d, kNumArms> &q,
                          int max_iterations = 200);

  /// 物体 twist [v; ω]（世界系，参考点 = 物体原点）→ 第 i 臂关节增量：dq_i = J_i⁺ G_iᵀ ξ，
  /// G_iᵀ = [I, −[p_i − p_o]×; 0, I]。调用前 model 须已 update 到当前构型。
  Eigen::Matrix<double, 7, 6> jointRateFromObjectTwist(const RobotModel &model, Arm arm, const Vector3d &object_origin);
} // namespace dual_arm
