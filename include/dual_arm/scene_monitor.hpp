#pragma once
/**
 * @file scene_monitor.hpp
 * @brief 场景评估量（只用于日志 / 绘图 / 测试，**不给控制器用**）：全部基于仿真真值（SimEnv::lastStep()）。
 *
 * 所有场景的列相同（不适用的量为 NaN），另加每个障碍组一列最小距离：
 *   grasp_squeeze, grasp_twist     两手抓同一 body 时，沿抓取连线 e = (p_gR − p_gL)/|·| 的简单诊断量（weld 真值）：
 *                                  挤压力 ½(f_L − f_R)·e [N]（> 0 压、< 0 拉），扭转力矩 ½(m_L − m_R)·e [N·m]
 *                                  （m_i 对各自抓取点）。完整的 6 维内力 = G 零空间投影，见日志 int_*（由你实现）
 *   roll_obj, roll_l, roll_r       物体 / 两个 TCP 相对 t=0 绕初始抓取连线 e0 的转角 [rad]
 *   roll_sync                      roll_l − roll_r：两手滚转不同步量（刚性闭链下 ≈ 0，非零即“拧”物体）
 *   obj_drift_{x,y,z,rx,ry,rz}     物体相对 t=0 的漂移，分解到物体初始坐标系（decomposePoseDrift）
 *   screw_{angle,rate,feed}        螺钉转角 [rad] / 角速度 / 进给 [m]
 *   screw_tau_{seat,damp,fric,resist}  hinge 上的座面阻力矩 / 阻尼 / 库仑摩擦 / 合计 [N·m]
 *   screw_lead_err                 螺纹耦合误差 slide − (lead/2π)·angle [m]（软约束变形）
 *   hold_ft_axis, hold_weld_axis   持握臂经腕部 F/T（扣夹爪重力）/ weld 真值施加给被抓 body 的、绕任务轴的力矩 [N·m]
 *   work_ft_axis, work_weld_axis   作业臂同上（任务轴 = 螺纹轴；没有螺旋副时 = 主物体的 z 轴，过物体原点）
 *   hold_ft_axial, hold_weld_axial 持握臂沿任务轴 +a 方向的力 [N]
 *   work_ft_axial, work_weld_axial 作业臂沿任务轴 +a 方向的力 [N]；压向工件的预紧力为负值
 *   d_min                          全部障碍对的最小有符号距离 [m]（CollisionModel，plant 真实基座位姿；> margin 记为 margin）
 *   d_<a>~<b>                      每个障碍组的最小距离
 *   grip_l1/l2/r1/r2               接触模式的四指法向力 [N]；weld 模式为 NaN
 * update() 不做动态内存分配。
 */
#include "dual_arm/collision_model.hpp"
#include "dual_arm/sim_env.hpp"

#include <memory>
#include <string>
#include <vector>

namespace dual_arm {

class SceneMonitor {
 public:
  /// with_distances = false 时不构建 CollisionModel（d_* 列为 NaN）
  explicit SceneMonitor(const SimEnv& env, bool with_distances = true);

  /// 记录 t = 0 的参考（物体 / TCP 初始位姿、抓取连线、任务轴）
  void reset(const DualArmState& s0);
  /// 计算一行（rec = SimEnv::lastStep()）
  void update(const DualArmState& rec);

  const std::vector<std::string>& columns() const { return names_; }
  const std::vector<double>& values() const { return values_; }
  /// 按列名取值（非实时路径，测试 / 打印用）
  double value(const std::string& column) const;
  const CollisionModel* collision() const { return collision_.get(); }

 private:
  const SimEnv& env_;
  std::unique_ptr<CollisionModel> collision_;
  bool same_body_ = false;
  Arm hold_ = Arm::Left, work_ = Arm::Right;
  int axis_body_ = -1;                 ///< 任务轴所在 body（螺纹孔 body 或主物体）
  Vector3d axis_local_ = Vector3d::UnitZ(), point_local_ = Vector3d::Zero();
  Pose obj0_;
  std::array<Pose, kNumArms> tcp0_;
  Vector3d e0_ = Vector3d::UnitX();
  std::vector<std::string> names_;
  std::vector<double> values_;
};

}  // namespace dual_arm
