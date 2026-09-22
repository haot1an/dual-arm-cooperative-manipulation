#pragma once
/**
 * @file object_trajectory.hpp
 * @brief 物体参考轨迹：定义“任务”，基线与协同控制器共用，同时用于计算物体位姿误差。
 *
 * 所有量在世界系下，参考点 = 物体中心 o：
 *   pose  : T_o,des(t)
 *   twist : [v_o; ω_o]
 *   accel : [dv_o/dt; dω_o/dt]
 *   screw_angle / rate / accel：螺钉转角参考（只有航点带 screw_angle_deg 的场景有意义）
 *
 * 类型：
 *   waypoints（默认）：按 SceneSpec 的航点序列插值——位置直线 + 姿态 slerp（绕固定轴转动）+ 螺钉角线性，
 *              三者由同一个梯形速度剖面 u(t) ∈ [0, 1] 驱动（同步开始、同步结束），u 的速度 / 加速度上限
 *              取各量上限 / 各量行程中最小的那个（v_max/a_max、w_max/alpha_max，螺钉角沿用转动上限）；
 *              到达每个航点后停留 dwell 秒；
 *   hold      ：保持初始位姿；
 *   min_jerk  ：五次多项式平滑移动到偏移位姿；
 *   sine      ：正弦往复。
 * evaluate() 不做动态内存分配（航点段在构造时预计算）。
 */
#include "dual_arm/config.hpp"
#include "dual_arm/scene_spec.hpp"
#include "dual_arm/types.hpp"

#include <string>
#include <vector>

namespace dual_arm {

struct ObjectReference {
  Pose pose;
  Twist twist = Twist::Zero();
  Vector6d accel = Vector6d::Zero();
  double screw_angle = 0.0, screw_rate = 0.0, screw_accel = 0.0;  ///< [rad], [rad/s], [rad/s²]
  int segment = -1;  ///< 当前所在的航点段（−1 = 第一个航点之前 / 非航点轨迹）
};

/**
 * 梯形速度剖面：从 0 走到 L（L ≥ 0），速度上限 v、加速度上限 a（不足以达到 v 时退化为三角形）。
 * sample(t) 返回 (s, ds, dds)；t 超出 [0, T] 时钳位。
 */
struct TrapezoidProfile {
  double L = 0.0, v = 1.0, a = 1.0;
  double t_acc = 0.0, t_flat = 0.0, T = 0.0;
  static TrapezoidProfile make(double L, double v_max, double a_max);
  void sample(double t, double& s, double& ds, double& dds) const;
};

class ObjectTrajectory {
 public:
  /// initial_pose：t=0 时物体的位姿（通常取 SimEnv 复位后的真实位姿）
  /// waypoints：场景航点（type = waypoints 时使用；第一个航点被 initial_pose 替代）
  ObjectTrajectory(const ObjectTrajectoryConfig& cfg, const Pose& initial_pose,
                   const std::vector<Waypoint>& waypoints = {});

  ObjectReference evaluate(double t) const;
  const Pose& initialPose() const { return p0_; }
  /// waypoints 轨迹的结束时刻（其它类型返回 t_start）
  double endTime() const { return t_end_; }
  /// 第 k 段（航点 k → k+1）的 [开始, 结束) 时刻与名字
  struct Segment {
    std::string from, to;
    double t0 = 0.0, T = 0.0;          ///< 开始时刻、运动时长（之后停留 dwell）
    Pose start;
    Vector3d dp = Vector3d::Zero();    ///< 位移（世界系）
    Vector3d axis = Vector3d::UnitZ(); ///< 转轴（世界系）
    double angle = 0.0;                ///< 转角 [rad]
    double psi0 = 0.0, dpsi = 0.0;     ///< 螺钉角起点与增量
    TrapezoidProfile prof;             ///< 归一化剖面（L = 1）
  };
  const std::vector<Segment>& segments() const { return segs_; }

 private:
  ObjectTrajectoryConfig cfg_;
  Pose p0_;
  std::vector<Segment> segs_;
  double t_end_ = 0.0;
  double psi_init_ = 0.0;
};

}  // namespace dual_arm
