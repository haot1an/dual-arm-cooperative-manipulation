#pragma once
/**
 * @file types.hpp
 * @brief 全工程共用的数据结构与坐标约定。
 *
 * ============================ 坐标 / 符号约定 ============================
 *  - 世界系 W：原点在地面、两基座连线中点的正下方；x 沿两臂连线（左臂在 +x），y 指向操作者，z 向上
 *    （见 models/workcell.xml）。两台 Panda 同侧并排、朝 −y 工作。
 *  - arm 1 = 左臂 (Arm::Left, 前缀 left_，在 +x)，arm 2 = 右臂 (Arm::Right, 前缀 right_，在 −x)；
 *    “左 / 右”以机器人自身朝向（面向 −y）为准。
 *  - Twist  = [v; ω] ∈ R^6：线速度在前、角速度在后。
 *  - Wrench = [f; m] ∈ R^6：力在前、力矩在后。
 *    一个 twist / wrench 必须同时说明 (a) 表达坐标系（各分量在哪个坐标系下投影）
 *    和 (b) 参考点（线速度 / 力矩是对哪个点而言）。本工程中除非特别注明，
 *    表达坐标系都是世界系 W，参考点在变量名或注释中给出。
 *  - 四元数使用 Eigen::Quaterniond（内部存储 x,y,z,w；构造函数参数顺序为 w,x,y,z）。
 *    MuJoCo 的四元数数组顺序是 (w,x,y,z)，转换时注意。
 *  - 关节向量 q, dq, tau 均为 7 维，顺序 joint1..joint7。
 * ========================================================================
 */
#include <Eigen/Core>
#include <Eigen/Geometry>

#include <array>

namespace dual_arm {

constexpr int kArmDof = 7;
constexpr int kNumArms = 2;

using Vector3d = Eigen::Vector3d;
using Vector6d = Eigen::Matrix<double, 6, 1>;
using Vector7d = Eigen::Matrix<double, 7, 1>;
using Vector12d = Eigen::Matrix<double, 12, 1>;
using Vector14d = Eigen::Matrix<double, 14, 1>;
using Matrix3d = Eigen::Matrix3d;
using Matrix6d = Eigen::Matrix<double, 6, 6>;
using Matrix7d = Eigen::Matrix<double, 7, 7>;
using Matrix12d = Eigen::Matrix<double, 12, 12>;
using Matrix14d = Eigen::Matrix<double, 14, 14>;
using Matrix6x7d = Eigen::Matrix<double, 6, 7>;
using Matrix6x12d = Eigen::Matrix<double, 6, 12>;
using Matrix12x6d = Eigen::Matrix<double, 12, 6>;
using Matrix6x14d = Eigen::Matrix<double, 6, 14>;
using Quaterniond = Eigen::Quaterniond;

/// Twist [v; ω]（线速度在前）。表达坐标系与参考点见变量注释。
using Twist = Vector6d;
/// Wrench [f; m]（力在前）。表达坐标系与参考点见变量注释。
using Wrench = Vector6d;

enum class Arm : int { Left = 0, Right = 1 };
inline constexpr std::array<Arm, kNumArms> kArms{Arm::Left, Arm::Right};
inline constexpr int armIndex(Arm a) { return static_cast<int>(a); }
inline constexpr const char* armName(Arm a) { return a == Arm::Left ? "left" : "right"; }

/// 刚体位姿 T = (p, q)：把局部坐标 x 映射为 p + R(q) x。
struct Pose {
  Vector3d p = Vector3d::Zero();
  Quaterniond q = Quaterniond::Identity();

  Matrix3d R() const { return q.toRotationMatrix(); }
  static Pose Identity() { return Pose{}; }
  /// 位姿复合：(*this) ∘ other
  Pose operator*(const Pose& other) const {
    Pose out;
    out.p = p + q * other.p;
    out.q = (q * other.q).normalized();
    return out;
  }
  Pose inverse() const {
    Pose out;
    out.q = q.conjugate();
    out.p = -(out.q * p);
    return out;
  }
  /// 把局部点变换到本位姿的父坐标系
  Vector3d transformPoint(const Vector3d& x) const { return p + q * x; }
};

/// 单臂状态。
///
/// 时间对齐（重要）：SimEnv::state() 返回的状态中，运动学量（q, dq, ee_pose, ee_twist）
/// 对应当前时刻 t；力相关量（ft_raw, ft_ee_world, weld_wrench）来自上一个仿真步
/// （在 t−dt 时刻、以 tau(t−dt) 计算），相当于 1 个控制周期的传感器延迟。
/// SimEnv::lastStep() 返回的记录中所有字段都对齐到同一时刻（用于日志）。
struct ArmState {
  // ---------- 真实机器人上也能测到的量 ----------
  Vector7d q = Vector7d::Zero();    ///< 关节角 [rad]
  Vector7d dq = Vector7d::Zero();   ///< 关节角速度 [rad/s]
  Vector7d tau = Vector7d::Zero();  ///< 实际施加的关节力矩（饱和后）[N·m]；state() 中为上一步的值
  /// 腕部 F/T 传感器读数 [f; m]：表达在 ft_site 坐标系，力矩参考点 = ft_site 原点，
  /// 物理含义 = link7 施加给 hand 子树（Franka Hand + 两根固定手指，0.76 kg）的力/力矩。
  /// 包含夹爪自身重力与惯性力，控制器侧需要补偿（见 ft_ee_world）。
  Wrench ft_raw = Wrench::Zero();
  /// 由 ft_raw 换算：夹爪施加给被夹物体的 wrench。表达在世界系，参考点 = ee_site（TCP），
  /// 已扣除 hand 子树的（静态）重力，未扣除其惯性力（加速时有 m_hand·a 量级的误差）。
  Wrench ft_ee_world = Wrench::Zero();

  // ---------- 仿真真值（只用于记录与评估；控制器应通过 RobotModel 计算运动学）----------
  Pose ee_pose;                          ///< ee_site（TCP）在世界系中的真实位姿（含标定误差）
  Twist ee_twist = Twist::Zero();        ///< ee_site 真实 twist [v; ω]，世界系，参考点 = ee_site 原点
  /// 通过夹爪 weld，hand 施加给被抓 body（物体或工具）的 wrench：世界系，参考点 = 抓取点（weld anchor）。
  /// 由约束力 efc_force 精确换算（见 SimEnv::weldWrench 的注释）。
  Wrench weld_wrench = Wrench::Zero();
};

/// 螺钉（螺旋副）状态：只有含 screw 约束的场景 valid = true。
struct ScrewState {
  bool valid = false;
  double angle = 0.0;         ///< hinge 转角 [rad]（负 = 从上往下看顺时针 = 拧紧）
  double rate = 0.0;          ///< hinge 角速度 [rad/s]
  double feed = 0.0;          ///< 轴向进给（拧入深度）= −slide [m]
  double lead_error = 0.0;    ///< 螺纹耦合误差 slide − (lead/2π)·angle [m]（软约束的变形，应 ≪ 导程）
  double tau_seat = 0.0;      ///< 座面贴合阻力矩（SimEnv 施加在 hinge 上，+z 方向为正）[N·m]
  double tau_damping = 0.0;   ///< hinge 粘性阻尼力矩 −b·ω [N·m]
  double tau_friction = 0.0;  ///< hinge 库仑摩擦力矩（frictionloss 约束力）[N·m]
  /// 作用在螺钉 hinge 上的总阻力矩
  double tauResist() const { return tau_seat + tau_damping + tau_friction; }
};

/// 物体状态：物体坐标系 o 位于几何中心（= 质心）。
struct ObjectState {
  Pose pose;                      ///< 世界系下位姿
  Twist twist = Twist::Zero();    ///< [v; ω]，世界系，参考点 = 物体中心 o
};

struct DualArmState {
  double t = 0.0;                              ///< 仿真时间 [s]
  std::array<ArmState, kNumArms> arms;         ///< arms[armIndex(Arm::Left)] 为左臂
  ObjectState object;                          ///< 主物体真值（相当于理想动捕测量）
  Wrench disturbance = Wrench::Zero();         ///< 施加在主物体质心的外部扰动 [f; m]，世界系
  ScrewState screw;                            ///< 螺钉状态（装配场景）

  ArmState& arm(Arm a) { return arms[armIndex(a)]; }
  const ArmState& arm(Arm a) const { return arms[armIndex(a)]; }
};

}  // namespace dual_arm
