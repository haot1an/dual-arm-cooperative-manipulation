#pragma once
/**
 * @file sim_env.hpp
 * @brief MuJoCo 仿真封装（“plant”，即真实世界）。场景由 SceneSpec 描述，不含任何布局假设。
 *
 * 职责：
 *  - 加载场景 MJCF，按 config 设置基座真实位姿（右臂可注入标定误差）与手指开度；
 *  - reset()：设置初始关节角、主物体放到第一个航点、工具放进手里，初始化所有 rigid 约束（weld）的 relpose；
 *  - step(tau_l, tau_r)：施加（饱和后的）关节力矩、扰动、螺钉座面阻力矩，推进一个 timestep；
 *  - 提供状态读取：q, dq, 末端位姿/速度, 腕部 F/T, 夹爪 weld 约束力, 主物体位姿/速度, 螺钉状态。
 *
 * 与控制器模型的关系：SimEnv 自己持有一份 mjModel（真实基座位姿）；控制器通过
 * RobotModel（MujocoRobotModel 持有另一份 mjModel，名义基座位姿）计算运动学/动力学。
 *
 * 单步时序（每个控制周期）：
 *     state()  ← 已由上一次 mj_step1 算好：q(t), dq(t), 末端位姿(t)；力测量来自 t−dt
 *     tau      = controller.compute(state(), t)
 *     step(tau):  写 ctrl / xfrc_applied / qfrc_applied → mj_step2（t 时刻的加速度、约束力、F/T，
 *                 然后积分到 t+dt）→ 记录 lastStep()（全部对齐到 t）→ mj_step1（t+dt 的运动学）
 * 因此 lastStep() 是完整、时间一致的一行日志；state() 中的力是 1 个周期前的测量（等价于 1 ms 传感器延迟）。
 *
 * 控制环内（step / state / 各 getter）不做任何动态内存分配。
 */
#include "dual_arm/config.hpp"
#include "dual_arm/mujoco_utils.hpp"
#include "dual_arm/scene_spec.hpp"
#include "dual_arm/types.hpp"

#include <array>
#include <memory>
#include <vector>

namespace dual_arm {

class SimEnv {
 public:
  explicit SimEnv(const SimConfig& cfg);

  /// 回到初始状态（t = 0）。构造函数中已调用一次。
  void reset();

  /// 施加关节力矩（超出 ctrlrange 的部分被截断）并推进一个 timestep。
  void step(const Vector7d& tau_left, const Vector7d& tau_right);

  /// 在外部直接修改了 data()（qpos、eq_active 等）之后调用：mj_forward 并刷新 state()/lastStep()。
  void forward();

  /// 当前时刻的观测（力相关字段为上一步的测量，见文件头说明）。
  const DualArmState& state() const { return state_; }
  /// 刚刚完成的那一步的完整记录：所有字段对齐到该步开始时刻 t（用于日志）。
  const DualArmState& lastStep() const { return last_step_; }

  double time() const { return data_->time; }
  double timestep() const { return model_->opt.timestep; }

  /// 场景描述（已用本模型补全数值字段）
  const SceneSpec& scene() const { return *scene_; }
  std::shared_ptr<const SceneSpec> sceneShared() const { return scene_; }

  // ------------------------------------------------------------------------
  // 单项读取。均基于 mjData 中当前已计算的量（运动学在 mj_step1 后有效，
  // 力相关量在 mj_step2 / mj_forward 后有效）。一般直接用 state()/lastStep() 即可。
  // ------------------------------------------------------------------------
  /// ee_site（TCP）在世界系中的真实位姿
  Pose eePose(Arm a) const;
  /// ee_site 的真实 twist [v; ω]，世界系，参考点 = ee_site 原点
  Twist eeTwist(Arm a) const;
  /// 腕部 F/T 读数 [f; m]：ft_site 坐标系，力矩参考点 = ft_site 原点，
  /// 含义 = link7 施加给 hand 子树（hand + 两指，0.76 kg）的力/力矩（含其重力与惯性力）。
  /// 若 config.sensors.fix_weld_torque = true，力矩已按 weldTorqueCorrection() 修正。
  Wrench wristFtRaw(Arm a) const;
  /// MuJoCo 腕部力矩读数中夹爪 weld 转动约束部分的误差修正量（世界系，纯力偶）：
  ///   Δτ = f_rot − τ_true,hand = f_rot + τ_rot,body
  /// f_rot = efc_force 的后 3 行（MuJoCo 在 mj_rnePostConstraint 中直接当作施加给 hand 的世界系力矩），
  /// τ_rot,body = 这 3 行经 Jᵀf 真正作用在被抓 body 上的力矩（hand 上为其相反数）。
  /// 修正后的读数 = MuJoCo 读数 + R_siteᵀ Δτ。
  Vector3d weldTorqueCorrection(Arm a) const;
  /// 由腕部 F/T 换算的“夹爪施加给被抓 body 的 wrench”：世界系，参考点 = ee_site（TCP），
  /// 已扣除 hand 子树的静态重力 m_hand·g（未扣除惯性力 m_hand·a）
  Wrench wristWrenchOnObject(Arm a) const;
  /**
   * 通过夹爪 weld，hand 施加给被抓 body（物体或工具）的 wrench [f; m]：世界系，参考点 = 抓取点。
   *
   * 计算方法（精确）：从 efc 中取出该 weld 的 6 行（efc_type == mjCNSTR_EQUALITY 且
   * efc_id == eq_id），计算这 6 行约束力对被抓 body 自由关节的广义力 J_rowsᵀ·efc_force，
   * 其前 3 维为世界系力、后 3 维为绕 body 原点的力矩（body 局部系），再换到世界系与抓取点。
   *
   * 与腕部 F/T 的区别：
   *  (1) weld 力是“约束直接作用在被抓 body 上的力”，不含夹爪重力/惯性，是内力的“理想测量”，
   *      真实系统中拿不到；腕部 F/T 是 link7 与 hand 之间的力，需补偿夹爪重力/惯性后才等于它；
   *  (2) weld 力在 mj_step2 中与加速度一起求出，不含传感器噪声；
   *  (3) MuJoCo 的 weld 转动行雅可比带 0.5 系数且表达在 body2 相关坐标系中，efc_force 的后
   *      3 维并不是世界系力矩——因此这里用 Jᵀf 换算，而不是直接读 efc_force。
   */
  Wrench weldWrench(Arm a) const;
  /// contact_grasp 模式下指定手指对被抓 body 的总法向接触力 [N]；weld 模式返回 0。
  double fingerContactNormal(Arm a, int finger) const;
  double fingerContactTangent(Arm a, int finger) const;
  double fingerContactMu(Arm a, int finger) const;
  /// contact_grasp 模式下当前手指开度 [m]。
  double fingerOpening(Arm a, int finger) const;
  /// 接触模式下指定臂的夹指目标开度 [m]；负值恢复全开（0.04 m）。
  void setFingerTarget(Arm a, double opening);
  /// 接触模式：下次 reset() 时两臂的起始构型（预抓取）；工具仍按 scene q_init 的抓取构型摆放。
  void setStartConfiguration(const std::array<Vector7d, kNumArms>& q);
  /// 接触模式：撤除把主物体固定在世界系的临时托持工装（抓取完成后调用）。
  void releaseStaging();
  bool stagingActive() const;
  /// 任一 weld（equality id）对 body（必须带 free joint）施加的 wrench：世界系，参考点 = body 原点
  Wrench constraintWrenchOnBody(int eq_id, int body) const;
  /// 主物体位姿与 twist（参考点 = 物体中心，世界系）
  ObjectState objectState() const;
  /// 螺钉状态（运动学部分 + 最近一次计算的阻力矩）
  ScrewState screwState() const;
  /// 在时刻 t 作用在主物体上的扰动之和 [f; m]（世界系，作用于质心）
  Wrench disturbanceAt(double t) const;
  /// 仿真是否发散（qpos/qvel 出现 NaN/Inf 或 MuJoCo 报告 BADQACC 等警告）
  bool diverged() const;

  /// plant 中基座的真实位姿（含标定误差）
  Pose basePose(Arm a) const { return base_true_[armIndex(a)]; }
  /// reset 时 ee_site 相对于对应抓取点的位姿误差 [Δp; Δθ]（世界系）；current 模式下被 relpose 吸收
  const Vector6d& initialGraspMismatch(Arm a) const { return init_mismatch_[armIndex(a)]; }
  const Vector7d& torqueLimit(Arm a) const { return tau_limit_[armIndex(a)]; }
  const SimConfig& config() const { return cfg_; }

  const mjModel* model() const { return model_.get(); }
  mjData* data() const { return data_.get(); }
  const SceneIndices& indices() const { return idx_; }

 private:
  void initWelds();
  void writeRelpose(int eq, const Vector3d& pos, const Quaterniond& quat);
  void updateWeldRamp(double t);
  void applyDisturbances(double t);
  void applyScrewSeatTorque();
  int weldFirstRow(int eq) const;
  void readKinematics(DualArmState& s) const;
  void readForces(DualArmState& s) const;
  void updateFingerContacts();

  SimConfig cfg_;
  std::shared_ptr<SceneSpec> scene_;
  MjModelPtr model_;
  MjDataPtr data_;
  SceneIndices idx_;
  std::array<Pose, kNumArms> base_true_;
  std::array<Vector7d, kNumArms> tau_limit_;
  std::array<Vector6d, kNumArms> init_mismatch_;
  std::array<std::array<int, 2>, kNumArms> finger_qpos_{{{-1, -1}, {-1, -1}}};
  std::array<std::array<int, 2>, kNumArms> finger_act_{{{-1, -1}, {-1, -1}}};
  std::array<std::array<int, 2>, kNumArms> finger_body_{{{-1, -1}, {-1, -1}}};
  std::array<std::array<double, 2>, kNumArms> finger_normal_{};
  std::array<std::array<double, 2>, kNumArms> finger_tangent_{};
  std::array<std::array<double, 2>, kNumArms> finger_mu_{};
  std::array<double, kNumArms> finger_target_override_{{-1.0, -1.0}};
  std::array<Vector7d, kNumArms> start_q_{};
  bool start_q_set_ = false;
  int staging_eq_ = -1;
  std::vector<int> dist_body_;      ///< 每条扰动作用的 body id
  double tau_seat_ = 0.0;           ///< 本步施加的螺钉座面阻力矩
  // 夹爪 weld 的 relpose：t=0 的实际值与名义值（nominal 模式下在两者之间过渡）
  std::array<Vector3d, kNumArms> relpos_current_, relpos_nominal_;
  std::array<Quaterniond, kNumArms> relquat_current_, relquat_nominal_;
  bool ramp_active_ = false;
  DualArmState state_;
  DualArmState last_step_;
};

}  // namespace dual_arm
