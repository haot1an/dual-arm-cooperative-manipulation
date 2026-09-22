#pragma once
/**
 * @file config.hpp
 * @brief 运行配置（config/default.yaml + config/scenes/<scene>.yaml）的 C++ 表示。
 *
 * 加载顺序：default.yaml → 场景文件中的 overrides（合并进公共配置）→ 命令行 --set a.b.c=value。
 * 场景文件的其余部分解析为 SceneSpec（见 scene_spec.hpp；数值部分由 completeSceneSpec 从模型补全）。
 * YAML 只在启动时解析一次；控制环内只读这里的定长字段。
 */
#include "dual_arm/scene_spec.hpp"
#include "dual_arm/types.hpp"

#include <array>
#include <string>
#include <vector>

namespace dual_arm
{

  struct BaseMountConfig
  {
    Vector3d pos = Vector3d::Zero();     ///< 基座原点在世界系中的位置 [m]
    Vector3d rpy_deg = Vector3d::Zero(); ///< R = Rz(yaw)Ry(pitch)Rx(roll) [deg]
  };

  /// 右臂基座标定误差：只作用于仿真 plant，控制器模型使用名义位姿。
  struct CalibrationErrorConfig
  {
    bool enabled = false;
    Vector3d right_base_offset_xyz = Vector3d::Zero();     ///< p_true = p_nom + offset（世界系）[m]
    Vector3d right_base_offset_rpy_deg = Vector3d::Zero(); ///< R_true = R(offset) R_nom [deg]
  };

  struct WeldConfig
  {
    enum class InitMode
    {
      Current,
      Nominal
    };
    InitMode init_mode = InitMode::Current;
    /// nominal 模式下夹爪 weld 的 relpose 从“当前实际相对位姿”平滑过渡到“名义相对位姿”的时间 [s]
    double nominal_ramp_time = 0.5;
    std::array<double, 2> solref{0.004, 1.0};
    std::array<double, 5> solimp{0.95, 0.99, 0.001, 0.5, 2.0};
  };

  /// 外力 / 外力矩扰动（梯形剖面），作用在 body 质心
  struct DisturbanceConfig
  {
    enum class Frame
    {
      World,
      Body
    };
    bool enabled = true;
    std::string body; ///< 空 = 场景主物体
    Frame frame = Frame::World;
    double t_start = 0.0;
    double t_end = 0.0;
    double ramp = 0.0; ///< 上升 / 下降时间 [s]（0 = 阶跃）
    Vector3d force = Vector3d::Zero();
    Vector3d torque = Vector3d::Zero();
    /// 梯形剖面系数 ∈ [0, 1]
    double profile(double t) const;
  };

  struct ObjectTrajectoryConfig
  {
    enum class Type
    {
      Waypoints,
      Hold,
      MinJerk,
      Sine
    };
    Type type = Type::Waypoints;
    struct Waypoints
    {
      double t_start = 1.0;
      double v_max = 0.15, a_max = 0.30;     ///< 平移速度 / 加速度上限
      double w_max = 0.60, alpha_max = 1.20; ///< 转动角速度 / 角加速度上限
      double dwell = 0.5;                    ///< 每个航点的停留时间
    } waypoints;
    struct MinJerk
    {
      double t_start = 1.0, t_end = 3.0;
      Vector3d offset_xyz = Vector3d::Zero();
      Vector3d offset_rpy_deg = Vector3d::Zero();
    } min_jerk;
    struct Sine
    {
      double t_start = 1.0, frequency = 0.25;
      Vector3d amplitude_xyz = Vector3d::Zero();
      Vector3d axis = Vector3d::UnitZ();
      double amplitude_deg = 0.0;
    } sine;
  };

  /// 传感器相关选项
  struct SensorConfig
  {
    /// MuJoCo 3.12 的 force/torque 传感器把 weld 转动约束力 efc_force[3:6] 直接当作世界系力矩
    /// （mj_rnePostConstraint），而约束实际施加的力矩是 Jᵀf（转动行雅可比带 0.5 系数）——
    /// 腕部力矩读数中由 weld 引起的部分因此偏大约一倍（力分量不受影响）。
    /// true：SimEnv 用 Jᵀf 精确修正腕部力矩读数，使其与仿真动力学一致（推荐）；false：保留原始读数。
    bool fix_weld_torque = true;
  };

  /// 距离查询（CollisionModel）
  struct CollisionConfig
  {
    double margin = 0.20; ///< 距离小于它的 geom 对才会被报告 [m]
    int max_pairs = 1024; ///< geom 对数量上限（缓冲区预分配）
    int threads = 0;      ///< > 1 时给 CollisionModel 的 mjData 建 MuJoCo 线程池，窄相碰撞并行（0 = 单线程）
  };

  struct GravityPdConfig
  {
    std::array<Vector7d, kNumArms> kp;
    std::array<Vector7d, kNumArms> kd;
    std::array<Vector7d, kNumArms> extra_torque; ///< 额外叠加的恒定关节力矩（默认 0）
    GravityPdConfig();
  };

  struct CartesianImpedanceConfig
  {
    /// 单臂笛卡尔刚度：
    /// 前三维 N/m，后三维 N·m/rad。
    Vector6d stiffness = Vector6d::Zero();

    /// 单臂笛卡尔阻尼：
    /// 前三维 N·s/m，后三维 N·m·s/rad。
    Vector6d damping = Vector6d::Zero();

    /// 左臂承担的物体重力比例，必须在 (0, 1) 内。
    double load_share_left = 0.5;
  };

  /// 对称协同控制器参数占位：字段含义由你在 CoopController 中定义。
  struct CoopConfig
  {
    Vector6d object_stiffness =
        Vector6d::Zero();

    Vector6d object_damping =
        Vector6d::Zero();

    Vector6d internal_wrench_des =
        Vector6d::Zero();

    double internal_force_gain = 0.0;

    /// 内部力参考从 0 平滑增加到目标值的时间。
    double internal_wrench_ramp_time = 0.5;

    double load_share_left = 0.5;
    double nullspace_kp = 0.0;
    double nullspace_kd = 0.0;
  };

  struct LogConfig
  {
    bool enabled = true;
    std::string dir = "logs"; ///< 解析后为绝对路径
    int decimation = 1;
  };
  struct AsymCoopConfig
  {
    Arm holding_arm = Arm::Left;

    Vector6d holding_stiffness =
        Vector6d::Zero();

    Vector6d holding_damping =
        Vector6d::Zero();
    /// 作业臂在螺旋运动正交补空间中的刚度。
    Vector6d working_constraint_stiffness =
        Vector6d::Zero();

    /// 作业臂在螺旋运动正交补空间中的阻尼。
    Vector6d working_constraint_damping =
        Vector6d::Zero();

    /// 作业臂沿螺纹轴压向工件的期望预紧力大小 [N]。
    double axial_preload_force = 0.0;

    /// 轴向预紧力误差的比例反馈增益（无量纲）。
    double axial_preload_gain = 0.0;

    /// 轴向预紧力命令绝对值上限 [N]。
    double axial_preload_limit = 0.0;

    /// 预紧力由 0 平滑增加到目标值的时间 [s]。
    double axial_preload_ramp_time = 0.5;

    /// 预紧力进入角度旋拧阶段的容差和持续时间。
    double preload_ready_tolerance = 0.5;
    double preload_ready_hold_time = 0.2;

    /// 腕部轴向力 / 力矩一阶低通滤波时间常数 [s]。
    double wrench_filter_time_constant = 0.02;

    /// 座面检测：测得拧紧力矩超过阈值并持续指定时间。
    double seat_torque_threshold = 0.3;
    double seat_detect_hold_time = 0.1;

    /// 座面后的恒拧紧力矩控制（数值为拧紧方向的正值大小）。
    double tightening_torque = 1.0;
    /// 力矩误差到螺钉角速度参考的 admittance [rad/(s·N·m)]。
    double tightening_admittance = 0.2;
    double tightening_max_rate = 0.05;
    double tightening_torque_limit = 3.0;
    double tightening_torque_ramp_time = 1.0;

    /// 拧紧完成判定。
    double completion_torque_tolerance = 0.1;
    double completion_speed_threshold = 0.05;
    double completion_preload_tolerance = 0.5;
    double completion_hold_time = 0.3;

    /// 螺钉角度误差到旋拧力矩，单位 N·m/rad。
    double task_gain = 0.0;

    /// 螺钉角速度阻尼，单位 N·m·s/rad。
    double task_damping = 0.0;

    /// 作业轴最大旋拧力矩，单位 N·m。
    double task_torque_limit = 0.0;
  };

  /// 力矩级 QP：任务跟踪、相邻周期平滑、执行器边界与关节状态安全约束。
  struct TorqueQpConfig
  {
    struct ReferenceGovernorConfig
    {
      bool enabled = false;
      std::string obstacle_group;
      double trigger_distance = 0.08;
      double release_distance = 0.06;
      double avoidance_offset = 0.15;
      double offset_speed = 0.12;
      Vector3d preferred_direction = Vector3d::UnitZ();
    } reference_governor;

    double tracking_weight = 1.0;
    double smoothing_weight = 0.01;
    /// 从模型额定力矩上限中预留的安全裕量 [N·m]。
    double torque_margin = 0.5;
    /// 关节位置安全区相对 URDF/MJCF 硬限位的裕量 [rad]。
    double joint_position_margin = 0.10;
    /// 预测窗口 [s]：假设窗口内恒加速度，限制预测 q / dq。
    double joint_prediction_horizon = 0.10;
    Vector7d joint_velocity_limit = Vector7d::Constant(2.0);
    Vector7d joint_acceleration_limit = Vector7d::Constant(15.0);
    /// 闭链受约束方向的相对速度反馈增益 [1/s]。
    double closed_chain_velocity_damping = 20.0;
    /// 约束 wrench 正则，消除冗余等式方向并改善 Hessian 条件数。
    double constraint_wrench_regularization = 1e-4;
    /// 碰撞速度阻尼器：d < influence_distance 时进入 QP。
    bool collision_avoidance_enabled = true;
    double collision_safe_distance = 0.010;
    double collision_influence_distance = 0.030;
    double collision_max_approach_speed = 0.10;
    double collision_slack_weight = 1e6;
    int collision_max_constraints = 8;
    int collision_threads = 4;
    /// 固定尺寸 ADMM 参数。
    double admm_rho = 1.0;
    int admm_max_iterations = 40;
    double admm_tolerance = 1e-5;
  };

  struct SimConfig
  {
    std::string root_dir;    ///< 工程根目录（相对路径的基准）
    std::string config_path; ///< 读取的公共配置文件
    std::string scene_path;  ///< 读取的场景文件
    double timestep = 0.001;
    double duration = 10.0;
    bool realtime = true;
    bool contacts = true; ///< false：关闭所有接触（只保留约束）

    CartesianImpedanceConfig cartesian_impedance;
    std::array<BaseMountConfig, kNumArms> base; ///< 名义基座位姿（layout）
    SceneSpec scene;                            ///< 场景描述（数值字段由 completeSceneSpec 补全）

    CalibrationErrorConfig calibration_error;
    WeldConfig weld;
    std::vector<DisturbanceConfig> disturbances;
    ObjectTrajectoryConfig object_trajectory;
    SensorConfig sensors;
    CollisionConfig collision;

    std::string controller = "gravity_pd";
    GravityPdConfig gravity_pd;
    CoopConfig coop;
    AsymCoopConfig asym_coop;
    TorqueQpConfig torque_qp;
    LogConfig log;

    std::string effective_yaml; ///< 应用所有覆盖后的 YAML 文本（保存到日志目录）
  };

  /// 默认工程根目录（编译时写入的 DUAL_ARM_SOURCE_DIR）。
  std::string defaultRootDir();
  /// 相对路径 → root_dir/path；绝对路径原样返回。
  std::string resolvePath(const std::string &root_dir, const std::string &path);

  /// 读取配置并应用 "a.b.c=value" 形式的覆盖。scene 非空时覆盖 default.yaml 中的 scene 选择。
  /// 出错时抛 std::runtime_error。
  SimConfig loadConfig(const std::string &path, const std::vector<std::string> &overrides = {},
                       const std::string &scene = "", const std::string &root_dir = defaultRootDir());

  /// 名义（控制器模型使用的）基座位姿
  Pose nominalBasePose(const SimConfig &cfg, Arm arm);
  /// 仿真 plant 中的真实基座位姿（右臂叠加标定误差）
  Pose plantBasePose(const SimConfig &cfg, Arm arm);

} // namespace dual_arm
