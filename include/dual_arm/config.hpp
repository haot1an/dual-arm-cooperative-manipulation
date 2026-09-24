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
    /// 距离小于它的 geom 对才会被报告 [m]。不要设到 0.2：MuJoCo 3.12 的 native CCD 在大 margin 下对个别
    /// “网格 ~ 大盒子”对会在 mj_narrowphase 中段错误（slot_gate 中已复现，0.15 及以下正常）。
    double margin = 0.12;
    int max_pairs = 1024; ///< geom 对数量上限（缓冲区预分配）
    int threads = 0;      ///< > 1 时给 CollisionModel 的 mjData 建 MuJoCo 线程池，窄相碰撞并行（0 = 单线程）
    /// 编译前平移指定 body（世界系偏移）：用来构造“模型与真实环境不一致”的碰撞模型（例如规划器的地图误差）。
    std::vector<std::pair<std::string, Vector3d>> body_offsets;
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

  /// 对称协同控制器参数。
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

    /// 仅 contact_grasp：分配 h = G_W^+ w_o 时，绕各手指开合轴（指垫法向）的手部力矩权重倍数。
    /// 平行夹爪在该方向只能靠摩擦传约 μ·N·r 的扭矩；加大权重让物体力矩改由两手的力差产生。
    double contact_torsion_weight = 100.0;
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

  /// 接触夹取的抓取对准：预抓取 → 沿接近轴平移到抓取点 → 对准保持 → 闭合手指 → 交给任务控制器。
  struct GraspAlignmentConfig
  {
    double approach_time = 1.5;          ///< [s] 预抓取到抓取点的 min-jerk 平移时间
    Vector6d stiffness = (Vector6d() << 1500.0, 1500.0, 1500.0, 80.0, 80.0, 80.0).finished();
    Vector6d damping = (Vector6d() << 80.0, 80.0, 80.0, 6.0, 6.0, 6.0).finished();
    double nullspace_kp = 10.0;          ///< 零空间姿态（抓取构型 q_init）保持
    double nullspace_kd = 2.0;
    double position_tolerance = 0.0015;  ///< [m] 对准判定
    double orientation_tolerance = 0.017; ///< [rad]
    double speed_tolerance = 0.005;      ///< [m/s]
    double align_hold_time = 0.1;        ///< [s] 对准条件需连续满足的时间
    double close_time = 0.5;             ///< [s] 手指从全开平滑闭合到目标
    double contact_force_min = 2.0;      ///< [N] 每指法向力下限（判定已夹住）
    double settle_time = 0.3;            ///< [s] 夹住后的稳定时间
    double timeout = 8.0;                ///< [s] 超时判定对准失败
  };

  /// 物体空间轨迹优化（TrajOpt 风格序列凸规划）+ 时间参数化；公式见 docs/trajectory_planning.md。
  struct PlannerConfig
  {
    bool enabled = false;
    double knot_dt = 0.2;              ///< 节点间隔（名义轨迹时间）[s]
    double pin_start = 1.0;            ///< 起始这段时间内偏移固定为 0 [s]
    double pin_end = 1.0;              ///< 结束前这段时间内偏移固定为 0 [s]
    double safety_margin = 0.010;      ///< 在各障碍对安全距离之上再留的规划裕量 [m]
    double activation_distance = 0.06; ///< 距离小于 要求值 + 它 的 pair 才进入线性化约束 [m]
    double accel_weight = 1.0;         ///< ∫‖δ̈‖² 权重
    double offset_weight = 0.2;        ///< ∫‖δ‖² 权重
    double max_offset = 0.25;          ///< ‖δ‖∞ 上限 [m]
    int max_iterations = 40;           ///< 序列凸规划最大迭代数
    double trust_region = 0.02;        ///< 初始信赖域（每个变量）[m]
    double min_trust_region = 1e-4;
    double penalty = 10.0;             ///< 约束违反的 ℓ1 罚系数初值（每米）
    double max_penalty = 1e5;
    double max_speed = 0.12;           ///< 时间参数化：规划后路径的平移速度上限 [m/s]
    double max_accel = 0.30;           ///< 时间参数化：平移加速度上限 [m/s²]
    bool multi_start = true;           ///< 从名义轨迹出发不可行时，再从 下/上/−x/+x 四个“鼓包”初值各跑一次
    double initial_offset = 0.10;      ///< 鼓包初值的幅值 [m]
    double initial_ramp = 1.0;         ///< 鼓包两侧的余弦过渡时长 [s]

    /// lateral：在名义轨迹上只做侧向平移偏移 + 事后分段放慢（ObjectPathPlanner）；
    /// full：物体 SE(3) + 时间联合优化（ObjectTrajectoryOptimizer，以 lateral 的结果为初值）。
    enum class Formulation
    {
      Lateral,
      Full,
    } formulation = Formulation::Lateral;
    // ---- 仅 full ----
    Vector3d rotation_axes = Vector3d::Ones(); ///< 允许姿态偏移的世界系转轴（分量为 0/1）
    double max_rotation = 0.8;         ///< 每个转轴偏移上限 [rad]
    double rotation_length = 0.3;      ///< 转角与长度的换算尺度 L [m/rad]：姿态平滑权重 = accel_weight·L²
    double rotation_offset_weight = 0.05; ///< ∫‖φ‖² 权重
    double time_weight = 0.1;          ///< 总时长权重 [1/s]
    double time_smooth_weight = 5.0;   ///< Σ(Δt_{i+1} − Δt_i)² 权重
    double min_dt_ratio = 0.4;         ///< Δt 下限 / 名义节点间隔
    double max_dt_ratio = 3.0;         ///< Δt 上限 / 名义节点间隔
    double max_angular_speed = 0.6;    ///< [rad/s]
    double joint_margin = 0.10;        ///< 关节到限位的最小余量 [rad]
    double joint_activation = 0.30;    ///< 余量小于它的关节进入线性化约束 [rad]
    int full_max_iterations = 60;

    /// 规划用的环境模型误差（实验用）：规划器以为 model_error_body 在真实位置 + model_error_offset 处；
    /// 执行（plant、控制器、CBF）仍用真实位置。用于验证“规划之后仍需要执行层安全滤波”。
    std::string model_error_body;
    Vector3d model_error_offset = Vector3d::Zero();
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

      /// state_machine：NORMAL→LIFT→CROSS→DESCEND 状态机（原实现）；
      /// cbf：连续偏移 + 虚拟时间的 CBF 参考滤波（CbfReferenceFilter，公式见 docs/cbf_reference_governor.md）。
      enum class Mode
      {
        StateMachine,
        Cbf,
      } mode = Mode::StateMachine;

      /// mode = cbf 的参数。决策变量 u = [δ̇ (3); ṡ]，δ 为物体参考的平移偏移，s 为轨迹虚拟时间。
      struct Cbf
      {
        double safe_distance = 0.010;      ///< d_s [m]：h_j = d_j − d_s
        double alpha = 2.0;                ///< 线性 class-K 增益 [1/s]：ḣ ≥ −α h（需 α·dt < 1）
        double offset_max = 0.20;          ///< ‖δ‖ 上限 [m]
        double offset_speed_max = 0.15;    ///< ‖δ̇‖∞ 上限 [m/s]
        double offset_accel_max = 0.6;     ///< 相邻周期 δ̇ 变化率上限 [m/s²]
        double time_rate_accel_max = 2.5;  ///< 相邻周期 ṡ 变化率上限 [1/s]（需 ≥ alpha，否则参考刹不住，见文档 §6）
        double return_gain = 1.5;          ///< 名义偏移速度 δ̇_nom = −k_r δ 中的 k_r [1/s]
        double offset_weight = 1.0;        ///< 代价中 ‖δ̇ − δ̇_nom‖² 的权重
        double time_rate_weight = 0.05;    ///< 代价中 (ṡ − 1)² 的权重（小 → 优先绕行而不是停下）
        Vector3d escape_direction = Vector3d::UnitZ();  ///< 防死锁的逃逸偏好方向（归一化）
        double escape_gain = 0.12;         ///< 逃逸项幅值 [m/s]
        double escape_activation = 0.08;   ///< h 小于该值开始加入逃逸项 [m]
      } cbf;
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
    bool contact_grasp = false; ///< true：slot/slot_avoid/assembly 使用可动手指与摩擦接触，禁用 hand 抓取 weld
    /// 接触夹取的摩擦模型：椭圆摩擦锥 + noslip 后处理。MuJoCo 摩擦行只有速度项，
    /// 持续切向载荷（拧螺钉的扭矩）下会出现“蠕滑”；0 表示关闭 noslip。
    bool contact_elliptic_cone = true;
    int contact_noslip_iterations = 30;

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
    GraspAlignmentConfig grasp_alignment;
    PlannerConfig planner;
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
