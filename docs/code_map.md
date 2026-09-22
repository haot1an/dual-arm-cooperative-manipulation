# 代码地图

## 1. 目录与文件

```
dual_arm_ws/
├── CMakeLists.txt                 纯 CMake：dual_arm_core（核心库）/ dual_arm_viewer（GLFW）/ run_sim / 测试
├── config/default.yaml            所有运行参数（基座位姿、q_init、标定误差、weld、扰动、轨迹、控制器、日志）
├── models/
│   ├── franka_emika_panda/        从 mujoco_menagerie 原样复制，不修改
│   ├── panda_hand_torque.xml      由 panda.xml 派生：力矩电机 + 固定手指的 Franka Hand + TCP + F/T 传感器
│   ├── dual_panda_scene.xml       双臂场景：工作台 + <attach prefix=left_/right_> ×2 + 箱子 + 两个 weld + 固定相机
│   └── README.md                  模型来源、许可证、改动说明
├── include/dual_arm/ , src/       （见下表）
├── apps/run_sim.cpp               主程序：加载 → 控制循环 → 日志 / 可视化
├── scripts/plot_log.py            画 CSV，支持多次运行对比
├── tests/                         GTest
└── docs/                          本文件 + cooperative_control.md（原理）
```

| 头文件 | 作用 | 状态 |
|---|---|---|
| `types.hpp` | `Pose`、`Twist=[v;ω]`、`Wrench=[f;m]`、`ArmState`、`ObjectState`、`DualArmState`、`Arm` 枚举；**坐标约定总说明** | 完成 |
| `math_utils.hpp` | `skew`、RPY→四元数、姿态误差（旋转向量）、wrench/twist 换参考点 | 完成 + 测试 |
| `config.hpp` | `SimConfig` 与 YAML 读取、`--set a.b=v` 覆盖、名义/真实基座位姿 | 完成 |
| `mujoco_utils.hpp` | 用 mjSpec 按基座位姿编译场景；按名字查 id/地址并检查模型结构 | 完成 |
| `sim_env.hpp` | **仿真 plant**：step、状态读取、F/T、weld 约束力、标定误差、扰动、weld relpose 初始化 | 完成 + 测试 |
| `robot_model.hpp` | **控制器侧模型抽象接口**（FK、J、J̇q̇、M、h、g、限位、物体参数） | 接口 |
| `mujoco_model.hpp` | 用第二份 mjModel（名义基座）实现 `RobotModel` | 完成 + 测试 |
| `controller.hpp` | `Controller` 基类：`compute(state, t) → (τ_l, τ_r)` | 接口 |
| `baseline_controllers.hpp` | `GravityCompJointPD`：重力补偿 + 关节 PD（只用于验证环境） | 完成 |
| `object_trajectory.hpp` | 物体参考轨迹 hold / min_jerk / sine（任务定义，两类控制器共用） | 完成 |
| `coop_kinematics.hpp` | 抓取矩阵、J_a/J_r、末端参考、负载分配、内力测量、日志钩子 | **桩（你来写）** |
| `coop_controller.hpp` | `CoopController`：目前只输出重力补偿 | **桩（你来写）** |
| `logger.hpp` | CSV 日志（139 列）、运行目录 | 完成 |
| `viewer.hpp` | GLFW 窗口、鼠标相机、固定相机（cam_iso / cam_front）、离屏截图 PNG/PPM（单独的库，核心库不依赖 GLFW） | 完成 |

| 测试 | 验证内容 |
|---|---|
| `test_math_utils.cpp` | skew、RPY、旋转向量误差、换参考点保持功率 |
| `test_jacobian.cpp` | J 与有限差分一致（**线速度在前、世界系、ee_site 参考点**）、J̇q̇、M 对称正定、h(q,0)=g、C 二次齐次、控制器模型 = plant（无误差时） |
| `test_gravity_comp.cpp` | 无箱子时纯重力补偿下机械臂静止（及 τ=0 的对照组）；基线 PD 持住箱子；协同控制器桩可运行 |
| `test_sensors_weld.cpp` | weld wrench = 关节侧广义力（牛顿第三定律）；腕部 F/T 与 weld 一致；复现 MuJoCo weld 力矩翻倍问题；合力与重力/扰动平衡；箱子 twist 约定 |
| `test_calibration.cpp` | plant 基座 = 名义 ∘ 误差；current 模式无预载；nominal 模式产生内力；过渡无冲击 |
| `test_scene_geometry.cpp` | TCP 与抓取点重合；指面与箱子间隙 1 mm、无穿透；两臂所有几何体（含视觉网格）不进入箱子；初始无接触；工作台无碰撞、箱子在台面上方 2–3 cm；夹爪质量保留、手指无关节；固定相机存在 |
| `test_coop_template.cpp` | **模板**：G·(I−G⁺G)h = 0、对挤是纯内力、分配保持 Gh=w_o、理想闭链下 J_r·dq = 0 等；桩返回 NaN 时自动 SKIP |
| `test_no_alloc.cpp` | 替换 malloc，验证控制环（控制器 + 仿真步 + 日志）零堆分配 |

## 2. 数据流（一个 1 ms 控制周期）

```
                        ┌──────────────── SimEnv（plant，真实基座位姿，含标定误差）────────────────┐
                        │  mjModel/mjData #1 ── weld ×2 ── 箱子 free joint ── xfrc_applied(扰动)    │
                        └───────┬───────────────────────────────────────────────────▲──────────────┘
          env.state()           │ DualArmState: q, dq（t）; ft_raw/ft_ee_world/       │ env.step(τ_l, τ_r)
                                │ weld_wrench（t−dt）; object; ee_pose（真值）         │  饱和 → mj_step2 → mj_step1
                                ▼                                                     │
┌────────────────────────── Controller::compute(state, t) ──────────────────────────┴─┐
│  model->update(state)  ──►  RobotModel（MujocoRobotModel：mjModel/mjData #2，名义基座） │
│                            eePose, jacobian, J̇q̇, M, h, g, objectParams            │
│  trajectory->evaluate(t) ─► ObjectReference（物体参考）                              │
│  coop::*（G, J_a, J_r, 分配, 内力）                                                  │
└───────────────────────────────────────────────────────────────────────────────────────┘
                                │
                                ▼  env.lastStep()（所有字段对齐到 t）
   run_sim: obj_err = poseError(ref, box)；int_* = coop::internalWrenchForLogging(rec, model)
            → CsvLogger → logs/<时间戳>_<控制器>[_calib]/log.csv + config.yaml + run_info.txt
            → scripts/plot_log.py
```

要点：

- **两份 mjModel**：plant（`SimEnv`）用真实基座位姿，控制器（`MujocoRobotModel`）用名义位姿；
  两者不共享任何数据。标定误差实验就靠这一点。
- `DualArmState` 同时带“测量量”和“仿真真值”。控制器只该用测量量（q、dq、F/T、视情况 object），
  运动学一律通过 `RobotModel`。真值（`ee_pose`、`weld_wrench`）用于评估。
- 力测量有 1 个周期延迟（真实系统也是）；日志行完全时间对齐。
- 控制环内无堆分配（`test_no_alloc` 保证）；你新增的代码也请只用定长 Eigen 类型，
  需要的缓冲区在构造函数 / `reset()` 里分配。

## 3. 从哪里开始写（建议顺序）

每一步都有对应的测试模板或可观察的日志列，写完一个验证一个。

1. **`coop::graspMatrixArm` / `graspMatrix`**（式 3.2、4.1）
   → `CoopTemplate.GraspMatrixMapsWrenchToObjectCenter`、`SqueezingIsPureInternalForce` 自动生效。
2. **`coop::weightedPseudoInverse` / `internalWrench` / `internalWrenchCoordinates`**（式 5.1–5.2、4.4、4.7）
   → `InternalWrenchLiesInNullSpaceOfG`。
3. **`coop::internalWrenchForLogging`**（头文件里有 4 步建议）
   → 之后每次 `run_sim` 的 `int_*` 列就有值了，`plot_log.py` 会自动画 `internal_force.png`。
   **先用基线跑两遍**（无误差 / `--calib-error --set weld.init_mode=nominal`）：两次的内力之差应为
   十牛量级（weld 真值中 x 向约 13 N、绕 y 约 3 N·m，见 cooperative_control.md §1）；再用 `*_weld_*` 真值
   离线算一遍同样的投影做对照，确认你的内力口径正确。注意无误差时也有约 7.5 N 的“载荷变形内力”。
4. **`coop::absoluteJacobian` / `relativeJacobian`**（式 3.6–3.8）
   → `RelativeJacobianVanishesOnIdealClosedChain`。
5. **`coop::eeReferencesFromObject`**（式 3.9–3.11）
   → `EeReferencesConsistentWithObjectReference`。
   有了它可以先写**基线**：两臂独立笛卡尔阻抗，各自跟踪自己的末端参考（建议新建
   `IndependentImpedanceController`，放在 `baseline_controllers.hpp`）。
6. **`coop::distributeObjectWrench`**（式 4.3）→ `DistributionReproducesObjectWrench`。
7. **`CoopController::compute`**（§6 结构，式 6.1–6.5）：先物体阻抗 + 平均分配，再加内力调节，
   最后加零空间项。每加一项就用 `min_jerk` 轨迹 + 标定误差对比一次基线。
8. **力矩级 QP**（式 6.6）：打开 `-DDUAL_ARM_WITH_OSQP=ON`（注意 README 里关于 OSQP 版本的说明），
   先把第 7 步的解析解写成等价 QP 验证一致，再加不等式约束。

## 4. 常见扩展点

- **新控制器**：继承 `Controller`，在 `apps/run_sim.cpp::makeController` 里注册一个名字，
  参数加到 `config.hpp` + `default.yaml`。
- **新日志列**：`logger.hpp` 的 `LogRow` 加字段，`logger.cpp` 的 `columnNames()` 与 `write()`
  按相同顺序追加（有计数检查）。
- **换 Pinocchio 模型**：实现 `RobotModel`（`PinocchioRobotModel`），注意保持雅可比约定；
  `test_jacobian.cpp` 可以直接拿来对拍两个实现。
- **ROS2 封装**：`Controller`、`RobotModel`、`types.hpp` 不依赖 MuJoCo；ROS2 节点只需把
  `/joint_states`、F/T 话题填进 `DualArmState`，调用 `compute()`，发布力矩命令。`SimEnv` 可以
  原样作为一个仿真节点的后端。
- **传感器噪声 / 延迟**：在 `SimEnv::readForces` 里加（当前没有噪声）。
