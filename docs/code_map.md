# 代码地图

本文件说明运行时数据链路、主要模块职责，以及修改控制算法时需要同步验证的位置。

## 1. 目录结构

```text
dual_arm_ws/
├── apps/run_sim.cpp              命令行、控制器工厂、1 kHz 仿真循环、终端诊断
├── config/default.yaml           公共参数
├── config/scenes/*.yaml          场景任务、航点、障碍组和控制器覆盖
├── include/dual_arm/             公共接口与定长数据类型
├── src/                          控制器、QP、碰撞、轨迹、日志和 MuJoCo 后端
├── models/                       双 Panda 工作站、任务物体和第三方 mesh
├── tests/                        单元、集成、几何与零分配测试
├── scripts/plot_log.py           CSV 对比绘图
├── scripts/run_portfolio_demo.sh 一键 A/B 作品集实验
├── scripts/run_experiment_matrix.py 消融与不确定性实验
├── scripts/run_acceptance_suite.py  核心场景验收矩阵
├── scripts/run_robustness_sweep.py  固定种子的随机鲁棒性扫参
├── scripts/run_realtime_benchmark.py 持续 1 kHz 耗时测量
└── tools/                        标定与场景设计辅助程序
```

## 2. 核心模块

| 模块 | 作用 | 主要验证 |
|---|---|---|
| `types.hpp` | `Pose`、twist/wrench、双臂状态和定长矩阵别名 | 全部控制器共享 |
| `math_utils.*` | skew、姿态误差、twist/wrench 换参考点 | `test_math_utils.cpp` |
| `sim_env.*` | MuJoCo plant、传感器、weld、扰动和标定误差 | gravity/sensor/calibration 测试 |
| `robot_model.hpp` | 控制器侧 FK、J、`Jdot*qdot`、M、h、g 抽象 | `test_jacobian.cpp` |
| `mujoco_model.*` | 独立名义 mjModel 的 `RobotModel` 实现 | plant/model 对拍 |
| `coop_kinematics.*` | G、绝对/相对 Jacobian、wrench 分配、内力投影 | `test_coop_template.cpp` |
| `coop_controller.*` | 物体阻抗、负载分配、内力反馈、零空间控制 | `test_gravity_comp.cpp` |
| `asym_coop_controller.*` | 预紧—转角—扭矩—保持装配状态机 | assembly tests |
| `torque_qp.*` | 固定尺寸 ADMM QP 与关节/闭链/碰撞约束 | `test_torque_qp.cpp` |
| `qp_coop_controller.*` | 对称协作 QP 与在线 reference governor | slot/slot_avoid tests |
| `qp_asym_coop_controller.*` | 非对称装配 QP 包装器 | assembly QP test |
| `collision_model.*` | geom pair 距离、关节梯度及并行查询 | `test_collision_model.cpp` |
| `object_trajectory.*` | hold、min-jerk、sine、waypoint 轨迹 | `test_trajectory.cpp` |
| `logger.*` | 固定列 CSV 和场景扩展列 | `test_no_alloc.cpp` |

## 3. 一个控制周期的数据流

```text
SimEnv::state()
  │  q, dq, wrist F/T, object pose/twist
  ▼
Controller::compute(state, t)
  ├─ RobotModel::update()       -> M, h, g, J, Jdot*dq, nominal FK
  ├─ ObjectTrajectory          -> raw object reference
  ├─ CollisionModel::query()   -> distance and ddq-space gradients
  ├─ Reference governor        -> governed object reference
  ├─ CoopController            -> nominal tau_left/tau_right
  └─ JointSafetyTorqueQp       -> constrained tau_left/tau_right
  │
  ▼
SimEnv::step(tau_left, tau_right)
  │
  ├─ CsvLogger
  ├─ SceneMonitor
  └─ terminal / viewer diagnostics
```

关键约定：

- twist 为 `[v; omega]`，wrench 为 `[f; m]`，均在世界系表达；
- 抓取向量 `r_i = p_object - p_grasp_i`；
- plant 和 controller 使用两份独立 `mjModel`，只有 plant 注入标定误差；
- 控制器使用测量量，`ee_pose` 和 `weld_wrench` 真值主要用于验证；
- 控制路径使用定长 Eigen 类型，不允许运行时 heap allocation。

## 4. 控制器层次

```text
ObjectTrajectory
      │
      ▼
ReferenceGovernor        任务参考层：必要时冻结虚拟时间并增加避障偏移
      │
      ▼
CoopController           名义任务层：物体阻抗、负载分配、内力和零空间
      │
      ▼
JointSafetyTorqueQp      关节安全层：动力学、闭链、关节、力矩和碰撞约束
      │
      ▼
Actuator torque
```

`qp_coop` 复用 `CoopController::computeWithReference()`，因此 reference governor 不复制名义控制律。
QP 负责局部安全约束，governor 负责生成能够绕过障碍的短期参考，二者不能互相替代。

## 5. 修改时的同步位置

### 新增控制器

1. 在 `include/dual_arm/` 和 `src/` 实现 `Controller`；
2. 在 `config.hpp`、`config.cpp`、`default.yaml` 添加参数；
3. 在 `apps/run_sim.cpp::makeController()` 注册名称；
4. 增加任务完成测试与 `test_no_alloc.cpp` 路径；
5. 在 README 的控制器/场景表中说明使用范围。

### 新增日志列

1. 在 `LogRow` 增加字段；
2. 按相同顺序修改 `CsvLogger::columnNames()` 与 `write()`；
3. 更新 `plot_log.py`；
4. 运行 `NoAlloc.CoopControlLoopDoesNotAllocate`。

### 新增碰撞组

1. 在场景 YAML 的 `obstacles` 中声明 body/geom group；
2. 设置任务相关 `safe_distance`；
3. 用 `CollisionModelTest` 验证距离和有限差分梯度；
4. 增加端到端最小距离断言。

## 6. 推荐验证命令

```bash
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j"$(nproc)"
ctest --test-dir build --output-on-failure

./scripts/run_portfolio_demo.sh --skip-tests
```

理论推导见 [cooperative_control.md](cooperative_control.md)。
