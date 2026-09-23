# 00 项目架构、轨迹与场景建模

## 1. 先建立完整数据流

运行入口是 `apps/run_sim.cpp`。一次控制周期的主要链路：

```text
YAML + scene YAML + CLI overrides
→ SimConfig / SceneSpec
→ MuJoCo plant + nominal RobotModel + CollisionModel
→ DualArmState
→ ObjectTrajectory
→ Controller::compute
→ left/right joint torque
→ SimEnv::step
→ monitor + logger + viewer
```

你必须能区分三个模型：

- Plant：MuJoCo 中真正推进的系统，可包含标定误差和扰动。
- Controller model：控制器使用的名义运动学与动力学。
- Collision/monitor model：用于控制器距离查询或离线真值评估。

面试时要明确哪些数据真实机器人可以测量，哪些只是仿真评估真值。

## 2. 核心接口职责

| 接口 | 责任 |
|---|---|
| `SimEnv` | 推进 plant、读取状态/传感器、施加扰动 |
| `RobotModel` | 提供 `J`、`Jdot*dq`、`M`、`bias`、`gravity`、末端状态 |
| `ObjectTrajectory` | 根据时间产生物体 pose/twist/acceleration 参考 |
| `Controller` | 将状态与参考转换为双臂关节力矩 |
| `CollisionModel` | 有符号距离、最近点、法向和关节梯度 |
| `SceneMonitor` | 统一计算日志和任务指标，不参与控制 |
| `CsvLogger` | 保存可复现实验数据 |

**【必须理解】** 为什么 `Controller` 依赖抽象 `RobotModel`，而不直接在控制律中调用 MuJoCo API：这样控制律与仿真后端解耦，未来可替换为 Pinocchio、厂商模型或实机状态接口。

## 3. 配置合并顺序

```text
config/default.yaml
→ config/scenes/<scene>.yaml overrides
→ CLI --set key.path=value
```

后加载项覆盖前面的值。每次运行把最终配置写入日志目录，是为了回答“这组结果到底用了哪些参数”。

需要掌握：

- YAML 标量、序列和 map 的类型差异。
- CLI 中包含方括号和花括号时要用单引号，避免 shell 展开。
- 新增配置字段必须同时修改 config struct、parser、默认配置、合法性校验和测试。
- 不能默默接受拼错的 key；当前 override 会提示不存在的路径。

## 4. 轨迹为什么必须输出 pose、twist、acceleration

控制器不只需要位置目标：

- Impedance 需要 pose 和 twist reference。
- 动力学前馈需要 acceleration reference。
- QP/governor 需要连续、限速的参考，避免力矩跳变。

项目轨迹类型：

- `hold`：保持初始状态。
- `min_jerk`：端点速度和加速度为零的五次多项式。
- `sine`：周期激励，用于频率响应或跟踪测试。
- `waypoints`：位置插值、姿态 slerp 和梯形速度时间参数化。

最小 jerk 标量：

$$
s(\tau)=10\tau^3-15\tau^4+6\tau^5,\qquad \tau\in[0,1]
$$

需要能求出 `s_dot`、`s_ddot`，并说明端点为什么连续。

**【必须手写】** 一维 minimum-jerk 的 position/velocity/acceleration；再扩展为三维位置。姿态部分至少能解释为什么使用 slerp 而不是线性插值四元数。

## 5. Waypoint 时间参数化

任务航点只定义几何路径，不自动满足速度和加速度限制。每段持续时间取决于：

- 平移距离与 `v_max/a_max`。
- 旋转角度与 `w_max/alpha_max`。
- 平移和旋转需要使用相同或协调的段时间。
- 航点 dwell。

必须理解梯形与三角形速度剖面的切换条件：距离不足以达到最大速度时，轨迹退化成三角形速度。

**【必须手写】** 一维梯形速度 profile，测试端点、最大速度、最大加速度和短距离三角形情况。

## 6. MuJoCo 场景与抓取建模

MJCF 中需要区分：

- `body`：刚体层级和惯性。
- `joint`：允许的相对自由度。
- `geom`：碰撞与可视几何。
- `site`：TCP、抓取点、传感器或参考标记。
- `actuator`：关节力矩输入。
- `sensor`：force/torque 等测量。
- `equality/weld`：刚性相对位姿约束。

当前项目从“抓取已经建立”开始：手与物体之间通过 `weld` 建立刚性约束。模型中显示的手指是固定开度，不执行接近和闭合控制。

这意味着当前项目没有建模：

- 手指位置/力控制。
- 摩擦锥和抓取稳定性。
- 接触点变化。
- 滑移和重抓取。
- 抓取检测状态机。

面试中必须主动说出这个边界。加入真实夹爪后，需要新增接触模式切换、摩擦不确定性、抓取 wrench cone、滑移检测和抓取失败恢复。

## 7. 场景安全距离

全局安全距离不一定适合所有 pair。例如放置任务中物体必须接近目标表面，但机械臂仍应保持更大距离。因此 `ObstaclePairSpec` 支持 pair-specific `safe_distance`。

必须理解：任务允许接触、希望避免接触和绝不允许接触是三种不同语义，不能用一个全局阈值混在一起。

## 8. Controller factory 与扩展流程

新增控制器时通常需要：

1. 在 `include/dual_arm/` 定义接口。
2. 在 `src/` 实现。
3. 加入 `CMakeLists.txt`。
4. 在 config struct/YAML 中增加参数。
5. 在 `run_sim` controller factory 注册名称。
6. 增加单元、集成和 no-allocation 测试。
7. 增加日志诊断与验收规则。
8. 更新 README 和 code map。

**【必须能完成】** 新增一个简单的 `hold_cartesian` 教学控制器并走完以上流程，但不要为了练习合并到主分支。

## 9. 构建、测试与 CI

CMake 负责核心库、仿真程序、viewer 和测试。需要理解：

- `PUBLIC/PRIVATE` include 与 link 传播。
- Release/Debug 的差别。
- Viewer 为什么可以在 CI 中关闭。
- `-Werror` 为什么适合 CI，但第三方库警告要谨慎处理。
- GitHub Actions 如何下载 MuJoCo、配置、编译和运行 CTest。

验证：

```bash
cd /home/tt/dual_arm_ws
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j"$(nproc)"
ctest --test-dir build --output-on-failure
```

## 10. 本章口试题

1. Plant model 和 controller model 为什么必须分离？
2. Object trajectory 为什么需要 acceleration 输出？
3. 当前夹爪是真实接触抓取还是 weld 抽象？
4. 为什么放置面不能直接使用与机械臂相同的安全距离？
5. 如何添加一个新控制器并确保它真的进入回归体系？
