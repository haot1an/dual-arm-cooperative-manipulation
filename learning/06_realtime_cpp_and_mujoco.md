# 06 MuJoCo、C++ 与 1 kHz 控制工程

## 1. 1 kHz 的真正含义

控制周期为 1 ms，不表示只要平均耗时小于 1 ms 就够了。需要分别看：

- mean：平均计算负载。
- P99/P99.9：绝大部分周期的时延。
- maximum：调度、缺页、锁竞争等长尾。
- deadline miss count/rate：超过 1 ms 的次数和比例。
- 是否包含传感器、通信、控制、日志等完整路径。

项目本机结果 P99.9 为 396.9 us，但 maximum 为 3869 us，因此只能叫 soft real-time benchmark。

源码入口：

- `apps/run_sim.cpp`
- `include/dual_arm/controller.hpp`
- `include/dual_arm/robot_model.hpp`
- `include/dual_arm/sim_env.hpp`
- `src/sim_env.cpp`
- `tests/test_no_alloc.cpp`
- `scripts/run_realtime_benchmark.py`

## 2. 控制循环正确顺序

概念循环：

```cpp
while (running) {
  state = readSensors();
  model.update(state);
  reference = trajectory.evaluate(state.t);
  torque = controller.compute(state, state.t);
  torque = safetyCheckAndClamp(torque);
  writeActuators(torque);
  logNonBlocking(state, torque);
  waitUntilNextPeriod();
}
```

真实机器人还必须有：状态时间戳、命令超时、watchdog、模式切换、急停、通信错误和传感器有效性检查。

## 3. 控制周期内禁止的操作

- `new/delete`、容量可能增长的 `std::vector::push_back`。
- 动态尺寸 Eigen 矩阵临时分配。
- 文件打开关闭、同步磁盘写入。
- `std::cout`/格式化大量字符串。
- 锁等待、条件变量和不可控系统调用。
- 第一次调用才初始化的库或 lazy allocation。
- 无界迭代求解。

推荐做法：

- 使用固定尺寸 Eigen 类型。
- 构造阶段分配所有缓冲区。
- `reserve()` 后确认最大容量不再增长。
- 使用 `.noalias()` 减少不必要临时量。
- 求解器设置固定最大迭代数。
- 控制线程只写预分配 ring buffer，由非实时线程落盘。

## 4. Eigen 需要掌握的工程点

- 固定尺寸矩阵通常存储在栈或对象内部。
- 矩阵相乘仍可能因表达式和 decomposition 产生临时量。
- 对齐问题：包含固定大小、需特殊对齐 Eigen 成员的类型在容器中要注意 allocator；当前编译器/Eigen 配置通常会自动处理，但必须认识这一风险。
- 不要写 `.inverse()`；使用 `ldlt().solve()`、`llt().solve()` 或合适 decomposition。
- `Map` 可在不复制的情况下访问 MuJoCo 数组，但维度和内存布局必须完全匹配。
- 控制器输入出现 NaN 时，应在进入分解和执行器前阻断。

**【必须手写】** 一个只使用固定尺寸矩阵的 `compute()`，并让 malloc 插桩测试连续调用期间计数为 0。

## 5. MuJoCo 数据流

需要理解两个核心对象：

- `mjModel`：拓扑、质量、几何、执行器、约束和仿真选项，通常初始化后只读。
- `mjData`：当前 `qpos/qvel`、动力学缓存、传感器、contact、constraint force 等运行状态。

典型流程：

```text
写 qpos/qvel 或 ctrl
→ mj_forward / mj_step
→ 读取 site pose、Jacobian、sensor、contact、constraint force
```

必须知道：

- `qpos` 与 `qvel` 维度可能不同，自由体 quaternion 占 4 个位置量但只有 3 个角速度量。
- `mj_step` 会推进积分；`mj_forward` 只刷新派生量。
- Site、body 原点和 geom 最近点不是同一个参考点。
- 约束求解器参数 `solref/solimp` 会影响闭链刚度和数值稳定性。
- Contact 关闭不代表 weld/关节约束关闭。

## 6. 仿真 plant 与 controller model 分离

真实控制中控制器永远只有模型估计，不应直接读取仿真真值作为控制量。因此项目将：

- Plant：带注入的真实基座误差、约束和扰动。
- Controller model：名义基座、机器人动力学与运动学。
- Monitor/logger：可以读取真值用于离线评估。

面试中要主动说明哪些信号是控制器可用的，哪些仅用于仿真评估。否则容易出现“用 ground truth 作弊”的质疑。

## 7. 日志与控制路径

日志必须支持复现实验，但不应破坏实时性。需要记录：

- 时间戳、状态、参考和误差。
- 输出力矩与饱和状态。
- F/T、weld、内部 wrench。
- QP status、迭代、约束违反、slack。
- 距离与 governor phase。
- 控制耗时。

新增日志列时要同步修改数据结构、列名、写入顺序和绘图脚本，否则会产生“列名与数据错位”的隐蔽错误。

## 8. Soft real-time 到实机还缺什么

- PREEMPT_RT 或机器人厂商实时控制器。
- CPU 隔离、线程 affinity、实时调度优先级。
- `mlockall`/预触页，避免 page fault。
- 禁用频率动态变化或进行可控设置。
- 传感器与执行器时钟同步。
- 网络周期与丢包统计。
- Watchdog、急停和安全 PLC/硬件限位。
- Hardware-in-the-loop worst-case latency。

## 9. 验证命令

```bash
cd /home/tt/dual_arm_ws
./build/test_no_alloc
./scripts/run_realtime_benchmark.py --duration 60 --skip-build
```

不要只贴 mean。报告至少包含 mean、P99、P99.9、maximum 和 deadline misses。

## 10. 本章口试题

1. 为什么零动态分配不等于 hard real-time？
2. 为什么最大耗时比平均耗时更重要？
3. `mj_forward` 和 `mj_step` 有什么区别？
4. 控制器可以读取仿真 object ground truth 吗？实机如何替代？
5. 日志应该如何从实时线程传给写盘线程？
