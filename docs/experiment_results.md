# 第二阶段实验报告：消融、鲁棒性与高频控制

本文记录任务级验收、避障模块消融、不确定性注入和持续 1 kHz 计算基准。所有结果均可由仓库脚本重新生成，不依赖本文中的手工计算。

## 1. 验证环境

- 日期：2026-09-23
- 构建：CMake Release，GCC 13.3，C++17
- 仿真：MuJoCo 3.12.0，步长 1 ms
- 主机：Intel Core i7-12800HX，24 logical CPUs
- 系统：普通 Linux 用户态，非 PREEMPT_RT
- 场景验收和消融实验关闭物理 contact，用有符号几何距离直接暴露穿透

完整严格构建命令：

```bash
cmake -S . -B build-ci-local -G Ninja \
  -DCMAKE_BUILD_TYPE=Release \
  -DDUAL_ARM_BUILD_VIEWER=OFF \
  -DDUAL_ARM_WERROR=ON \
  -Dmujoco_DIR=/path/to/mujoco/lib/cmake/mujoco
cmake --build build-ci-local --parallel 2
ctest --test-dir build-ci-local --output-on-failure
```

结果为 `98/98` 测试通过，其中包括控制循环无动态内存分配、有限差分 Jacobian/距离梯度、闭链约束和任务级控制器测试。

## 2. 避障消融实验

运行命令：

```bash
./scripts/run_experiment_matrix.py
```

四个消融组使用同一条原始直线路径和相同的 `slot_avoid` 场景。`governor_without_collision_damper` 仍保留闭链等式、关节安全和力矩约束 QP，只关闭碰撞 acceleration-damper 不等式，因此可以独立观察 reference governor 的贡献。

| 配置 | `object~barrier` 最小距离 | 终态位置误差 | governor 触发 | P99 | 任务结果 |
|---|---:|---:|---:|---:|---|
| `straight_coop` | -30.12 mm | 0.00 mm | 0 | 13.3 us | 失败：穿透障碍 |
| `qp_only` | 9.81 mm | 114.53 mm | 0 | 185.8 us | 失败：停在障碍前 |
| `governor_without_collision_damper` | 7.02 mm | 0.00 mm | 1 | 240.7 us | 失败：安全裕量不足 |
| `full_qp_governor` | 10.00 mm | 0.00 mm | 1 | 237.8 us | 通过 |

结论：collision damper 是局部安全约束，只能限制朝障碍运动的加速度，不能产生跨越障碍的新路径；reference governor 能在线修改物体参考并完成 `LIFT -> CROSS -> DESCEND`，但离散控制、模型误差和路径切换附近仍需要 QP 距离约束兜底。完整方案同时具备任务可达性和约束安全性。

## 3. 鲁棒性实验

鲁棒性组保持完整 QP + governor，分别加入控制器/plant 基座位姿不一致、持续 1 s 的世界系外部 wrench（4 N 横向力和 0.2 N·m 转矩），以及两者组合。

| 配置 | 障碍最小距离 | 终态位置误差 | governor 触发 | P99 | 结果 |
|---|---:|---:|---:|---:|---|
| 基座标定误差 | 10.00 mm | 0.02 mm | 1 | 321.7 us | 通过 |
| 外部 wrench | 10.00 mm | 0.00 mm | 1 | 257.1 us | 通过 |
| 标定误差 + 外部 wrench | 10.00 mm | 0.02 mm | 1 | 248.1 us | 通过 |

这些结果验证的是当前配置附近的确定性仿真鲁棒性，不等价于真实机器人上的统计可靠性。进一步迁移到实机时，应增加多随机种子 Monte Carlo、传感器噪声、时延、摩擦/质量摄动及抓取柔顺性。

## 4. 核心场景验收

运行命令：

```bash
./scripts/run_acceptance_suite.py
```

阈值定义在 `config/acceptance.json`，脚本返回非零状态码表示验收失败，适合本地回归或 CI 扩展。

| 场景 | 终态位置误差 | 全局最小距离 | P99 | 关键任务指标 | 结果 |
|---|---:|---:|---:|---|---|
| `lift` | 0.0002 mm | 0.785 mm | 158.0 us | 物体安全放置，机械臂距台阶 > 12 mm | 通过 |
| `slot` | 0.100 mm | 0.896 mm | 211.3 us | 滚转后完成窄槽插入 | 通过 |
| `slot_avoid` | 0.0001 mm | 0.896 mm | 241.1 us | 障碍裕量 10.00 mm，完整状态序列 | 通过 |
| `assembly` | 0.949 mm | 20.305 mm | 178.8 us | 4.99 N 预紧、0.97 N·m 拧紧、最终 `HOLD` | 通过 |

这里的“全局最小距离”包括任务允许接近的目标表面。例如 `lift` 中物体需要放到台阶上，因此该 pair 使用 1 mm 任务专属安全距离；机械臂与台阶仍保持 10 mm 全局安全距离。

## 5. 持续 1 kHz 计算基准

运行命令：

```bash
./scripts/run_realtime_benchmark.py --duration 60
```

前 2 s 作为 warm-up，后 58 s 共统计 58,000 个控制周期：

| 指标 | 结果 |
|---|---:|
| mean | 177.0 us |
| P99 | 284.2 us |
| P99.9 | 396.9 us |
| maximum | 3869.0 us |
| 超过 1 ms | 15 / 58,000（0.0259%） |

P99.9 小于 1 ms，说明算法在本机具有 1 kHz 的典型计算预算；最大值仍达到 3.869 ms，说明普通 Linux 调度存在长尾。本项目只能表述为 **soft real-time 1 kHz benchmark**，不能据此宣称 hard real-time。实机部署需要 PREEMPT_RT/实时控制器、CPU 隔离、线程优先级、锁页以及硬件在环 worst-case latency 测量。

## 6. 指标产物

每个脚本会在被 `.gitignore` 排除的 `artifacts/` 下生成带时间戳的结果：

```text
artifacts/experiment_matrix/<timestamp>/
  results.json
  summary.md
  plots/

artifacts/acceptance/<timestamp>/
  results.json
  summary.md

artifacts/realtime/<timestamp>/
  benchmark.json
  summary.md
```

JSON 用于机器判定，Markdown 和图用于人工审阅。日志路径写入 JSON，可继续使用 `scripts/plot_log.py` 复核原始时序。
