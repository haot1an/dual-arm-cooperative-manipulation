# 07 测试、实验设计与结果表达

## 1. 测试金字塔

本项目需要四层证据：

1. 数学单元测试：矩阵恒等式、维度、功率、投影。
2. 数值验证：有限差分 Jacobian、`Jdot*qdot`、距离梯度。
3. 控制器集成测试：闭链保持、插槽、装配状态机、QP 约束。
4. 任务实验：消融、扰动、鲁棒性 sweep、持续高频 benchmark。

只有视频没有指标，无法证明控制器正确；只有单元测试没有端到端任务，也无法证明系统行为。

源码入口：

- `tests/`
- `config/acceptance.json`
- `scripts/run_metrics.py`
- `scripts/check_acceptance.py`
- `scripts/run_acceptance_suite.py`
- `scripts/run_experiment_matrix.py`
- `scripts/run_robustness_sweep.py`

## 2. 好测试的结构

一个高价值测试应明确：

- Arrange：设置哪个场景、初始状态、误差或扰动。
- Act：调用哪个函数或运行多少控制周期。
- Assert：验证物理不变量、数值阈值或任务结果。
- Failure meaning：失败时指向哪个模块，而不是只得到“仿真跑偏”。

优先验证不变量：

```text
skew(a)b = a×b
power before shift = power after shift
G * internal_wrench = 0
P_perp * S = 0
M = M^T and M > 0
distance analytic gradient ≈ finite difference
torque within limits
closed-chain acceleration residual ≈ 0
```

## 3. 有限差分测试

中心差分：

$$
f'(x)\approx\frac{f(x+\epsilon)-f(x-\epsilon)}{2\epsilon}
$$

选择 `epsilon` 时：过大会有截断误差，过小会被浮点和仿真碰撞求解噪声淹没。应该测试多个数量级，而不是只找一个刚好通过的值。

**【必须手写】** Jacobian 和距离梯度各一个中心差分测试。失败时输出关节编号、解析值、数值值和相对误差。

## 4. 场景验收指标

每个场景必须有任务相关指标：

- `lift`：终态位姿、物体与放置面距离、机械臂与台阶距离。
- `slot`：滚转角度、插入终态、槽框最小距离。
- `slot_avoid`：障碍距离、完整 governor 状态序列、插入终态。
- `assembly`：预紧力、拧紧力矩、螺钉角度、工件漂移、最终 phase。

所有场景共同检查：力矩饱和、QP status、控制耗时和数值发散。

验收阈值应来自任务公差、安全要求或数值精度，不应在看到结果后随意放宽。

## 5. 消融实验

消融不是多跑几组参数，而是一次只移除一个机制，并让结果回答因果问题：

- 普通协作：原始参考是否碰障碍？
- 仅 QP：局部安全层能否完成被阻挡任务？
- 仅 governor：参考可行化是否足够保证安全裕量？
- 完整方案：两者组合是否同时完成任务并满足约束？

面试时不要只说“完整方案最好”，要说每个失败组为什么失败，以及它证明哪个模块不可替代。

## 6. 鲁棒性与 Monte Carlo

必须在实验前声明：

- 随机参数和分布。
- 误差范围。
- 随机种子。
- 试验数。
- 成功判据。
- 最坏样本指标。

固定 seed 用于回归；多个 seed 和更多 trial 用于扩大覆盖。12/12 只能说明声明包络中的 12 个确定性样本通过，不能声称真实成功率是 100%。

## 7. 性能测试

性能测试必须使用 Release 构建，记录硬件和系统。Debug 构建结果不能用于简历性能数字。

需要避免：

- 只报告最快一次。
- 只报告 mean。
- 把离线仿真倍速当成控制周期耗时。
- 在不同机器或不同日志 decimation 下直接对比。
- 把普通 Linux 结果写成 hard real-time。

## 8. 故障注入练习

建议在学习分支逐个制造以下错误，并记录哪个测试最先发现：

1. 把 `r_i` 改成相反方向。
2. 把一个 F/T wrench 符号反转。
3. 删除 `Jdot*qdot`。
4. 把碰撞法向反向。
5. 关闭 governor 的 virtual time 暂停。
6. 把 torque limit 增加十倍。
7. 在控制循环中加入 `std::vector` 增长。
8. 把标定误差同时加到 plant 和 controller model。

完成标准：能在查看实现前，根据失败测试和日志指出最可能的错误类别。

## 9. 完整回归命令

```bash
cd /home/tt/dual_arm_ws

cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j"$(nproc)"
ctest --test-dir build --output-on-failure

./scripts/run_acceptance_suite.py --skip-build
./scripts/run_experiment_matrix.py --skip-build
./scripts/run_robustness_sweep.py --trials 12 --seed 20260923 --skip-build
./scripts/run_realtime_benchmark.py --duration 60 --skip-build
```

## 10. 如何表达结果

推荐结构：

```text
问题：直线路径被障碍阻断。
方法：reference governor 负责参考可行化，torque QP 负责局部约束安全。
证据：QP-only 安全停止但终态误差 114.53 mm；完整方案保持 10 mm 裕量并完成任务。
边界：绕行方向仍由配置给定，尚不是通用 3D planner。
```

这种表达比“使用 QP 提升了性能”更可信。

## 11. 本章口试题

1. 单元测试通过为什么仍然可能在任务中失败？
2. 有限差分 epsilon 应如何选择？
3. 消融实验和调参对比有什么区别？
4. 12/12 通过可以怎样表述，不能怎样表述？
5. 一个验收阈值应该从哪里来？
