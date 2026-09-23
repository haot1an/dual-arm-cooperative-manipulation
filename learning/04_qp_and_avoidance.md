# 04 力矩级 QP、自主避障与 reference governor

## 1. QP 在控制链路中的位置

本项目不是用 QP 重新生成所有行为，而是：

```text
协同控制器生成 nominal torque
→ QP 在尽量接近 nominal torque 的同时满足约束
→ 输出最终双臂 torque
```

源码入口：

- `include/dual_arm/torque_qp.hpp`
- `src/torque_qp.cpp`
- `include/dual_arm/qp_coop_controller.hpp`
- `src/qp_coop_controller.cpp`
- `include/dual_arm/collision_model.hpp`
- `src/collision_model.cpp`
- `tests/test_torque_qp.cpp`
- `tests/test_collision_model.cpp`

## 2. 从 box QP 开始

最简单的力矩 QP：

$$
\min_\tau \frac12w_t\|\tau-\tau_d\|^2+
\frac12w_s\|\tau-\tau_{prev}\|^2
$$

$$
\tau_{min}\le\tau\le\tau_{max}
$$

Hessian 是正对角阵，因此可以先算无约束最优值，再逐元素投影。必须理解这个 baseline，因为它用于检查：目标函数、力矩边界、平滑项和 invalid-input fallback。

**【必须手写】** `BoxTorqueQp::solve()`，包括 NaN、上下界颠倒和 previous torque fallback。

## 3. 闭链力矩 QP

完整决策变量：

$$
y=[\ddot q_{14};\lambda_6;s_8]
$$

- `ddq`：双臂关节加速度。
- `lambda`：闭链约束 wrench。
- `s`：碰撞约束 slack。

动力学：

$$
M\ddot q+h=\tau+C^T\lambda
$$

因此：

$$
\tau=[M\;-C^T\;0]y+h
$$

闭链硬等式：

$$
C\ddot q=b,
\qquad C=J_r,
\qquad b=-\dot J_r\dot q-k_vJ_r\dot q
$$

最后一项是 Baumgarte/速度阻尼，用于抑制数值漂移。

必须掌握的约束：

- 执行器力矩上下界。
- 关节加速度硬界。
- 预测速度界。
- 预测位置安全区。
- 闭链加速度等式。
- 碰撞距离不等式与非负 slack。

**【必须推导】** 从动力学推导 torque map；从下一周期速度/位置预测推导 `ddq` 上下界。

## 4. ADMM 求解需要理解到什么程度

不要求面试现场从零实现通用 OSQP，但必须能解释：

- 为什么这是凸 QP：Hessian 半正定/正定、约束线性。
- 固定尺寸矩阵为什么适合实时控制。
- `rho`、primal residual、dual residual 和最大迭代次数是什么。
- 为什么 warm start 有用。
- 为什么每周期重新动态分配稀疏矩阵会引入抖动。
- `MaxIterations` 不等价于一定不可用，但必须检查约束违反量。
- 求解失败时如何保持输出有限、有界、可诊断。

**【必须手写】** 不要求完整通用 ADMM，但必须能写出固定维度 QP 的矩阵装配：Hessian、linear term、constraint matrix、lower/upper bounds。

## 5. 碰撞有符号距离与梯度

每个碰撞 pair 提供：

- 有符号距离 `d`，负数表示穿透。
- 从 geom1 指向 geom2 的世界系法向 `n`。
- 两个最近点 `p1,p2`。
- 对 14 个关节的梯度 `g_d = ∂d/∂q`。

$$
\dot d=g_d\dot q
$$

$$
g_d=n^T(J_{p2}-J_{p1})
$$

推导时要说明为什么法向变化项和最近点沿表面的滑动项在一阶距离导数中消失，以及棱/顶点切换处只能认为是分段光滑。

**【必须手写】** 距离梯度有限差分测试，而不是从头重写 MuJoCo GJK/EPA。

## 6. Collision acceleration damper

速度级思想：当 `d < d_influence` 时，限制接近速度；越接近 `d_safe`，允许的接近速度越小。

把下一周期速度写成：

$$
\dot q_{k+1}=\dot q_k+\ddot q\Delta t
$$

代入：

$$
g_d(\dot q_k+\ddot q\Delta t)\ge\dot d_{min}(d)
$$

得到对 `ddq` 的线性不等式。

必须理解：

- `d_safe` 是希望保持的最小距离。
- `d_influence` 决定约束何时开始生效。
- 约束只影响接近方向，不应阻止远离障碍。
- 离散时间、高速运动和模型误差可能造成一步跨越。
- Slack 可以避免 QP 完全不可行，但 slack 变大本身是安全告警。

**【必须手写】** 给定 `d,g_d,dq,dt`，生成一行 acceleration inequality，并测试远离障碍时不会误触发。

## 7. 为什么 QP 不是路径规划器

若原始参考直线穿过障碍，局部 damper 会让系统停在障碍前；它无法判断应该从上、下、左还是右绕行。这是局部约束与全局/行为级参考生成的区别。

本项目消融证据：

- `straight_coop`：完成轨迹但穿透 30.12 mm。
- `qp_only`：保持 9.81 mm 距离，但终态误差 114.53 mm。
- 完整 QP + governor：保持 10.00 mm 并完成任务。

## 8. Reference governor

状态机：

```text
NORMAL → LIFT → CROSS → DESCEND → NORMAL
```

它使用距离、参考运动方向和预设绕行方向，修改物体参考并管理 virtual time。

必须理解：

- Governor 修改的是参考，不直接输出关节力矩。
- 原轨迹时间不能在绕行期间无条件继续，否则回到轨迹时会跳变。
- 状态切换需要 hysteresis，避免距离阈值附近抖动。
- 偏移要限速/平滑，不能产生加速度突变。
- 当前 `preferred_direction` 是配置给定的，因此它是局部行为层，不是通用 3D planner。

**【必须手写】** 一个无机器人依赖的四状态 governor 更新函数，并用合成距离序列测试完整状态顺序和只触发一次。

## 9. 优先级与不可行性

推荐优先级：

1. 输出必须有限、力矩必须在硬件范围内。
2. 物理闭链等式不能为了避障随意放松。
3. 关节安全边界。
4. 碰撞约束；必要时使用高权重 slack 并触发故障状态。
5. 名义跟踪性能。

面试时必须能回答：如果碰撞约束与闭链约束冲突怎么办？正确答案不是让两只刚性抓住物体的手在数学上分开，而是调整上层参考、停止任务或进入安全退化。

## 10. 验证命令

```bash
cd /home/tt/dual_arm_ws
./build/dual_arm_tests --gtest_filter='BoxTorqueQp.*:JointSafetyTorqueQp.*:QpCoopController.*'
./build/dual_arm_tests --gtest_filter='CollisionModel.*:Scenes/CollisionModelTest.*'
./scripts/run_experiment_matrix.py --skip-build --ablation-only
```

## 11. 本章口试题

1. QP 中 `lambda` 是传感器测得的 wrench，还是动力学约束乘子？
2. 为什么闭链约束使用硬等式，而碰撞约束带 slack？
3. P99 很低但偶尔 `MaxIterations`，你会看哪些诊断量？
4. 为什么 collision damper 保证局部不接近，却不保证任务可达？
5. Governor 的 virtual time 解决了什么问题？
