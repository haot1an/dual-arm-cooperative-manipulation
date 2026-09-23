# 02 单臂动力学与 Cartesian impedance

## 1. 为什么先掌握单臂基线

双臂控制不是两个单臂控制器的简单复制，但单臂阻抗是理解双臂物体阻抗的最小基础。你必须能把下面链路完整说清楚：

```text
期望末端位姿/速度
→ 位姿与速度误差
→ Cartesian wrench
→ Jacobian transpose
→ 关节力矩
→ 机器人动力学
```

源码入口：

- `include/dual_arm/baseline_controllers.hpp`
- `src/baseline_controllers.cpp`
- `tests/test_gravity_comp.cpp`
- `tests/test_no_alloc.cpp`

## 2. 关节 PD 与重力补偿

$$
\tau=g(q)+K_p(q_d-q)-K_d\dot q
$$

它适合检查模型、执行器符号和初始状态，不是 Cartesian 控制。需要理解：

- 增大 `Kp` 减小静差，但会提高刚度和振荡风险。
- `Kd` 通常按临界阻尼附近选择，但多自由度耦合时不能只看单个标量。
- 只补偿机械臂自身重力时，被抓物体重量会由位置误差产生的 PD 力矩承担。

**【必须手写】** 两臂循环中的 gravity PD，不允许在控制循环中创建动态矩阵或容器。

## 3. Cartesian impedance

基本形式：

$$
h_{cmd}=K_x e_x+D_x e_\nu+h_{ff}
$$

$$
\tau=J^T h_{cmd}+h(q,\dot q)
$$

这里不是“先算一个目标位置，再做 IK”。阻抗控制直接规定位姿误差与交互 wrench 的关系。

必须理解各部分：

- `e_x`：世界系六维位姿误差，位置和旋转向量。
- `e_nu = nu_d - nu`：同一坐标系、同一参考点的 twist 误差。
- `K_x`：前三维 N/m，后三维 N·m/rad。
- `D_x`：前三维 N·s/m，后三维 N·m·s/rad。
- `h_ff`：物体重力、期望加速度等前馈。
- `J^T`：通过虚功原理把末端 wrench 映射为关节力矩。

## 4. 两臂独立阻抗为什么只是 baseline

独立阻抗让两臂分别追踪由物体参考得到的抓取点参考。它的优点是结构直观，缺点是：

- 没有显式控制物体合 wrench。
- 没有把内部 wrench 与运动 wrench 分离。
- 标定或跟踪误差会形成互相对抗的力。
- 两臂负载分配只能通过前馈或增益间接调整。
- 无法统一处理闭链加速度、关节和碰撞约束。

这正是它作为消融基线的价值：它证明后续协同控制解决的不是“让两只手同时动”这么简单。

## 5. 重力前馈与负载分配

物体重力补偿需要在物体中心产生向上的 wrench：

$$
w_g=[0,0,mg,0,0,0]^T
$$

再通过抓取矩阵分配为两只手的前馈 wrench。不能简单地给每只手固定 `mg/2`，因为：

- 抓取点偏离质心会产生力矩。
- 负载份额可能不是 50/50。
- 一只手的 wrench 包含力和力矩六个分量。

## 6. 增益调整方法

推荐顺序：

1. 关闭轨迹，只做初始位姿保持。
2. 先调平移，再调旋转。
3. 从小刚度开始，逐步增加。
4. 观察位置误差、weld/F/T、力矩和饱和。
5. 加入低速最小 jerk 轨迹。
6. 加扰动检查恢复，不用单次峰值代替稳定性判断。

常见现象：

| 现象 | 可能原因 |
|---|---|
| 持续振荡 | 阻尼不足、时延、约束过硬 |
| 稳态下沉 | 缺少物体重力前馈或刚度太低 |
| 旋转方向相反 | 姿态误差坐标系/四元数顺序错误 |
| 两臂互相拉扯 | 抓取参考不一致、标定误差、缺少内力管理 |
| 力矩突跳 | 参考不连续、没有 ramp、姿态误差跨越分支 |

## 7. 必须完成的代码练习

### 练习 A：独立写阻抗核心

关闭 `src/baseline_controllers.cpp` 的原实现，独立写出：

```cpp
std::pair<Vector7d, Vector7d>
IndependentCartesianImpedance::compute(
    const DualArmState& state,
    double t);
```

必须包含：模型更新、物体参考到末端参考、重力 wrench 分配、位姿/速度误差、`J.transpose()`、bias 补偿。

验收：

```bash
cd /home/tt/dual_arm_ws
cmake --build build -j"$(nproc)"
./build/dual_arm_tests --gtest_filter='IndependentCartesianImpedance.*'
./build/test_no_alloc --gtest_filter='NoAlloc.CartesianImpedanceControlLoopDoesNotAllocate'
```

### 练习 B：解释每个变量的单位

对 `compute()` 中每个六维量标注：坐标系、参考点、排列和单位。如果有任何一个说不清楚，说明还没有达到 L3。

## 8. 本章口试题

1. 为什么阻抗控制使用 `J^T`，而速度 IK 使用伪逆 `J^+`？
2. 为什么要补 `bias`，而不是只补 `gravity`？
3. 两个独立阻抗控制器为什么会产生不可控内力？
4. Cartesian 刚度翻倍会对闭链内力和稳定性产生什么影响？
5. 如何证明你的姿态误差与 Jacobian 角速度行在同一坐标系？
