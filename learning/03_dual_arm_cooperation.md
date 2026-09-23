# 03 双臂闭链、抓取矩阵与内力

## 1. 双臂协作的核心分解

双臂刚性抓取同一个物体后，控制问题分成两类互相正交的任务：

- 物体运动：两臂合力产生物体的净 wrench。
- 内部作用：两臂互相挤压、拉伸或扭转，但不改变物体净 wrench。

如果不能明确区分这两类量，就还没有真正掌握双臂协作。

源码入口：

- `include/dual_arm/coop_kinematics.hpp`
- `src/coop_kinematics.cpp`
- `include/dual_arm/coop_controller.hpp`
- `src/coop_controller.cpp`
- `tests/test_coop_template.cpp`

## 2. 单臂抓取矩阵

约定 `r_i = p_o - p_i`，从抓取点指向物体中心：

$$
G_i=\begin{bmatrix}I&0\\-S(r_i)&I\end{bmatrix}
$$

静力学：

$$
h_o=G_i h_i
$$

运动学：

$$
\nu_i=G_i^T\nu_o
$$

为什么是转置：因为功率必须相等。

$$
h_i^T\nu_i=(G_i h_i)^T\nu_o
$$

**【必须手写】** `graspMatrixArm()` 和整体 `graspMatrix()`。必须能在白板上解释 `-S(r_i)` 的符号，而不是只记矩阵。

## 3. 整体抓取矩阵与内力空间

$$
G=[G_1\;G_2]\in\mathbb{R}^{6\times12},\qquad w_o=Gh
$$

刚性双抓取通常 `rank(G)=6`，因此：

$$
\dim\operatorname{null}(G)=12-6=6
$$

如果 `h_int in null(G)`：

$$
Gh_{int}=0
$$

它不会产生物体净 wrench，却会改变接触和结构载荷。

典型内部作用：沿抓取连线挤压、横向剪切、相对扭转、相对弯矩。不能把“内力”只理解成一个夹紧标量。

## 4. 绝对与相对 Jacobian

绝对 Jacobian描述物体整体运动：

$$
J_a=\frac12[G_1^{-T}J_1\quad G_2^{-T}J_2]
$$

相对 Jacobian描述两条虚拟杆预测的物体 twist 差：

$$
J_r=[-G_1^{-T}J_1\quad G_2^{-T}J_2]
$$

刚性闭链速度约束：

$$
J_r\dot q=0
$$

加速度约束：

$$
J_r\ddot q+\dot J_r\dot q=0
$$

**【必须推导】** 从 `nu_i=G_i^T nu_o` 分别推导 `J_a` 和 `J_r`，并解释为什么闭链 QP 必须使用加速度级等式。

## 5. 加权 wrench 分配

满足 `Gh=w_o` 的解不唯一。加权最小范数解：

$$
G_W^+=W^{-1}G^T(GW^{-1}G^T)^{-1}
$$

$$
h=G_W^+w_o+(I-G_W^+G)h_{int}
$$

第一项产生物体运动，第二项位于 `null(G)`。

工程实现注意：

- 不应显式计算矩阵逆，优先使用 LDLT/LLT 求解线性系统。
- `W` 必须对称正定。
- 左臂承担比例更大，意味着对应代价权重更小。
- 必须验证 `G*h ≈ w_o` 和 `G*h_internal ≈ 0`。

**【必须手写】** `weightedPseudoInverse()`、`distributeObjectWrench()`、`internalWrench()`。

## 6. 内力坐标

项目使用六维相对 wrench 坐标：

$$
h_r=\frac12(G_2h_2-G_1h_1)
$$

纯内力时：

$$
G_1h_1=-h_r,\qquad G_2h_2=h_r
$$

你必须能说明：

- 为什么 `Gh=0` 不代表每只手 wrench 为零。
- 为什么内力过大会损坏物体或使抓取滑移。
- 为什么完全不控制内力也可能在存在模型误差时积累预载。

## 7. 物体空间阻抗

典型物体 wrench：

$$
w_o=M_o\dot\nu_d+c_o-w_g+K_oe_o+D_o(\nu_d-\nu_o)
$$

项目实现包括：

- 平移惯性前馈 `m*a_des`。
- 世界系旋转后的物体惯量。
- `I*alpha + omega x I*omega`。
- 物体重力补偿。
- 位姿与 twist 反馈。

随后通过 `G_W^+` 分配，再用 `J_i^T h_i + bias_i` 映射到双臂。

**【必须手写】** `CoopController::computeObjectWrench()` 和完整 `computeWithReference()` 主链路。

## 8. 必须完成的验证

```bash
cd /home/tt/dual_arm_ws
./build/dual_arm_tests --gtest_filter='CoopTemplate.*:CoopController.*'
./build/test_no_alloc --gtest_filter='NoAlloc.CoopControlLoopDoesNotAllocate'
```

手工检查四个恒等式：

```text
G * G_weighted_plus ≈ I6
G * distributed_wrench ≈ object_wrench
G * internal_wrench ≈ 0
J_relative * dq ≈ 0  （理想闭链）
```

## 9. 常见错误及症状

| 错误 | 典型症状 |
|---|---|
| `r_i` 符号反了 | 物体转矩方向错误、功率测试失败 |
| wrench 参考点不一致 | 合力正确但合力矩错误 |
| F/T 作用方向反了 | 内力反馈变成正反馈 |
| 直接求逆 | 奇异附近数值放大 |
| 左右臂堆叠顺序混乱 | 负载分配与力矩映射交叉 |
| 漏掉物体重力 | 轨迹保持存在明显下沉 |
| 把内力加到物体 wrench | 物体产生不希望的整体运动 |

## 10. 本章口试题

1. 为什么刚性双抓取的内力空间是 6 维？
2. `G` 的零空间和 `J_r` 的约束空间有什么对偶关系？
3. 为什么负载分配不能简单地把每个 wrench 分量除以 2？
4. 左臂负载份额从 0.5 改成 0.7 时，`W` 应怎样变化？
5. 如何通过实验区分物体跟踪误差和内部力问题？
