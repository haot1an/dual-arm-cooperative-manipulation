# 08 必须能独立写出的代码

本章是最直接的能力检查。建议建立临时学习分支，先不看正式实现，写完后运行对应测试。面试不一定要求完整控制器，但以下项目应能写出核心部分。

## A 级：必须现场写出

### 1. 叉乘矩阵

函数：

```cpp
Matrix3d skew(const Vector3d& v);
```

限时：5 分钟。

必须同时写出测试：`skew(a)*b == a.cross(b)`、`skew(a).transpose() == -skew(a)`。

### 2. 位姿误差

函数：

```cpp
Vector6d poseError(const Pose& desired, const Pose& measured);
```

限时：15 分钟。必须说明姿态误差表达在哪个坐标系，不能使用 RPY 直接相减。

### 3. 单臂抓取矩阵

函数：

```cpp
Matrix6d graspMatrixArm(const Vector3d& r_i);
```

限时：10 分钟。必须写出功率一致性测试，并说明 `r_i = p_o-p_i`。

### 4. 整体抓取矩阵

函数：

```cpp
Matrix6x12d graspMatrix(
    const Vector3d& r_left,
    const Vector3d& r_right);
```

限时：5 分钟。重点是 block 位置和左右臂堆叠顺序。

### 5. Cartesian impedance 核心

核心代码必须能写出：

```cpp
const Vector6d error = poseError(desired_pose, current_pose);
const Vector6d derror = desired_twist - current_twist;
const Wrench wrench = K.cwiseProduct(error)
                      + D.cwiseProduct(derror)
                      + wrench_ff;
Vector7d tau = J.transpose() * wrench + bias;
```

限时：15 分钟。必须解释每个量的单位、坐标系和参考点。

### 6. 内力投影

给定 `G` 与 `h_measured`，写出：

$$
h_{internal}=(I-G^+G)h_{measured}
$$

限时：20 分钟。要求使用线性求解器，不显式求逆，并验证 `G*h_internal≈0`。

### 7. 中心有限差分测试

函数形式自定。限时：15 分钟。必须处理尺度、误差输出和每一列独立检查。

## B 级：必须能在 30–45 分钟完成

### 8. 加权 wrench 分配

```cpp
Matrix12x6d weightedPseudoInverse(
    const Matrix6x12d& G,
    const Matrix12d& W);

Vector12d distributeObjectWrench(
    const Matrix6x12d& G,
    const Wrench& object_wrench,
    const Vector12d& requested_internal_wrench,
    const Matrix12d& W);
```

验收：物体 wrench 重构、内力零空间、不同 load share 三组测试。

### 9. 绝对/相对 Jacobian

```cpp
Matrix6x14d absoluteJacobian(...);
Matrix6x14d relativeJacobian(...);
```

验收：理想闭链 `J_r*dq≈0`，两臂给出的物体 twist 一致。

### 10. 独立 Cartesian impedance `compute()`

必须完成整个函数，而不只是三行控制律：模型更新、轨迹、末端参考、物体重力分配、两臂循环、bias 和返回值。

验收：保持测试、轨迹测试、零分配测试。

### 11. 物体空间协作控制主链

需要独立写出：

```text
object reference
→ object wrench
→ internal wrench feedback
→ weighted distribution
→ two J^T mappings
→ torque outputs
```

验收：`CoopController.*`、`CoopTemplate.*` 和 no-allocation。

### 12. Collision damper 单行约束

输入：`d, grad_d, dq, dt, safe_distance, influence_distance, max_approach_speed`。

输出：对 `ddq` 的线性不等式行和下界。必须覆盖：约束未激活、接近、远离、安全距离内四种情况。

### 13. Governor 状态机

输入：当前 phase、距离、法向运动方向、offset、virtual time。输出：新 phase、平滑 offset、是否推进原轨迹。

验收：完整顺序、hysteresis、单次触发、最终 offset 归零。

## C 级：必须能讲清并补全代码

### 14. 固定尺寸 QP 装配

要求能根据公式填写：

- Hessian 与 linear cost。
- Torque map。
- `ddq` bounds。
- Closed-chain equality rows。
- Collision rows 与 slack。
- Warm start 与 status diagnostics。

不要求现场从零实现通用求解器，但必须能定位每一行矩阵的物理含义。

### 15. 螺旋运动与约束投影

写出 screw basis 和 `P_perp`，解释为什么 rank 为 5，并能扩展到任意世界系轴和参考点。

### 16. 装配状态机

给出 preload/torque/speed 数据，写出带保持时间的 phase 转移，避免单点噪声触发。

### 17. 无动态分配测试

能解释并补全 malloc/new 插桩测试：初始化结束后开始计数，连续运行控制器，断言分配次数为零。

## 练习规则

1. 不复制正式实现。
2. 先写测试，再写函数。
3. 所有六维量在代码旁标注 frame/reference/order。
4. 所有矩阵在纸上写出维度。
5. 禁止用 `.inverse()` 逃避线性求解。
6. 失败时先解释物理现象，再看数值。
7. 完成后用 `git diff` 比较思路，不以逐行一致为目标。

## 推荐测试命令

```bash
cd /home/tt/dual_arm_ws

./build/dual_arm_tests --gtest_filter='MathUtils.*'
./build/dual_arm_tests --gtest_filter='CoopTemplate.*'
./build/dual_arm_tests --gtest_filter='IndependentCartesianImpedance.*:CoopController.*'
./build/dual_arm_tests --gtest_filter='BoxTorqueQp.*:JointSafetyTorqueQp.*'
./build/dual_arm_tests --gtest_filter='QpCoopController.*:QpAsymCoopController.*'
./build/test_no_alloc
```

## 最终闭卷题

在 90 分钟内完成以下最小系统：

1. 写 `skew` 与 `graspMatrixArm`。
2. 写两臂整体 `G`。
3. 写加权 wrench 分配。
4. 写六维物体 PD wrench。
5. 写 `J^T` 力矩映射。
6. 写 `G*h=w` 与 `G*h_internal=0` 测试。

如果不看项目能通过这组题，说明你已经真正掌握双臂控制的核心，而不是只会运行 Demo。
