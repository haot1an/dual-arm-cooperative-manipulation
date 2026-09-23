# 01 数学、坐标系与机器人基础

## 1. 坐标约定是所有公式的入口

本项目统一使用：

- 所有控制量默认表达在世界系 `W`。
- `twist = [v; omega]`，前 3 维是线速度，后 3 维是角速度。
- `wrench = [f; m]`，前 3 维是力，后 3 维是力矩。
- `Pose T_ab` 应理解为把 `b` 系中的点表达为 `a` 系。
- Quaternion 必须归一化，项目使用 Eigen 的 `(w,x,y,z)` 语义。
- 角速度、旋转误差和 Jacobian 角速度行必须在同一表达坐标系。

看到任何六维量，必须先问四件事：表达在哪个坐标系、参考点在哪里、排列顺序是什么、正方向是什么。

源码入口：

- `include/dual_arm/types.hpp`
- `include/dual_arm/math_utils.hpp`
- `src/math_utils.cpp`
- `tests/test_math_utils.cpp`

## 2. 叉乘矩阵

定义：

$$
S(a)=\begin{bmatrix}
0&-a_z&a_y\\
a_z&0&-a_x\\
-a_y&a_x&0
\end{bmatrix},\qquad S(a)b=a\times b
$$

必须记住：

- `S(a)^T = -S(a)`。
- `a x b = -b x a`。
- 抓取矩阵符号错误经常来自 `p_o-p_i` 与 `p_i-p_o` 混用。

**【必须手写】** `Matrix3d skew(const Vector3d& v)`，并用随机向量验证 `skew(a)*b == a.cross(b)`。

## 3. SE(3) 位姿

位姿由旋转 `R` 和位置 `p` 构成：

$$
T=\begin{bmatrix}R&p\\0&1\end{bmatrix}
$$

复合与逆：

$$
T_{ac}=T_{ab}T_{bc},\qquad
R_{ac}=R_{ab}R_{bc},\qquad
p_{ac}=p_{ab}+R_{ab}p_{bc}
$$

$$
T_{ab}^{-1}=\begin{bmatrix}R_{ab}^T&-R_{ab}^Tp_{ab}\\0&1\end{bmatrix}
$$

**【必须手写】** `Pose::operator*`、`Pose::inverse()` 和点变换。验收条件：随机位姿满足 `T*T.inverse()` 的位置误差 `<1e-12`，旋转误差 `<1e-12 rad`。

## 4. 旋转误差

姿态误差不能直接对 RPY 做减法，因为：

- RPY 有奇异性和角度跳变。
- 旋转不满足普通向量加法。
- 大角度时三个轴耦合。

本项目使用世界系旋转向量形式。你需要理解：

$$
R_e=R_dR^T,\qquad e_R=\log(R_e)^\vee
$$

小角度下可以近似为反对称部分的 vee 映射；接近 `pi` 时要特别处理数值稳定性。

**【必须手写】** `rotationErrorWorld()` 或等价函数。至少测试：零误差、绕单轴小角度、90 度、接近 180 度、四元数符号翻转。

## 5. Twist 与 wrench 的参考点移动

刚体角速度相同，不同点的线速度满足：

$$
v_i=v_o+\omega\times(p_i-p_o)
$$

力不随参考点改变，力矩满足：

$$
m_o=m_i+(p_i-p_o)\times f
$$

二者必须满足功率不变性：

$$
h_i^T\nu_i=h_o^T\nu_o
$$

这是检查转置、符号和参考点最有效的统一原则。

**【必须推导】** 从速度关系推导 wrench 变换，不允许分别死记两个矩阵。

## 6. Jacobian

几何 Jacobian：

$$
\nu_{ee}=J(q)\dot q
$$

加速度：

$$
\dot\nu_{ee}=J\ddot q+\dot J\dot q
$$

必须掌握：

- Jacobian 的线速度参考点必须与末端控制点一致。
- 世界系 Jacobian 不能直接和 body-frame 误差混用。
- `Jdot*qdot` 比完整 `Jdot` 更常用，可通过中心有限差分验证。
- 奇异值 `sigma_min`、条件数和 manipulability 的含义不同。

**【必须手写】** Jacobian 有限差分测试：对每个关节加入 `epsilon`，比较末端位姿变化与 `J.col(j)`。不要在业务代码中用有限差分替代解析 Jacobian；有限差分是验证工具。

验证：

```bash
cd /home/tt/dual_arm_ws
./build/dual_arm_tests --gtest_filter='Jacobian.*:MathUtils.*'
```

## 7. 刚体动力学

单臂动力学采用：

$$
M(q)\ddot q+h(q,\dot q)=\tau+J^Th_{ext}
$$

其中 `h` 通常包含 Coriolis、离心和重力项。必须区分：

- `gravity(q)`：只有重力。
- `bias(q,dq)`：通常是 `C(q,dq)dq + g(q)`。
- `M(q)`：对称正定质量矩阵。
- `J^T h_ext`：外部 wrench 到广义力的映射。

常见错误：把 `gravity` 与 `bias` 同时加入，导致重力补偿两次；或者 wrench 符号定义是“环境作用在机器人”，代码却按“机器人施加给物体”处理。

最低验收：

- 能解释为什么 `M` 必须对称正定。
- 能解释静止时 `bias ≈ gravity`。
- 能写出 `tau = J^T h + bias` 并说明每项单位。
- 能根据自由落体、重力保持测试判断力矩符号。

验证：

```bash
cd /home/tt/dual_arm_ws
./build/dual_arm_tests --gtest_filter='Dynamics.*:GravityComp.*'
```

## 8. 本章口试题

1. 为什么 twist 和 wrench 的变换互为逆转置？
2. 如果 `r` 的定义反过来，抓取矩阵哪一块需要改变？
3. 为什么旋转误差不能使用 RPY 直接相减？
4. `Jdot*qdot` 在闭链 QP 中漏掉会产生什么现象？
5. `bias` 与 `gravity` 的差别是什么？
