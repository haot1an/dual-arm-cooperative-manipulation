# 05 非对称装配、力控制与标定误差

## 1. 对称协作与非对称协作

对称搬运中，两臂共同控制同一个物体的六维运动。螺钉装配中：

- Holding arm 稳定工件。
- Working arm 稳定工具与螺纹轴的五个约束方向。
- 沿螺旋自由度执行旋转和轴向进给。

因此不能直接使用“相对自由度为 0”的刚性协作控制器。

源码入口：

- `include/dual_arm/asym_coop_controller.hpp`
- `src/asym_coop_controller.cpp`
- `include/dual_arm/qp_asym_coop_controller.hpp`
- `src/qp_asym_coop_controller.cpp`
- `config/scenes/assembly.yaml`
- `models/scene_assembly.xml`

## 2. 螺旋运动基础

螺纹导程 `lead` 表示转一圈的轴向进给。每弧度进给：

$$
h=\frac{lead}{2\pi}
$$

若轴方向为单位向量 `a`，轴上一点为 `c`，twist 参考点为 `p`，单位角速度对应的 screw basis：

$$
S=\begin{bmatrix}
ha+a\times(p-c)\\a
\end{bmatrix}
$$

允许运动是一维 `span(S)`；其正交补是五维约束空间：

$$
P_\perp=I-\frac{SS^T}{S^TS}
$$

因此相对自由度为 1，内部约束 wrench 维数为 `6-1=5`。

**【必须手写】** `screwMotionBasisAtPoint()` 和 `screwConstraintProjector()`，验证：

```text
P * S ≈ 0
P * P ≈ P
P.transpose() ≈ P
S.transpose() * constrained_wrench ≈ 0
```

## 3. Holding arm 与 working arm

Holding arm 需要：

- 抑制工件位姿漂移。
- 承担工具施加的反力/反力矩。
- 保持在力矩和碰撞约束内。

Working arm 需要：

- 在五维约束子空间保持对准。
- 沿轴向建立预紧力。
- 沿螺旋自由度调节旋转速度或拧紧力矩。
- 检测入座和完成条件。

这才是双臂配合：右臂的作业 wrench 会通过工件和闭链传到左臂，左臂必须稳定工件而不是静止不参与。

## 4. 轴向预紧与拧紧力矩

轴向预紧不能只给一个恒定位置偏移。更合理的闭环：

$$
F_{cmd}=F_{ref}+K_f(F_{ref}-F_{meas})
$$

需要 ramp、滤波和限幅。拧紧阶段可使用力矩 admittance：力矩误差决定允许的螺旋速度，并受最大转速限制。

必须理解：

- F/T 的力方向和“机器人作用于物体”/“物体作用于机器人”符号。
- preload 和 tightening torque 的参考轴必须来自同一个世界系 screw axis。
- 力滤波降低噪声，但引入相位滞后。
- 完成判据不能只看瞬时力矩，要加入保持时间、转速和预紧范围。

## 5. 装配状态机

典型阶段：

```text
APPROACH/PRELOAD → ROTATE/TIGHTEN → HOLD
```

每个转移需要：阈值、持续时间、超时和失败出口。项目目前验证正常路径；真实系统还需要：工具未对准、预紧建立失败、螺纹卡死、滑牙、传感器异常和超时退回。

**【必须手写】** 一个纯逻辑状态机测试：给定合成的 preload、torque、speed 时间序列，验证不会被单点噪声提前触发，并最终进入 `HOLD`。

## 6. 标定误差如何产生内力

右臂真实基座位姿与控制器名义位姿不同：

```text
外参误差
→ 控制器预测 TCP 与真实 TCP 不一致
→ 两臂给出互相矛盾的抓取几何
→ 刚性闭链阻止相对运动
→ 位姿误差通过等效刚度转化为预载 wrench
```

`weld.init_mode=current` 会按实际初始相对位姿建立约束，避免启动预载；`nominal` 模式按名义几何建立约束，可以主动暴露标定误差影响。

## 7. 实机标定与检测流程

建议顺序：

1. 单臂关节零位、TCP、工具质量和质心标定。
2. 两机械臂基座外参标定。
3. F/T 零点、温漂和重力补偿。
4. 空载单臂轨迹检查。
5. 柔顺低刚度建立闭链。
6. 检查静止内部 wrench 和相对位姿残差。
7. 小幅运动验证闭链一致性。
8. 再逐步提高刚度、速度和任务载荷。

发现手段：

- 闭链相对位姿误差。
- 左右臂末端对同一物体位姿的独立估计差。
- 静止时 F/T 非零预载。
- 关节力矩观测器残差。
- 不同姿态下残差是否呈系统性变化。

处理手段：重新标定、在线 bias、柔顺控制、内力反馈、在线外参估计和安全阈值。不能把“无限降低刚度”当作根治办法。

## 8. F/T、weld wrench 与仿真真值

必须区分：

- Wrist F/T：传感器参考点和方向定义。
- Weld constraint wrench：MuJoCo 约束求解器中的作用量。
- Hand subtree gravity：传感器会同时感受到手部/工具重力。
- 物体 wrench：换算到物体中心后的合 wrench。

任何比较前都要统一：作用方向、坐标系、参考点、重力是否扣除。

## 9. 验证命令

```bash
cd /home/tt/dual_arm_ws
./build/dual_arm_tests --gtest_filter='AsymCoopController.*:QpAsymCoopController.*:Assembly.*'
./build/dual_arm_tests --gtest_filter='Calibration.*:WristSensor.*:WeldWrench.*'
./scripts/run_acceptance_suite.py --skip-build --scenario assembly
```

## 10. 本章口试题

1. 为什么螺旋副使内部力维数从 6 变成 5？
2. Holding arm 在拧螺钉时做了什么，而不是“什么都没做”？
3. 如何区分螺钉入座、摩擦增大和工具卡死？
4. `current` 与 `nominal` weld 初始化分别适合验证什么？
5. 为什么 F/T 与 weld wrench 不能不经变换直接比较？
