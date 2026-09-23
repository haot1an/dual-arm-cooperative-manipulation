# 09 六周学习与复习计划

这个计划按每天 1.5–2.5 小时设计。时间不足时可以延长周期，但不要跳过手写和故障注入。

## 第 1 周：数学与单臂控制

目标：建立统一的 frame、point、order、sign 思维。

- Day 1：`Pose`、SE(3) 复合与逆。
- Day 2：skew、twist、wrench、功率不变性。
- Day 3：旋转误差、Quaternion、有限差分。
- Day 4：Jacobian、`Jdot*qdot`、奇异性。
- Day 5：动力学 `M,h,g` 与 gravity PD。
- Day 6：独立手写 Cartesian impedance。
- Day 7：闭卷复盘和故障注入。

周验收：不看源码写出 Cartesian impedance 核心，并通过对应测试和 no-allocation。

## 第 2 周：双臂协作数学

- Day 1：推导 `G_i` 与功率对偶。
- Day 2：整体 `G`、rank、null space。
- Day 3：推导 `J_a`、`J_r`。
- Day 4：加权伪逆和 load sharing。
- Day 5：内部 wrench 和六维相对坐标。
- Day 6：完成 `CoopTemplate.*` 手写练习。
- Day 7：白板讲解完整双臂链路。

周验收：能从 `nu_i=G_i^Tnu_o` 推到相对 Jacobian，并写出 wrench 分配代码。

## 第 3 周：协同控制器

- Day 1：物体空间阻抗。
- Day 2：物体惯量、重力与旋转动力学前馈。
- Day 3：内力反馈和 ramp。
- Day 4：`J^T` 映射、bias、零空间姿态。
- Day 5：独立补全 `computeWithReference()`。
- Day 6：跑 lift/slot，对照日志解释每一列。
- Day 7：比较独立阻抗与协同控制。

周验收：能说明两个控制器在物理结构上的差别，而不只是代码差别。

## 第 4 周：QP 与避障

- Day 1：box QP、凸性、KKT 基础。
- Day 2：动力学 torque map 与闭链等式。
- Day 3：关节位置/速度预测 bounds。
- Day 4：距离、最近点、法向和梯度。
- Day 5：collision damper 与 slack。
- Day 6：reference governor 和 virtual time。
- Day 7：运行四组消融并复述因果结论。

周验收：画出完整 QP 的变量和每类约束，解释 QP-only 为什么任务失败。

## 第 5 周：装配、标定与传感

- Day 1：screw theory 和投影矩阵。
- Day 2：holding/working arm 分工。
- Day 3：预紧力、admittance、拧紧力矩。
- Day 4：装配状态机。
- Day 5：F/T、weld wrench、重力补偿。
- Day 6：基座标定误差到内部力的传播。
- Day 7：运行 assembly 与 calibration 测试。

周验收：能解释左臂如何参与拧紧，以及实际系统如何发现标定误差。

## 第 6 周：工程、实验与面试

- Day 1：固定尺寸 Eigen 和零分配。
- Day 2：MuJoCo model/data、约束和 contact。
- Day 3：测试金字塔与有限差分。
- Day 4：验收、消融、Monte Carlo。
- Day 5：1 kHz 指标与实机边界。
- Day 6：完成 90 分钟闭卷题。
- Day 7：录制一次 5 分钟项目讲解并回看。

周验收：能在 5 分钟内用“问题—方法—证据—边界”讲清整个项目。

## 每周固定输出

每周至少留下：

- 一页手写推导或电子笔记。
- 一个自己写的核心函数。
- 一个自己写的测试。
- 一个故障注入记录。
- 一段 2–5 分钟口头讲解。

## 5 分钟项目讲解模板

### 0:00–0:40 问题

双臂刚性抓取形成闭链，除了物体运动，还会产生不会改变物体净 wrench 的内部力；狭窄环境还要求同时满足力矩、关节和碰撞约束。

### 0:40–1:40 数学与控制

抓取矩阵统一运动学和静力学；物体阻抗生成合 wrench；加权伪逆分配到双臂；内力在 `null(G)` 中独立调节。

### 1:40–2:40 QP 与避障

固定尺寸 torque QP 处理闭链、关节、力矩和距离约束；reference governor 负责把被障碍阻断的参考在线修改成抬升、跨越、下降。

### 2:40–3:30 装配

Holding arm 稳定工件，working arm 在五维约束子空间对准，并沿一维 screw motion 建立预紧和拧紧力矩。

### 3:30–4:30 证据

讲 98/98 测试、消融数字、12/12 鲁棒性 sweep、1 kHz P99.9 和无动态分配。

### 4:30–5:00 边界

明确 weld 抽象、无手指摩擦/滑移、局部 governor、普通 Linux soft real-time 和尚未完成实机/HIL。

## 是否可以进入下一阶段

满足以下条件后再考虑 ROS 2 或实机：

- [ ] 90 分钟闭卷题完成。
- [ ] 五个核心面试题能连续回答。
- [ ] 能独立定位一次符号错误和一次 QP 约束错误。
- [ ] 能解释所有简历数字如何复现。
- [ ] 能指出仿真中至少五个不等同于实机的假设。
- [ ] 能给出实机 bring-up 的安全步骤。
