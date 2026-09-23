# 项目面试问题与回答主线

下面五个问题同时用于项目自查和面试准备。回答时先讲物理/数学约束，再指出代码位置和实验数字，不要只罗列技术名词。

## 1. 为什么抓取矩阵中平移项的符号是 `-S(r_i)`？如何证明实现没有坐标系错误？

回答主线：先明确 wrench 和 twist 都按 `[linear; angular]` 排列，且 `r_i = p_o - p_i`。抓取点的力在物体中心产生力矩 `-r_i x f_i`，因此 `G_i = [I, 0; -S(r_i), I]`。运动学映射是其对偶 `nu_i = G_i^T nu_o`。最后用功率不变性验证：`h_i^T nu_i = (G_i h_i)^T nu_o`。

仓库证据：`graspMatrixArm()`、`MathUtils.WrenchAndTwistShiftPreservePower`、`CoopTemplate.GraspMatrixMapsWrenchToObjectCenter`。

## 2. 为什么只加入 collision QP 仍然无法完成越障？QP 和 reference governor 分别解决什么问题？

回答主线：collision acceleration damper 是局部约束，只限制距离方向上的接近速度/加速度；当原始路径被障碍分成不可直达的区域时，它会把机器人停在障碍前，却不会创造绕行参考。reference governor 根据几何距离切换 `LIFT -> CROSS -> DESCEND`，负责参考可行化；QP 继续执行闭链、关节、力矩和距离安全约束。

仓库证据：消融中 `qp_only` 保持 9.81 mm 障碍距离，但终态误差为 114.53 mm；完整方案保持 10.00 mm 并以近零终态误差完成任务。

## 3. 力矩级 QP 的变量、等式约束和失败退化策略是什么？

回答主线：决策变量为双臂关节加速度、闭链约束 wrench 和有限数量的 collision slack。动力学用于把期望加速度与执行器力矩联系起来；闭链约束使用相对 Jacobian 和 `Jdot*qdot` 构造加速度等式；不等式包含关节位置/速度预测、力矩边界和碰撞 damper。回答还应说明数值残差、最大迭代次数、slack 权重，以及求解输入无效或不收敛时不能直接输出 NaN/越界力矩。

仓库证据：`JointSafetyTorqueQp`、`QpCoopController`、`QpAsymCoopController`，以及 `BoxTorqueQp.FallsBackToPreviousTorqueForInvalidDesiredInput` 等测试。

## 4. 基座标定误差为什么会产生闭链内力？真实系统如何发现和处理？

回答主线：两个末端被同一刚性物体闭合后，基座外参误差会让控制器预测的抓取位姿与真实约束不一致；两臂各自追踪互相矛盾的目标，因此误差通过闭链刚度变成预载内力。实际系统通过手眼/基座标定残差、腕部 F/T、关节力矩观测器、闭链位姿残差和静止时非零内力发现问题。处理顺序是离线外参标定、在线 bias/零点估计、柔顺控制与内力反馈，必要时进行在线外参估计，而不是无限降低刚度。

仓库证据：plant/controller 模型分离、`Calibration.*` 测试、weld 与 F/T 对照，以及随机鲁棒性 sweep。

## 5. “1 kHz、零动态分配、98 个测试通过”能证明实机可用吗？还缺什么？

回答主线：这些结果证明控制路径的计算预算、内存行为和已建模约束，不证明 hard real-time、传感器可靠性或接触模型真实性。当前普通 Linux 的 P99.9 为 396.9 us，但最大值为 3869 us，已经说明存在调度长尾。实机还需要 PREEMPT_RT/实时控制器、CPU 隔离、锁页、传感器同步、watchdog、急停、通信丢包处理、硬件在环和分阶段低速调试。

仓库证据：`test_no_alloc`、`run_realtime_benchmark.py`、GitHub Actions，以及 `docs/experiment_results.md` 中明确列出的建模边界。
