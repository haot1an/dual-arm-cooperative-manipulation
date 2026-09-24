# 物体空间轨迹优化：TrajOpt 风格序列凸规划 + 时间参数化

本文说明 `ObjectPathPlanner`（`include/dual_arm/trajectory_planner.hpp`）的问题定义、推导、算法与验收结果。
它在任务开始前把名义物体轨迹变形成无碰撞路径并重新计时；执行时，CBF 参考滤波（`docs/cbf_reference_governor.md`）
与力矩 QP 仍作为安全层。对比场景是 `slot_gate`。

---

## 0 阅读顺序（建议）

| 步骤 | 内容 | 位置 |
|---|---|---|
| 1 | 分层结构和为什么需要规划 | 本文 §1 |
| 2 | 变量、代价、约束 | §2–§3，`trajectory_planner.hpp` 文件头 |
| 3 | 距离对偏移的梯度（闭链链式法则） | §3，`ObjectPathPlanner::evaluate` |
| 4 | 序列凸规划主循环（信赖域 + 罚函数） | §4，`ObjectPathPlanner::plan` |
| 5 | 多初值与每次迭代的 QP（ADMM 求解器） | §4.3–§4.4，`src/sparse_qp.cpp` |
| 6 | 样条与重新计时 | §5–§6，`src/path_deformation.cpp`、`ObjectTrajectory::evaluate` |
| 7 | 结果与局限 | §7–§9 |
| 8 | 物体 SE(3) + 时间联合优化（`slot_gate` 默认） | §10，`src/trajectory_optimizer.cpp`、`src/closed_chain_ik.cpp` |
| 9 | 为什么规划之后还需要 CBF：执行层偏差实验 | §11，`scripts/run_execution_deviation.py` |

---

## 1 分层结构

```
名义轨迹（航点插值）
   │  任务开始前运行一次（slot_gate 约 2 s，含多初值）
   ▼
物体空间轨迹优化（本文）── 决定“往哪绕、绕多少”，碰撞检查覆盖物体 + 两臂（闭链 IK）
   │  PathDeformation：δ(τ) + τ(t)
   ▼
ObjectTrajectory::evaluate(t)  ── 变形 + 重新计时后的参考
   ▼
CBF 参考滤波（1 kHz）── 执行时的安全滤波（有全局规划时不主动绕行，escape_gain = 0）
   ▼
协同控制律 + 力矩 QP（1 kHz）── 跟踪、闭链、关节与碰撞约束兜底
```

规划解决的是局部方法解决不了的“方向选择”：`slot_gate` 中正确的绕行方向是**向下**穿过横梁，
而 CBF 的逃逸偏好方向是向上（§7 有对比）。执行偏差、模型误差和扰动仍交给下面两层。

## 2 变量

- 名义时间 $\tau$ 上取均匀节点 $\tau_k=kh$（$h\approx$ `knot_dt`，$k=0..N$），每个节点一个平移偏移 $\delta_k\in\mathbb R^3$；姿态沿用名义轨迹。
- **只允许侧向偏移**：名义速度 $v_k\ne0$ 的节点，$\delta_k=B_k z_k$，$B_k=[b_1\ b_2]$ 是 $\hat v_k$ 的正交补，$z_k\in\mathbb R^2$；
  静止节点（滚转、航点停留）$\delta_k=z_k\in\mathbb R^3$。沿路径方向的“进度”交给 §6 的时间参数化。
  这与 CBF 滤波的分工约束是同一个思想：否则偏移会沿路径方向“抵消”推进，问题退化。
- 起始 `pin_start`、结束 `pin_end` 内的节点固定 $\delta_k=0$：抓取起点和插槽末段保持名义设计。

## 3 代价与约束

$$
\min_{z}\ \underbrace{w_a\,h\sum_k\Big\|\tfrac{\delta_{k+1}-2\delta_k+\delta_{k-1}}{h^2}\Big\|^2 + w_o\,h\sum_k\|\delta_k\|^2}_{C(z)\ \text{（二次）}}
\quad\text{s.t.}\quad d_{kj}(\delta_k)\ \ge\ r_{kj}
$$

- $d_{kj}$：第 $k$ 个节点、第 $j$ 个障碍对（场景 `obstacles`，包括物体与两臂的所有 geom 对）的有符号距离。
  物体位姿 $T_k=(p_{\text{nom},k}+\delta_k,\ R_{\text{nom},k})$，两臂构型由**闭链 IK** 求得：两只 TCP 分别到达
  $T_k\,T_{o,g_i}$（阻尼最小二乘，相邻节点顺序热启动）。
- **距离对偏移的梯度**。物体平移 $\mathrm d\delta$ 时，两个抓取点都平移 $\mathrm d\delta$，关节增量为
  $\mathrm dq_i=J_i^{+}\,[\mathrm d\delta;0]$（与 IK 同一阻尼伪逆），所以

  $$
  \frac{\partial d}{\partial\delta}=\sum_{i\in\{L,R\}}\frac{\partial d}{\partial q_i}\,J_i^{+}\begin{bmatrix}I_3\\0\end{bmatrix},
  \qquad \frac{\partial d}{\partial z_k}=\frac{\partial d}{\partial\delta}B_k ,
  $$

  $\partial d/\partial q$ 直接来自 `CollisionModel`（已有有限差分测试）。
- **要求距离 $r_{kj}$**：目标值是 $d_s+$`safety_margin`（$d_s$ 为该障碍对的安全距离）。
  - 如果该障碍组在名义轨迹上有穿透（最小距离 < −2 mm），就是要修的障碍：$r=$ 目标值。
  - 否则取“**不比名义轨迹更差**”：$r=\min(\text{目标值},\ d^{\text{nom}}_{kj})$。这样起点贴着支撑块、
    插槽 1 mm 间隙这类名义设计不会让问题不可行。

## 4 序列凸规划（TrajOpt）

距离是非凸的，所以围绕当前解 $\bar z$ 线性化，逐次求解凸 QP（Schulman 等，TrajOpt，IJRR 2014）。

### 4.1 罚函数形式

$$
f_\mu(z)=C(z)+\mu\sum_{k,j}\max\big(0,\ r_{kj}-d_{kj}(z)\big)
$$

$\ell_1$ 罚是**精确罚**：$\mu$ 足够大时，$f_\mu$ 的极小点就是原约束问题的解。

### 4.2 每次迭代

1. 在 $\bar z$ 处取 $d<r+$`activation_distance` 的障碍对，线性化 $d\approx\bar d+g^\top(z-\bar z)$。
2. 求解凸子问题（松弛变量 $s$ 表示线性化后的违反量）：

   $$
   \min_{z,s}\ C(z)+\mu\sum s\quad
   \text{s.t.}\quad \bar d+g^\top(z-\bar z)+s\ge r,\ \ s\ge0,\ \ |z-\bar z|_\infty\le\Delta,\ \ |z|_\infty\le\delta_{\max}
   $$

3. 模型预测的下降 $\text{pred}=f_\mu(\bar z)-\hat f_\mu(z)$，实际下降 $\text{act}=f_\mu(\bar z)-f_\mu(z)$（在新解处重新做 IK 和距离查询）。
4. $\text{act}/\text{pred}>0.1$ 时接受，$>0.75$ 时信赖域 $\Delta$ 加倍；否则拒绝并把 $\Delta$ 减半。
5. 预测下降 ≈ 0、步长 ≈ 0 或 $\Delta$ 过小：若约束已满足（最坏不足 ≤ 0.5 mm）则结束，否则 $\mu\times10$ 继续。

### 4.3 多初值（应对局部解）

从名义轨迹（$z=0$）出发时，如果障碍穿透很深，最近点法向会沿着**路径方向**（板的正面撞上横梁的竖直前表面）。
而偏移只允许在侧向，投影后的梯度为 0，序列凸规划原地不动。这就是基于梯度方法的局部性（§9）。

对策：名义初值不可行时，找出违反要求距离的节点区间，在区间上（两侧加 `initial_ramp` 的余弦过渡）
预置沿候选方向（下 / 上 / −x / +x）、幅值 `initial_offset` 的平滑“鼓包”，各自跑一次序列凸规划，
**优先取可行解，再取代价最小者**。`slot_gate` 中“向下”的初值胜出（`PlanResult::initialization`）。
这是采样规划给初值（方案 C）的一个轻量替代：候选方向是有限的几个，而不是在构型空间里随机采样。

### 4.4 QP 求解器

变量约为 $2N$ 加上激活的松弛变量（`slot_gate` 中约 200 个）。约束是稀疏的：每行只涉及一个节点。
`sparse_qp::solve` 是 OSQP 形式的 ADMM（Stellato 等，2020）：

$$
(P+\sigma I+A^\top R A)\tilde x=\sigma x-q+A^\top(Rz-y),\quad
z\leftarrow\Pi_{[l,u]}(\alpha A\tilde x+(1-\alpha)z+R^{-1}y),\quad
y\leftarrow y+R(\cdots)
$$

KKT 矩阵用稀疏 LDLᵀ 分解，$\rho$ 按原始 / 对偶残差比自适应调整。单元测试 `SparseQp.MatchesBruteForceOnRandomProblems` 用穷举有效集对照。
它只在规划时离线使用（会分配内存），与 1 kHz 回路里的定长求解器（`small_qp`、`torque_qp`）分开。

## 5 路径变形 $\delta(\tau)$

节点偏移用**两端一阶导为 0 的三次样条**插值（C²，三对角方程），节点外为 0。
$\delta',\delta''$ 解析给出（`PathDeformation::offset`），供参考速度和加速度前馈使用。

## 6 时间参数化

变形后的路径更长，可能超过速度 / 加速度上限。做法是**分段放慢**：

$$
k(\tau)=\max\Big(1,\ \frac{\|v_{\text{nom}}+\delta'\|}{v_{\max}},\ \sqrt{\frac{\|a_{\text{nom}}+\delta''\|}{a_{\max}}}\Big)
$$

先做滑动最大，再做滑动平均（结果 $\ge$ 原值，且连续），然后由 $\mathrm dt=k\,\mathrm d\tau$ 积分得到 $t(\tau)$ 并反查。
$\dot\tau=1/k$，$\ddot\tau=-k'/k^3$。执行时的参考（`ObjectTrajectory::evaluate`）为：

$$
p=p_{\text{nom}}(\tau)+\delta,\quad v=\dot\tau(v_{\text{nom}}+\delta'),\quad a=\dot\tau^2(a_{\text{nom}}+\delta'')+\ddot\tau(v_{\text{nom}}+\delta') .
$$

这是近似做法：速度上限严格满足，加速度上限因为 $\ddot\tau$ 项只是近似满足（测试允许 30% 余量）。
时间最优的做法是 TOPP-RA（可达性分析），后续可替换。`slot_gate` 中名义速度已经较低，重新计时没有增加时长。

## 7 结果（`slot_gate`，刚性抓取、关闭接触）

场景：横梁底面 z = 1.17 m，竖直板顶在名义路径上高出 6 cm，须下压约 8 cm 从横梁下穿过，
穿过后又要在约 5 cm 的平移内抬回槽壁（顶面 1.0 m）以上。

规划：名义初值停在局部解，多初值中“向下”胜出；5 次起点、56 次 QP 迭代，约 2.1–2.3 s。
最大偏移 96.5 mm（向下），所有节点满足要求距离（目标 20 mm），名义速度下无需额外放慢。

| 指标 | 规划 + CBF 安全滤波 | 只用 CBF（逃逸方向 +z） | 只用力矩 QP 层 |
|---|---:|---:|---:|
| 是否完成插槽 | **是** | **否**（距目标 95 mm） | 是 |
| 板 ~ 横梁最小距离 | **19.5 mm** | 7.5 mm | 9.8 mm |
| 两臂 ~ 横梁最小距离 | **36.5 mm** | 10.0 mm | 10.1 mm |
| 最大位置跟踪误差 | **9.3 mm** | 275 mm | 160 mm |
| 到达目标（2 mm 内） | 15.49 s | — | 15.45 s |
| 最大参考偏移（CBF） | 3 mm | 162 mm | — |

- **只用 CBF**：逃逸项把参考往上推，但横梁高到 1.60 m，向上绕不过去；物体被下层 QP 顶在横梁前，
  两臂被压在 10 mm 安全距离上，板与横梁的距离也跌破了 CBF 自己的 10 mm；约 20 s 时仍离目标 95 mm。
  这次运行在 t = 20.6 s 因 MuJoCo 的 EPA 缓冲区越界而崩溃（见 §9.6，未打补丁时可复现），数据取崩溃前的日志。
- **只用力矩 QP 层**：下层距离约束最终把物体挤到横梁下方，也完成了任务，但跟踪误差 160 mm，是规划方案的 17 倍，
  两臂全程贴着 10 mm 安全距离。
- **规划 + CBF**：路径事先就从下方穿过（20 mm 余量），CBF 基本不介入，跟踪误差最小，两臂离横梁 36 mm。

接触夹取 + 抓取对准 + 规划 + CBF：终态 0.56 mm / 0.04°，无失接触（`ctest -R ContactGrasp.SlotGatePlanned`）。

## 8 测试

| 测试 | 内容 |
|---|---|
| `SparseQp.MatchesBruteForceOnRandomProblems` | ADMM 与穷举有效集一致（120 个随机问题） |
| `PathDeformation.SplineInterpolatesKnotsWithConsistentDerivatives` | 样条过节点，一阶 / 二阶导与差分一致，节点外为 0 |
| `TrajectoryPlanner.PlansUnderTheGateInSlotGate` | 规划成功、多初值中“向下”胜出、偏移向下（7–15 cm）、节点满足要求距离、重新计时后的速度 / 加速度 |
| `TrajectoryPlanner.PlannedPathWithCbfPassesUnderGateAndInserts` | 闭环：板 ~ 横梁 > 15 mm，两臂 > 30 mm，跟踪误差 < 20 mm，插槽终态 < 2 mm |
| `NoAlloc.PlannedTrajectoryControlLoopDoesNotAllocate` | 带路径变形的控制循环零动态分配 |
| `ContactGrasp.SlotGatePlanned` | 接触夹取完整任务 |
| 验收 `slot_gate` | `scripts/run_acceptance_suite.py` |

```bash
# 规划 + CBF（场景默认）
./build/run_sim --scene slot_gate --controller qp_coop --duration 22 --camera cam_front \
  --set simulation.contacts=false --set 'disturbances=[]'

# 对比：只用 CBF（逃逸方向向上；未打 MuJoCo 补丁时约 20 s 会崩溃，见 §9.6）
./build/run_sim --scene slot_gate --controller qp_coop --duration 26 --camera cam_front \
  --set simulation.contacts=false --set 'disturbances=[]' \
  --set planner.enabled=false --set controller.torque_qp.reference_governor.cbf.escape_gain=0.12

# 对比：只用力矩 QP 层
./build/run_sim --scene slot_gate --controller qp_coop --duration 22 --camera cam_front \
  --set simulation.contacts=false --set 'disturbances=[]' \
  --set planner.enabled=false --set controller.torque_qp.reference_governor.enabled=false

# 接触夹取 + 抓取对准 + 规划
./build/run_sim --scene slot_gate --controller qp_coop --contact-grasp --check-contact \
  --duration 26 --camera cam_front --set 'disturbances=[]'
```

每次运行的日志目录会多一个 `planned_path.csv`：名义时间 τ、执行时间、偏移 δ、每个节点的最小距离余量。

## 9 局限与后续

1. **仍是局部方法**：梯度只在“穿透方向”或“最近点法向”上提供信息。`slot_gate` 从名义初值出发就会停在局部解，
   靠 §4.3 的多初值（下 / 上 / ±x 四个鼓包）解决。候选方向之外的绕法（例如需要先后退、再从侧面绕）仍找不到。
   更一般的做法是工业流水线里的**采样规划器给初值**（RRT-Connect）再用 TrajOpt 平滑（方案 C）。
2. **姿态固定**：只优化平移。需要的话可以加入姿态偏移（李代数 $\xi\in\mathfrak{so}(3)$ 作为变量）。
3. **离散节点**：只在节点处约束距离，节点之间靠 `safety_margin`（10 mm）和节点间距（约 0.2 s × 0.08 m/s ≈ 16 mm）保证。
   严格的做法是 TrajOpt 的连续时间碰撞（扫掠体凸包）。
4. **时间参数化是近似的**：见 §6，可换成 TOPP-RA。
5. **静态环境、规划一次**：环境变化时需要重新规划。执行期的局部偏差由 CBF 与力矩 QP 处理。
6. **MuJoCo 3.12 的 EPA 缓冲区越界**：native CCD 的 EPA 把“地平线”（horizon）缓冲区固定为 24 个 int，
   而 `addEdge` 不做越界检查；`ccd_iterations = N` 时多面体最多有 5 + N 个顶点，N > 19 后，近乎面面平行的接触
   （例如板贴着槽壁 1.8 mm）可能产生超过 24 条边的地平线，写坏 mjData 栈，随后在 `mj_narrowphase → mj_freeStack` 段错误。
   本工程在 `collision.margin = 0.2` 和“只用 CBF”的对比运行中都复现过。
   - 修复：`patches/mujoco-3.12.0-epa-horizon-overflow.patch`（按多面体容量分配地平线缓冲区）。
     需要打到 `~/mujoco` 并重新安装到 `~/.local`：`cd ~/mujoco && git apply --recount <patch> && cmake --build build -j && cmake --install build`。
   - 把 `ccd_iterations` 降到 19 以下能从上界上保证不越界，但碰撞梯度精度会崩掉（有限差分误差达 280%），不可取。
   - 另外：全局 `collision.margin` 默认改为 0.12 m（0.2 m 时更容易触发），规划器只用它需要的约 0.09 m。

## 10 物体 SE(3) + 时间联合优化（`planner.formulation: full`）

§2–§6 的做法是“在名义轨迹附近只做侧向平移修正，再事后放慢”：姿态、时间、沿路径的推进都不在优化里。
`ObjectTrajectoryOptimizer` 把它们一起放进同一个问题（直接配点）。

### 10.1 变量与问题

起始保持段和最后一段插槽运动固定为名义值，其余节点：

- $p_k\in\mathbb R^3$：完整三维位置（不再限制侧向，因为时间已经是变量，不会“用偏移抵消推进”）；
- $\varphi_k$：姿态偏移，$R_k=\operatorname{Exp}(\varphi_k)\bar R_k$（世界系左乘），只在 `rotation_axes` 选中的世界系转轴上取值；
- $\Delta t_i$：区间时长，节点执行时刻 $t_k=\sum_{i<k}\Delta t_i$。

$$
\begin{aligned}
\min\quad & w_a h\sum\Big\|\tfrac{\Delta^2 p_k}{h^2}\Big\|^2+w_aL^2 h\sum\Big\|\tfrac{\Delta^2\varphi_k}{h^2}\Big\|^2+w_\varphi h\sum\|\varphi_k\|^2
  +w_T\sum\Delta t_i+w_\Delta\sum(\Delta t_{i+1}-\Delta t_i)^2\\
\text{s.t.}\quad & d_{kj}(p_k,R_k)\ge r_{kj}\qquad q_{\min}+\epsilon\le q_i(p_k,R_k)\le q_{\max}-\epsilon\\
& \|p_{i+1}-p_i\|\le v_{\max}\Delta t_i,\quad \|a_k\|\le a_{\max},\quad \|\operatorname{Log}(R_{i+1}R_i^\top)\|\le\omega_{\max}\Delta t_i\\
& \Delta t_i\in[0.4,\,3.0]\,h,\qquad |\varphi|\le\varphi_{\max}
\end{aligned}
$$

其中 $a_k=\big(\tfrac{p_{k+1}-p_k}{\Delta t_k}-\tfrac{p_k-p_{k-1}}{\Delta t_{k-1}}\big)/\tfrac{\Delta t_{k-1}+\Delta t_k}{2}$，
$L$（`rotation_length`，0.3 m/rad）把转角与长度放到同一尺度。

### 10.2 梯度

物体 twist $\xi=[\mathrm dp;\ J_l(\varphi)\mathrm d\varphi]$（$J_l$ 为 SO(3) 左雅可比）传到两臂关节：

$$
\mathrm dq_i=J_i^{+}\,G_i^\top\xi,\qquad G_i^\top=\begin{bmatrix}I&-[p_i-p_o]_\times\\0&I\end{bmatrix},\qquad
\frac{\partial d}{\partial\xi}=\sum_i\frac{\partial d}{\partial q_i}J_i^{+}G_i^\top .
$$

$G_i^\top$ 就是抓取矩阵的转置（`coop::graspMatrixArm`），§3 的平移梯度是它的前三列。关节余量约束的梯度就是 $\mathrm dq_i/\mathrm d\xi$ 的对应行；
速度、加速度、角速度约束的梯度解析给出（角速度用 $\mathrm d\rho\approx J_l(\varphi_{i+1})\mathrm d\varphi_{i+1}-J_l(\varphi_i)\mathrm d\varphi_i$ 近似）。
所有约束统一写成 $g(x)\le0$，按 §4 的方式线性化、加 ℓ1 罚与信赖域求解。

### 10.3 初值（仍然是局部方法）

1. **lateral**：§2–§4 只平移规划的结果（“下压穿梁”）。
2. **late-rotation**：自由窗口内保持起始姿态，窗口末尾按名义转动时长完成姿态变化，位置取名义值
   （“先平着穿过障碍，再在插槽上方滚转”）。

两个起点各跑一次序列凸规划，优先取可行解，再取代价最小者。`slot_gate` 中 late-rotation 胜出：
从局部解的角度看，“边转边走”与“先下压再转”是两个不同的盆地，梯度方法不会从一个自己走到另一个。

### 10.4 执行

`PathDeformation` 给出 $\delta(\tau)=p-p_{\text{nom}}$ 与 $\varphi(\tau)$ 的三次样条，以及由 $t_k$ 做单调三次 Hermite 插值
（Fritsch–Carlson）得到的 $\tau(t)$。`ObjectTrajectory::evaluate` 合成：

$$
R=\operatorname{Exp}(\varphi)R_{\text{nom}},\qquad
\omega=\dot\tau\big(J_l(\varphi)\varphi'+\operatorname{Exp}(\varphi)\,\omega_{\text{nom}}\big),
$$

角加速度忽略 $\mathrm dJ_l/\mathrm d\tau$ 项（仅作前馈）。`PathDeformation.RotationAndKnotTimeMapMatchNumericalDerivatives`
用数值微分检查速度、角速度和加速度。

### 10.5 结果（`slot_gate`，刚性抓取）

优化器找到的路径：**抬升的同时滚转到约 60°，以约 60° 的姿态从横梁下方穿过（此时板的竖直高度更小，只需轻微下压），
再边向插槽上方平移边完成滚转，然后插入**。规划约 3.6 s（含只平移规划作初值）。

| 指标 | 只平移规划（§7） | SE(3) + 时间联合优化 | 联合优化，速度上限与名义相同 |
|---|---:|---:|---:|
| 到达目标 | 15.49 s | **7.60 s** | 8.96 s |
| 最大跟踪误差 | 9.3 mm | 8.5 mm | 7.1 mm |
| 板 ~ 横梁最小距离 | 19.5 mm | 26.1 mm | 55.9 mm |
| 两臂 ~ 横梁最小距离 | 36.5 mm | 18.6 mm | 17.8 mm |
| 穿梁时的滚转角 | 90°（固定） | 约 60° | 约 80°→90° |

- 时间缩短的主要原因是“边转边走”和更好的姿态策略：把速度 / 加速度上限设成与名义航点相同（0.08 m/s、0.2 m/s²），
  仍从 15.49 s 缩短到 8.96 s（−42%）。
- 两臂离横梁比只平移方案近（约 18 mm，规划目标 20 mm）：距离只在节点处约束，执行中还有跟踪误差；仍远大于 10 mm 安全距离。
- 接触夹取 + 抓取对准 + 联合优化 + CBF：终态 0.26 mm / 0.04°，无失接触。

测试：`TrajectoryOptimizer.JointSe3TimeOptimizationRollsWhileMovingAndIsFaster`（可行、用上姿态自由度、快 > 3 s）、
`TrajectoryOptimizer.OptimizedTrajectoryWithCbfCompletesInsertion`（闭环完成插槽）、
`NoAlloc.PlannedTrajectoryControlLoopDoesNotAllocate`（带姿态偏移和节点时间映射时仍零分配）。

```bash
# 场景默认：联合优化
./build/run_sim --scene slot_gate --controller qp_coop --duration 12 --camera cam_front \
  --set simulation.contacts=false --set 'disturbances=[]'
# 对比：只平移规划
./build/run_sim --scene slot_gate --controller qp_coop --duration 22 --camera cam_front \
  --set simulation.contacts=false --set 'disturbances=[]' --set planner.formulation=lateral
```

局限：仍是局部方法（依赖初值）；距离只在节点处约束；角速度约束与角加速度前馈用了小角度近似；
没有把抓取力（摩擦锥、指垫抗扭）放进优化，这是下一步可以做的“接触感知”规划。

## 11 执行层偏差：为什么规划之后还需要 CBF

规划（无论 RRT 还是轨迹优化）保证的是**规划那一刻、在模型里**的参考路径不撞；执行时仍有规划器看不到的偏差：
环境模型误差、扰动、跟踪误差、节点之间的离散误差。本节在 `slot_gate` 上用联合优化规划好路径，再在执行时引入偏差，
对比三种执行配置（`scripts/run_execution_deviation.py`，报告写到 `artifacts/execution_deviation/<时间戳>/`）：

- **只按规划执行**：关闭 CBF 与力矩 QP 碰撞约束；
- **规划 + 力矩 QP**：只保留关节层的距离约束（velocity damper）；
- **规划 + CBF + 力矩 QP**：完整架构。

偏差：S1 规划器以为横梁比实际高 3 cm（`planner.model_error_offset`，执行层用真实位置，相当于“执行时的感知”与“规划时的地图”不一致）；
S2 穿梁时向上 20 N 推力（t = 3.8–4.8 s）；S3 两者叠加。距离均为真实环境下的最小有符号距离（负值 = 穿透），安全距离 10 mm。

| 偏差 | 执行配置 | 板~横梁 [mm] | 两臂~横梁 [mm] | < 9.5 mm 步数 | 最大跟踪误差 [mm] | 最大内力 [N] | 到达 [s] |
|---|---|---:|---:|---:|---:|---:|---:|
| 无偏差 | 规划 + CBF + QP | 26.1 | 18.6 | 0 | 8.5 | 7.2 | 7.60 |
| S1 模型误差 | 只按规划 | **−12.6** | 29.2 | 1459 | 11.4 | 0.8 | 7.91 |
|  | 规划 + QP | 10.1 | 30.4 | 0 | 21.4 | 13.0 | 8.62 |
|  | 规划 + CBF + QP | 10.6 | 34.7 | 0 | **12.6** | **7.2** | 8.17 |
| S2 推力 | 只按规划 | **−1.1** | 16.8 | 485 | 30.9 | 1.2 | 7.51 |
|  | 规划 + QP | 10.1 | 16.3 | 0 | 30.0 | 9.7 | 7.56 |
|  | 规划 + CBF + QP | 11.4 | 19.7 | 0 | 29.3 | **7.1** | 7.60 |
| S3 叠加 | 只按规划 | **−28.3** | 28.4 | 1459 | 34.0 | 1.8 | 7.91 |
|  | 规划 + QP | 9.1 | **12.3** | **9** | 53.6 | **36.3** | 7.94 |
|  | 规划 + CBF + QP | 10.6 | 34.5 | 0 | 45.1 | 28.3 | 8.65 |

接触夹取（S1，规划器以为横梁高 3 cm）：

| 执行配置 | 任务验收 | 失接触步数 | TCP–抓取点最大偏差 | 终态误差 |
|---|---|---:|---:|---:|
| 规划 + QP | **失败** | 3 | 7.2 mm | 3.0 mm / 0.68° |
| 规划 + CBF + QP | 通过 | 0 | 0.8 mm | 0.9 mm / 0.04° |

结论：

1. **只靠规划不够**：三种偏差下都穿透横梁（最深 28 mm）。规划给的是开环的安全，执行需要闭环的安全。
2. **力矩 QP 能兜底，但代价大**：它在关节加速度层“硬拦”，参考仍往障碍里走，于是跟踪误差和内力变大
   （S1 内力 13 N vs 7 N；S3 36 N vs 28 N），叠加偏差时还会轻微越界（S3 有 9 步低于 9.5 mm，两臂只剩 12 mm）。
   接触夹取时，这种“两臂顶着物体较劲”直接导致失接触、任务失败。
3. **CBF 在参考层做最小修正**：把参考压低或放慢（S1 最多修正 34 mm），控制器跟踪的就是安全的参考，
   距离守住、内力小、夹持不被破坏；代价是叠加偏差时稍慢（S3 8.65 s vs 7.94 s）。
4. **分工**：规划负责方向和大尺度路径（CBF 单独用会选错方向，§7）；CBF 负责执行中的安全与平滑修正；力矩 QP 负责最后兜底。

局限：基座标定误差不在本实验中。控制器的碰撞模型同样用名义基座，CBF 与力矩 QP 都看不到这类误差；
要覆盖它，需要用实测物体位姿（动捕 / 视觉）直接计算物体 ~ 障碍距离，而不是经名义运动学推出。

回归测试：`TrajectoryOptimizer.ExecutionLayerKeepsClearanceUnderPlannerModelError`（只按规划穿透、完整架构 ≥ 9.5 mm 且跟踪误差 < 20 mm）、
`ContactGrasp.SlotGateModelErrorWithCbf`（接触夹取 + 模型误差下任务验收通过）。

```bash
./scripts/run_execution_deviation.py   # 约 2 分钟，12 次仿真
```

## 参考文献

- J. Schulman et al., “Motion Planning with Sequential Convex Optimization and Convex Collision Checking”, IJRR, 2014（TrajOpt）。
- B. Stellato et al., “OSQP: An Operator Splitting Solver for Quadratic Programs”, Math. Prog. Comp., 2020。
- H. Pham, Q.-C. Pham, “A New Approach to Time-Optimal Path Parameterization Based on Reachability Analysis”, IEEE T-RO, 2018（TOPP-RA）。
