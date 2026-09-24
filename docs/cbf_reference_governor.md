# CBF 参考滤波：连续的在线避障参考生成

本文说明 `CbfReferenceFilter`（`include/dual_arm/cbf_reference_filter.hpp`）的问题定义、推导、QP 形式、
已知陷阱与验收标准。它替代 `QpCoopController` 里 `NORMAL → LIFT → CROSS → DESCEND` 的状态机，
是 `slot_avoid` 的默认模式（`controller.torque_qp.reference_governor.mode: cbf`）；
状态机仍可用 `mode: state_machine` 选择，并保留独立的验收项 `slot_avoid_state_machine`。

---

## 1 定位

```
原始物体轨迹 p_nom(s) ──► CBF 参考滤波（本文）──► 协同控制律 CoopController ──► 力矩 QP（velocity damper 兜底）
        ▲                    │ δ, ṡ
        └── 虚拟时间 s ◄──────┘
```

- 上层（本文）：在**物体参考**上做最小修改，使参考轨迹对障碍保持安全；方向由几何与代价决定，不再是离散状态。
- 下层（已有）：力矩 QP 中的距离约束 $J_d\ddot q \ge (\dot d_\text{allow}-\dot d)/\Delta t$，处理跟踪误差和手臂本体的避碰。

与状态机的对应关系见 §7。

## 2 变量与参考合成

- $s$：原始轨迹的虚拟时间；$\dot s\in[0,1]$，$\dot s<1$ 表示放慢，$\dot s=0$ 表示停住。
- $\delta\in\mathbb R^3$：物体参考的平移偏移（世界系）；姿态不偏移。
- 每周期决策变量 $u=[\dot\delta;\ \dot s]\in\mathbb R^4$。

设原始轨迹在 $s$ 处的参考为 $p_\text{nom},R_\text{nom},v_\text{nom},\omega_\text{nom},a_\text{nom},\alpha_\text{nom}$，则滤波后的参考为

$$
\begin{aligned}
p_\text{ref} &= p_\text{nom}(s)+\delta, & R_\text{ref} &= R_\text{nom}(s),\\
v_\text{ref} &= \dot s\,v_\text{nom}+\dot\delta, & \omega_\text{ref} &= \dot s\,\omega_\text{nom},\\
a_\text{ref} &\approx \dot s^2 a_\text{nom}+\dfrac{\dot\delta_k-\dot\delta_{k-1}}{\Delta t}, & \dot\omega_\text{ref} &\approx \dot s^2\alpha_\text{nom}.
\end{aligned}
$$

严格地 $a_\text{ref}$ 还有 $\ddot s\,v_\text{nom}$ 项；变化率约束 $|\Delta\dot s|\le a_s\Delta t$ 使它有界，这里忽略。
本周期用 $\delta_k,u_k$ 合成参考，然后积分 $\delta_{k+1}=\delta_k+\dot\delta\Delta t$、$s_{k+1}=s_k+\dot s\Delta t$
（`update()` 已实现）。

## 3 屏障函数

对障碍组 `reference_governor.obstacle_group`（必须写成 `[object, <障碍>]`）中距离最近的至多 8 个 geom 对：

$$
h_j = d_j - d_s ,
$$

$d_j$ 是 `CollisionModel` 在**实测**物体位姿下的有符号距离，$n_j$ 为从物体指向障碍的单位法向，
$p_j$ 为物体上的最近点（`CbfObstacle::{distance, normal, point}`）。

物体上一点 $p_j$ 在参考运动下的速度（$p_o$ 为物体原点的实测位置）：

$$
\dot p_j = \dot s\,\underbrace{\big(v_\text{nom}+\omega_\text{nom}\times(p_j-p_o)\big)}_{w_j} + \dot\delta ,
\qquad
\dot h_j = -n_j^\top \dot p_j .
$$

一阶 CBF 条件 $\dot h_j\ge-\alpha h_j$ 化为**关于 $u$ 的线性不等式**：

$$
\boxed{\; n_j^\top\dot\delta + (n_j^\top w_j)\,\dot s \;\le\; \alpha\, h_j \;}
$$

- 离散时间：前向 Euler 下 $h_{k+1}\approx h_k+\Delta t\,\dot h_k\ge(1-\alpha\Delta t)h_k$，
  需要 $\alpha\Delta t<1$（构造函数已检查）。
- $h$ 在实测位姿下计算，而约束施加在参考速度上，这隐含“跟踪误差小”的假设。
  可选的改进是用 $h_j^\text{ref}=h_j-n_j^\top(p_\text{ref}-p_\text{act})$（当前未采用：实测跟踪误差 ≤ 11 mm，墙距离仍精确保持 10.00 mm）。跟踪误差造成的残余由下层 QP 兜底。

偏移上限写成屏障 $h_\delta=\delta_\max^2-\|\delta\|^2$：

$$
2\delta^\top\dot\delta \le \alpha\,(\delta_\max^2-\|\delta\|^2).
$$

## 4 每周期 QP

$$
\begin{aligned}
\min_{u=[\dot\delta;\dot s]}\quad & \tfrac12 w_\delta\|\dot\delta-\dot\delta_\text{nom}\|^2+\tfrac12 w_s(\dot s-1)^2\\
\text{s.t.}\quad
& n_j^\top\dot\delta+(n_j^\top w_j)\dot s\le\alpha h_j, && j=1..m\ (m\le 8)\\
& 2\delta^\top\dot\delta\le\alpha(\delta_\max^2-\|\delta\|^2)\\
& |\dot\delta_i|\le v_\max,\quad 0\le\dot s\le 1\\
& |\dot\delta-\dot\delta_\text{prev}|\le a_\delta\Delta t,\quad |\dot s-\dot s_\text{prev}|\le a_s\Delta t\\
& \hat v^\top\dot\delta = -k_r\,\hat v^\top\delta,\qquad \hat v=v_\text{nom}/\|v_\text{nom}\|\ \ (\|v_\text{nom}\|>0)
\end{aligned}
$$

最后一条是**分工约束**：$\delta$ 只做侧向绕行，沿路径方向的分量只按 $-k_r$ 衰减；沿路径的进度只由 $\dot s$ 负责（原因见 §6）。

**实现方式**：

- 分工等式用参数化消去：$\dot\delta=c_v\hat v+B z$，$B=[b_1\ b_2]$ 为 $\hat v$ 的正交补，$c_v=-k_r\hat v^\top\delta$
  （限幅到 $\pm v_\max/2$）；决策变量降为 $y=[z;\dot s]\in\mathbb R^3$（$\|v_\text{nom}\|<1$ mm/s 时不加分工约束，$y=u\in\mathbb R^4$）。
  $B$ 列正交归一且与 $\hat v$ 正交，所以 $y$ 空间的代价仍是对角的。
- 对角 Hessian 的 QP 经 $W^{1/2}$ 缩放后，就是“目标点到多面体的欧氏投影”，即最小距离问题（LDP）。
  `small_qp::solveDiagonalQp`（`src/small_qp.cpp`）按 Lawson & Hanson 用 NNLS 求解：有限步的有效集法，结果精确，
  并能显式判定不可行。它全部使用定长 Eigen，不做动态分配，单元测试 `SmallQp.*` 用穷举有效集的暴力解逐一对照。
- 分级松弛：约束不可同时满足时，先去掉变化率约束，再去掉偏移上限屏障，CBF 行从不松弛；仍无解则返回 `Infeasible`（参考停住）。
- 不宜用 Hildreth 型对偶坐标上升：它对近似等式约束收敛很慢，不适合 1 kHz。

## 5 死锁与逃逸项

**纯 CBF-QP 在墙前会死锁**：墙面法向 $n$ 与前进方向相反时，垂直于 $n$ 的 $\dot\delta$ 既不降低代价也不帮助满足约束，
最优解就是 $\dot s\to0$，参考停在墙前。这是 CBF-QP 的已知性质：它会引入非期望的渐近稳定平衡点
（见 Reis, Aguiar, Tabuada, “Control Barrier Function-Based Quadratic Programs Introduce Undesirable Asymptotically
Stable Equilibria”, IEEE L-CSS 2021）。

解决办法是在名义偏移速度里加入**切向逃逸项**：

$$
\dot\delta_\text{nom} = (1-\sigma)\,(-k_r\,\delta) + \sigma\, g\,\hat e_t,
\qquad
\hat e_t=\frac{(I-n n^\top)e}{\|(I-n n^\top)e\|},
\qquad
\sigma=\sigma_h\,\sigma_a ,
$$

$$
\sigma_h=\operatorname{clamp}\!\Big(1-\frac{h_\min}{h_\text{act}},0,1\Big),\qquad
\sigma_a=\operatorname{clamp}\!\big(n^\top\hat v_\text{nom},0,1\big).
$$

- $n$ 取当前最小 $h$ 的法向，$e$ 为配置的偏好方向（`escape_direction`），$g$ 为 `escape_gain`，$h_\text{act}$ 为 `escape_activation`。
- $\sigma_a$ 不可省：它让逃逸项只在参考**正朝障碍运动**时起作用。验证时如果去掉它，越墙后板在槽里、离墙仍在 7 cm 以内，
  逃逸项会一直往上抬，导致插槽失败。
- 逃逸项只**影响代价**，安全性完全由 CBF 约束保证；$e$ 与 $n$ 平行时 $\hat e_t$ 无定义，此时不逃逸。
- 更一般的做法（可作为后续研究）：从候选方向中按代价选择 $e$，或改成短时域 MPC 让前瞻自然解决死锁。

## 6 可行性与参数相容性

1. **CBF 与变化率约束**：参考以名义速度 $v$ 接近障碍时，CBF 要求 $\dot s$ 按 $\dot s\,\alpha$ 的速率衰减，
   因此需要 $a_s\ge\alpha$，否则刹不住、QP 不可行。默认 $\alpha=2$、$a_s=2.5$。
2. **偏移吸收进度**：如果不加 §4 的分工约束，$w_s$ 较小时，让 $\dot\delta$ 沿 $-v_\text{nom}$ 抵消推进比降低 $\dot s$ 更“便宜”。
   $\delta$ 会一直向后滑到 $\|\delta\|=\delta_\max$，随后偏移屏障与障碍屏障同时起作用而不可行（验证中出现过 0.7 mm 穿透）。
3. **不可行时的处理**：优先保留 CBF，先松弛变化率约束，返回 `Status::Relaxed`；确实无解时返回 `Status::Infeasible`，
   `update()` 会令 $\dot\delta=0,\ \dot s=0$，参考停住，由下层 QP 兜底。
4. **参考与实测的偏差**：$h$ 由实测位姿给出；跟踪误差大时，可按 §3 的 $h^\text{ref}$ 修正，或增大 $d_s$。

## 7 与原状态机的对应

| 状态机 | CBF 参考滤波中的对应机制 |
|---|---|
| NORMAL | $\sigma=0$、CBF 不起作用：$\delta\to0$，$\dot s=1$ |
| LIFT（冻结轨迹 + 抬升） | CBF 使 $\dot s$ 连续下降；逃逸项把 $\delta$ 推向 $\hat e_t$ |
| CROSS | 物体越过墙顶时，最近点法向转为向下，CBF 阻止 $\delta$ 下降；$\dot s$ 恢复 |
| DESCEND | $\sigma_a\to0$ 后回复项 $-k_r\delta$ 起作用，CBF 保证下降过程不碰墙 |
| 方向 `preferred_direction` | `escape_direction`，但只作为代价偏好，且被投影到障碍切平面 |

## 8 验收与结果

测试：

| 测试 | 内容 |
|---|---|
| `CbfReferenceFilter.PassesNominalReferenceThroughWithoutObstacles` | 无障碍时参考与原轨迹完全一致，$s=t$ |
| `CbfReferenceFilter.BarrierStaysNonNegativeInFrontOfInfiniteWall` | 无限高墙：全程 $h\ge-0.1$ mm，参考停在墙前 |
| `CbfReferenceFilter.ClimbsOverFiniteWallWithoutDeadlockAndReturnsToPath` | 有限高墙：不死锁、越过墙顶、之后偏移回到 < 1 mm |
| `CbfReferenceFilter.RespectsOffsetRateAndTimeScalingBounds` | $\|\delta\|$、$\|\dot\delta\|_\infty$、两种变化率、$\dot s\in[0,1]$ |
| `CbfReferenceFilter.SlotAvoidTransportClearsBarrierAndInserts` | 完整 `slot_avoid`（刚性抓取、关闭接触）：板与墙 $\ge d_s-0.5$ mm，终态 < 2 mm / 0.57° |
| `NoAlloc.CbfGovernorControlLoopDoesNotAllocate` | CBF 模式控制循环零动态分配 |
| `SmallQp.MatchesBruteForceOnRandomProblems` / `ReportsInfeasibility` | LDP/NNLS 求解器与穷举有效集一致，并能正确判定不可行 |
| `ContactGrasp.SlotAvoidance` | 接触夹取 + 抓取对准 + CBF 的完整任务 |

`slot_avoid` 刚性抓取（关闭接触、20 s）两种模式的对比：

| 指标 | 状态机 | CBF |
|---|---:|---:|
| 板 ~ 墙最小距离 | 10.00 mm | 10.00 mm |
| 板 ~ 槽框最小距离 | 0.90 mm | 1.01 mm |
| 最大位置跟踪误差 | 14.0 mm | 11.1 mm |
| 最大偏移 | 150 mm（固定抬升量） | 133.5 mm（几何需要多少抬多少） |
| 到达目标（2 mm 内） | 14.62 s | 14.45 s |
| 控制器 P99 耗时 | 300 µs | 309 µs |

接触夹取版本：终态 1.03 mm / 0.03°，无失接触，TCP–抓取点最大偏差 0.97 mm（状态机为 1.38 mm）。
随机鲁棒性扫描（`scripts/run_robustness_sweep.py`，标定误差 + 扰动，12 组）全部通过。

`return_gain` 的取值：0.8 时越墙后偏移按指数回落、拖尾约 3 s，到达目标晚于状态机；
2.5 以上因为参考变化快、跟踪滞后，反而更慢；1.5 最快。

```bash
cmake --build build -j"$(nproc)"
ctest --test-dir build -R "Cbf" --output-on-failure

# 可视化（刚性抓取基线）
./build/run_sim --scene slot_avoid --controller qp_coop --duration 24 --camera cam_front \
  --set simulation.contacts=false --set 'disturbances=[]' \
  --set controller.torque_qp.reference_governor.mode=cbf

# 接触夹取版本
./build/run_sim --scene slot_avoid --controller qp_coop --contact-grasp --check-contact --duration 24 \
  --camera cam_front --set 'disturbances=[]' --set controller.torque_qp.reference_governor.mode=cbf
```

日志中 `governor_phase` 在 CBF 起作用时记为 4（`CBF`），`governor_offset` 为 $\|\delta\|$，`governor_virtual_time` 为 $s$。


## 9 参数（`controller.torque_qp.reference_governor.cbf`）

| 参数 | 默认 | 含义 |
|---|---|---|
| `safe_distance` | 0.010 m | $d_s$ |
| `alpha` | 2.0 1/s | 线性 class-K 增益 |
| `offset_max` | 0.20 m | $\delta_\max$ |
| `offset_speed_max` | 0.15 m/s | $v_\max$ |
| `offset_accel_max` | 0.6 m/s² | $a_\delta$ |
| `time_rate_accel_max` | 2.5 1/s | $a_s$，需 $\ge\alpha$ |
| `return_gain` | 1.5 1/s | $k_r$ |
| `offset_weight` / `time_rate_weight` | 1.0 / 0.05 | $w_\delta$ / $w_s$ |
| `escape_direction` | $+z$ | $e$ |
| `escape_gain` | 0.12 m/s | $g$ |
| `escape_activation` | 0.08 m | $h_\text{act}$ |

碰撞模型的查询范围会自动扩到 `escape_activation + safe_distance + 0.01`，保证 CBF 看得到障碍。

## 11 与轨迹规划配合

有全局规划（`docs/trajectory_planning.md`）时，CBF 关闭主动绕行（`escape_gain: 0`），只做执行层安全滤波。
规划之后仍需要它的实验依据见该文档 §11：地图误差、扰动下，只按规划执行会穿透障碍；只靠力矩 QP 兜底时内力更大、
接触夹取会失接触；加上 CBF 后安全距离全部守住。

## 参考文献

- C. L. Lawson, R. J. Hanson, *Solving Least Squares Problems*, Prentice-Hall, 1974（第 23 章 NNLS 与 LDP）。

- A. D. Ames, X. Xu, J. W. Grizzle, P. Tabuada, “Control Barrier Function Based Quadratic Programs for Safety Critical Systems”, IEEE TAC, 2017.
- B. Faverjon, P. Tournassoud, “A local based approach for path planning of manipulators with a high number of degrees of freedom”, ICRA, 1987.
- M. F. Reis, A. P. Aguiar, P. Tabuada, “Control Barrier Function-Based Quadratic Programs Introduce Undesirable Asymptotically Stable Equilibria”, IEEE L-CSS, 2021.
