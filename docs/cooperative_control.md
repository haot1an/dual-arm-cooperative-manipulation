# 双臂闭链协同操作：原理笔记

> 本文配合 `include/dual_arm/coop_kinematics.hpp`、`coop_controller.hpp` 阅读。代码注释中的
> “式 x.y” 即本文公式编号。所有量都标注了**维度、表达坐标系、参考点**——协同控制里
> 80% 的 bug 来自这三样没对齐。

---

## 0. 本仿真环境里的具体设定

| 量 | 值 |
|---|---|
| 世界系 $W$ | 原点在地面、两基座连线中点正下方，$z$ 向上 |
| 工作台 | 台面高 0.80 m，只作尺度参照（不参与接触） |
| 左臂 arm 1 / 右臂 arm 2 | 带 Franka Hand，基座装在台面上：$(-0.745,0,0.80)$ 朝 $+x$ / $(+0.745,0,0.80)$ 朝 $-x$ |
| 箱子 | $0.40\times0.07\times0.12$ m，2 kg，中心 $p_o=(0,0,0.885)$，长轴沿 $x$，底面在台面上方 2.5 cm |
| 抓取点 | 距两端面 3 cm、顶面下 2 cm：$p_1=(-0.17,0,0.925)$，$p_2=(0.17,0,0.925)$ |
| 末端点（TCP） | `ee_site`：两指之间、法兰前方 0.1034 m；$z$ 轴竖直向下（接近方向），$y$ 轴为手指开合方向 |
| 刚性夹持 | 夹爪竖直向下夹住箱子两侧（手指固定、与侧面留 1 mm），受力完全由 MuJoCo `weld`（hand ↔ box）传递，见 §8 |

两抓取点连线沿世界 $x$ 轴，所以下文“沿连线方向”就是 $x$ 方向。

---

## 1. 问题动机：闭链把位置误差变成内力

单臂搬东西时，末端位置误差只会让物体位置不准。**两臂刚性夹住同一个箱子后**，
“左臂—箱子—右臂—地面—左臂”构成一条**闭合运动链**，箱子的 6 个自由度被两只手重复约束了
一次（12 个约束对 6 个自由度），多出来的 6 维约束方向上任何“不一致”都无法通过运动消解，
只能变成**内力**（挤压 / 拉伸 / 扭转 / 弯曲），而物体本身一动不动。

最简单的一维模型：两臂各自用位置伺服控制末端，在连线方向上的等效刚度分别为 $k_1,k_2$，
箱子自身刚度 $k_o$。若两臂**认为**的抓取点间距与真实间距差 $\delta x_r$（例如右臂基座标定差了
几毫米），那么闭链串联后的内力约为

$$
f_\text{int} \approx k_\text{eq}\,\delta x_r,\qquad
k_\text{eq} = \left(k_1^{-1} + k_2^{-1} + k_o^{-1}\right)^{-1}. \tag{1.1}
$$

**伺服刚度越高，内力越大**——这与“位置控制越准越好”的直觉相反。量级（示意）：

| 场景 | 单臂等效刚度 | $k_\text{eq}$（刚性箱子） | $\delta x_r=3$ mm 时的内力 |
|---|---|---|---|
| 高增益位置伺服 | $5\times10^4$ N/m | $2.5\times10^4$ N/m | ≈ 75 N |
| 中等笛卡尔阻抗 | $3\times10^3$ N/m | $1.5\times10^3$ N/m | ≈ 4.5 N |
| 本环境的基线关节 PD（$K_p=600$） | ≈ $1.35\times10^3$ N/m（$x$ 向，在 $q_\text{init}$ 处由 $(J K_q^{-1}J^\top)^{-1}$ 的对角近似算得） | ≈ $0.68\times10^3$ N/m | ≈ 2 N |

真实系统里 75 N 足以压坏纸箱、触发碰撞检测或让腕部 F/T 饱和。姿态误差同理：
0.3° 的基座 yaw 误差在 0.575 m 前伸处就是 3.0 mm 的横向误差，还会叠加一个扭转内力矩。

**本环境实测**（`--calib-error --set weld.init_mode=nominal`，默认误差为 3/−3/2 mm + 0.3° yaw，
基线关节 PD，3 s 后稳态）：左右臂 weld 力相对无误差情况多出 $\Delta f\approx(-13.1,\,1.5,\,-0.6)$ N 与
$(+13.1,\,-1.5,\,+0.6)$ N，另有绕 $y$ 约 $\mp3.1$ N·m 的附加弯矩——大小相等、方向相反，合力不变，
典型的内力。比 (1.1) 的一维估计大得多，是因为 3 个平移误差分量与 0.3° 姿态误差通过机械臂柔度矩阵
（6×6，平移与转动强耦合）耦合在一起，weld 还要强制两端姿态一致。

还有一个容易被忽略的来源：**即使没有标定误差**，只要两臂各自柔顺，物体重量引起的变形
也会产生内力。本环境中两臂用关节 PD 托住 2 kg 箱子时，两臂柔度矩阵把竖直载荷耦合成水平
变形，而两臂的水平变形方向相反，于是闭链中出现约 7.5 N 的沿连线拉伸和 ±0.23 N·m 的弯矩
（`tests/test_calibration.cpp` 的注释里有记录；符号与大小取决于构型）。协同控制要解决的正是这两类问题：
**把“物体怎么动”和“两手之间怎么较劲”分开控制**。

---

## 2. 记号与约定

- arm $i\in\{1,2\}$（1 = 左，2 = 右），关节角 $q_i\in\mathbb R^7$，堆叠 $q=[q_1;q_2]\in\mathbb R^{14}$。
- 物体坐标系 $o$：原点 $p_o$ 在箱子中心（= 质心），姿态 $R_o$。
- 抓取点 $p_i$：与 `ee_site` 原点重合。**虚拟杆**向量

$$ r_i := p_o - p_i \in \mathbb R^3\quad(\text{世界系，从抓取点指向物体中心}). \tag{2.1}$$

  注意：有些书用 $p_i-p_o$，下面所有 $S(r_i)$ 项的符号会随之翻转。
- 反对称矩阵 $S(a)b = a\times b$，$S(a)^\top = -S(a)$。
- **Twist**（速度旋量）$\nu=[v;\omega]\in\mathbb R^6$：线速度在前。线速度 $v$ 与**参考点**有关，
  角速度 $\omega$ 与参考点无关。
- **Wrench**（力旋量）$h=[f;m]\in\mathbb R^6$：力在前。力矩 $m$ 与**参考点**有关。
- **表达坐标系**：本工程所有 6 维量默认在**世界系**下投影。

**换参考点**（表达坐标系不变，$d := p_B - p_A$）：

$$
\nu_B = \begin{bmatrix} I & -S(d)\\ 0 & I\end{bmatrix}\nu_A =: X(d)\,\nu_A,\qquad
h_B = \begin{bmatrix} I & 0\\ -S(d) & I\end{bmatrix} h_A = X(d)^{-\top} h_A. \tag{2.2}
$$

即 $v_B = v_A + \omega\times(p_B-p_A)$，$m_B = m_A + (p_A-p_B)\times f$。二者互为“伴随/对偶”：
功率 $h^\top\nu$ 与参考点无关（$h_B^\top\nu_B = h_A^\top X^{-1}X\nu_A = h_A^\top\nu_A$）。
代码里对应 `shiftTwistRefPoint` / `shiftWrenchRefPoint`（`math_utils.hpp`，已实现并测试）。

**换表达坐标系**（参考点不变）：$\nu^{(B)} = \mathrm{blkdiag}(R,R)\,\nu^{(A)}$，wrench 同理。
MuJoCo 自由关节的角速度 `qvel[3:6]` 是**局部系**表达的，F/T 传感器读数在**site 系**表达，
`mj_objectVelocity` 输出顺序是 **[ω; v]**——`SimEnv` 已经全部换成本约定。

**雅可比约定**（`RobotModel::jacobian`）：$J_i\in\mathbb R^{6\times7}$，$[v_i;\omega_i]=J_i\dot q_i$，
线速度在前、参考点为 `ee_site` 原点、世界系表达（`tests/test_jacobian.cpp` 用有限差分验证）。

---

## 3. 运动学关系

### 3.1 刚性抓取：末端 twist 由物体 twist 决定

刚性抓取时末端 $i$ 与物体是同一个刚体，末端 twist 就是物体 twist 换到参考点 $p_i$：
$v_i = v_o + \omega_o\times(p_i-p_o) = v_o + S(r_i)\,\omega_o$，$\omega_i=\omega_o$。写成矩阵：

$$
\nu_i = G_i^\top \nu_o, \tag{3.1}
$$

$$
G_i = \begin{bmatrix} I_3 & 0\\ -S(r_i) & I_3\end{bmatrix}\in\mathbb R^{6\times 6},\qquad
G_i^\top = \begin{bmatrix} I_3 & S(r_i)\\ 0 & I_3\end{bmatrix}. \tag{3.2}
$$

$G_i$ 就是式 (2.2) 中“把 wrench 从 $p_i$ 搬到 $p_o$”的矩阵，所以同一个 $G_i$ 在静力学里
把末端 wrench 映射到物体中心（§4）——运动学与静力学互为转置，这是后面一切的根源。

### 3.2 虚拟杆、绝对运动与相对运动（Uchiyama 对称形式）

把每只手的末端 twist “沿虚拟杆”推到物体中心：

$$
\tilde\nu_i := G_i^{-\top}\nu_i = \begin{bmatrix} I & -S(r_i)\\ 0 & I\end{bmatrix}\nu_i. \tag{3.3}
$$

$\tilde\nu_i$ 是“arm $i$ 认为物体中心在怎么动”。理想刚性闭链下 $\tilde\nu_1=\tilde\nu_2=\nu_o$。定义

$$
\text{绝对 twist:}\quad v_a := \tfrac12(\tilde\nu_1+\tilde\nu_2), \tag{3.4}
$$
$$
\text{相对 twist:}\quad v_r := \tilde\nu_2-\tilde\nu_1. \tag{3.5}
$$

$v_a$ 描述**物体怎么动**；$v_r$ 描述**两只手之间的相对运动**，理想闭链下恒为 0。
代入 $\nu_i=J_i\dot q_i$：

$$
v_a = J_a\dot q,\qquad J_a = \tfrac12\begin{bmatrix} G_1^{-\top}J_1 & G_2^{-\top}J_2\end{bmatrix}\in\mathbb R^{6\times14}, \tag{3.6}
$$
$$
v_r = J_r\dot q,\qquad J_r = \begin{bmatrix} -G_1^{-\top}J_1 & G_2^{-\top}J_2\end{bmatrix}\in\mathbb R^{6\times14}. \tag{3.7}
$$

**闭链约束**：

$$
J_r\,\dot q = 0. \tag{3.8}
$$

它的含义：14 个关节速度中只有满足 (3.8) 的那部分（14 − 6 = 8 维，其中 6 维驱动物体、
2 维是两臂各自的冗余零空间）是“允许的”；违反 (3.8) 的运动要么被箱子挡住（变成内力），
要么撕开抓取。位置级对应的是“两臂各自由正运动学算出的物体位姿必须一致”——标定误差恰恰
破坏了这一点：控制器以为 (3.8) 成立，实际上闭链在内部被拉扯。

> Caccavale、Chiacchio 等人的形式用两末端坐标系定义相对位置/姿态（相对姿态表达在
> 末端 1 系中），绝对坐标系取两末端的“平均”；和上面的对称形式本质相同，只是参考点与坐标系
> 的选择不同。实现时选一种，**全程一致**即可。

### 3.3 由物体目标运动得到两臂末端期望量

给定物体参考 $T_{o,\text{des}}, \nu_{o,\text{des}}, \dot\nu_{o,\text{des}}$（`ObjectTrajectory`），以及抓取点
在物体系中的固定位姿 $T_{o,g_i}$（`RobotModel::objectParams().grasp_in_object`）：

$$ T_{i,\text{des}} = T_{o,\text{des}}\,T_{o,g_i}, \tag{3.9}$$
$$ \nu_{i,\text{des}} = G_i^\top\nu_{o,\text{des}}, \tag{3.10}$$
$$ \dot\nu_{i,\text{des}} = G_i^\top\dot\nu_{o,\text{des}} + \begin{bmatrix}-\omega_o\times(\omega_o\times r_i)\\ 0\end{bmatrix}, \tag{3.11}$$

其中 $r_i$ 用期望位姿计算。(3.11) 最后一项是向心加速度，别漏。

---

## 4. 静力学关系：协同作用的核心

### 4.1 物体受到的合 wrench

设 $h_i$ 是 arm $i$ 的末端**施加给物体**的 wrench（世界系，参考点 $p_i$），$h=[h_1;h_2]\in\mathbb R^{12}$。
物体中心受到的合 wrench：

$$
w_o = \sum_i G_i h_i = G\,h,\qquad G = \begin{bmatrix}G_1 & G_2\end{bmatrix}\in\mathbb R^{6\times12}. \tag{4.1}
$$

物体动力学（参考点 = 质心，世界系）：

$$
\begin{bmatrix} m_o\dot v_o\\ I_o\dot\omega_o + \omega_o\times I_o\omega_o\end{bmatrix}
= G h + \begin{bmatrix} m_o g\\ 0\end{bmatrix} + w_\text{ext}. \tag{4.2}
$$

### 4.2 解的分解：外力部分 + 内力部分

(4.1) 是 6 个方程、12 个未知数。给定想要的 $w_o$，所有满足 $Gh=w_o$ 的解为

$$
h = G^{+}w_o + \left(I_{12}-G^{+}G\right)h_\text{int}, \tag{4.3}
$$

$G^{+}$ 是任意右逆（§5），$h_\text{int}\in\mathbb R^{12}$ 任意。第二项在 $G$ 的零空间里：
$G(I-G^+G)=G-G=0$，**不改变物体受到的合 wrench，也就不改变物体运动**——它就是内力。
反过来，给定测量值 $h_\text{meas}$（腕部 F/T 扣除夹爪重力后，或仿真真值 weld wrench），

$$
h_\text{int,meas} = \left(I_{12}-G^{+}G\right)h_\text{meas} \tag{4.4}
$$

就是“内力测量”（代码：`coop::internalWrench`，日志 `int_*` 列）。

**零空间是 6 维**：$\operatorname{rank}G=6$（$G_i$ 可逆），所以 $\dim\mathcal N(G)=12-6=6$。（式 4.5）

这 6 维内力的直观含义（以本场景连线沿 $x$ 为例）：

| 内力方向 | 物理含义 |
|---|---|
| 沿连线的一对相反力 | 挤压 / 拉伸（夹持力） |
| 垂直连线的一对相反力（$y$ 或 $z$） | 剪切；由于力臂 $2|r_i|$ 会产生力矩，需要配合相反的力矩才能平衡 |
| 绕连线的一对相反力矩 | 扭转（拧毛巾） |
| 绕 $y$ / $z$ 的一对相反力矩 | 弯曲（掰树枝） |

**例子**：两臂沿连线方向对挤——左手在 $p_1$ 施加 $f_1=(30,0,0)$ N，右手在 $p_2$ 施加 $f_2=(-30,0,0)$ N，
末端力矩为 0。两个力对 $p_o$ 各有 $\pm1.2$ N·m 的力矩（抓取点比 $p_o$ 高 4 cm），恰好抵消，
所以 $G h = (0,0,0,\;0,0,0)$：**纯内力**，箱子不动但被夹紧。
若两手同方向推，$f_1=f_2=(30,0,0)$，则 $Gh=(60,0,0,0,0,0)$：箱子受 60 N 合力、加速运动。
（`tests/test_coop_template.cpp::SqueezingIsPureInternalForce`）

### 4.3 运动—力对偶：为什么“调相对方向的柔顺”就是在调内力

功率 $P=h_1^\top\nu_1+h_2^\top\nu_2$。把 $\nu_i=G_i^\top\tilde\nu_i$、
$\tilde\nu_1=v_a-\tfrac12v_r$、$\tilde\nu_2=v_a+\tfrac12v_r$ 代入：

$$
P = \underbrace{(G_1h_1+G_2h_2)}_{h_a}{}^\top v_a + \underbrace{\tfrac12(G_2h_2-G_1h_1)}_{h_r}{}^\top v_r ,
$$

$$
h_a := G h = w_o, \tag{4.6}
$$
$$
h_r := \tfrac12\left(G_2h_2 - G_1h_1\right)\in\mathbb R^6. \tag{4.7}
$$

所以 $(h_a, v_a)$ 是一对功率共轭量，$(h_r, v_r)$ 是另一对。反解：

$$
h_1 = G_1^{-1}\!\left(\tfrac12h_a - h_r\right),\qquad h_2 = G_2^{-1}\!\left(\tfrac12h_a + h_r\right). \tag{4.8}
$$

$h_a=0$ 时 $h = V h_r$，$V=\begin{bmatrix}-G_1^{-1}\\ G_2^{-1}\end{bmatrix}\in\mathbb R^{12\times6}$，$GV=0$：
**$h_r$ 就是 6 维内力空间的一组坐标**（式 4.9；代码：`coop::internalWrenchCoordinates`）。

结论：

- 物体运动只由 $h_a$ 决定；$h_r$ 只做“无用功”——理想闭链下 $v_r=0$，$h_r^\top v_r=0$。
- 在相对空间做阻抗 $h_r = K_r e_r + D_r\dot e_r$（$e_r$ 为相对位姿误差），就是用一个弹簧把
  “两手之间的不一致”转化为内力：$K_r$ 小 → 标定误差只产生小内力；这就是 §1 表格里“刚度
  越高内力越大”的控制器版本。
- 反之，直接闭环 $h_r$（力控）可以让内力收敛到期望的夹持力，与位置误差无关。

符号约定：本场景 $r_1=(0.17,0,-0.04)$、$r_2=(-0.17,0,-0.04)$，对挤时 $h_{r,x}=\tfrac12(-30-30)=-30<0$，
即 **$h_r$ 沿“左→右”方向的分量为负表示挤压、为正表示拉伸**。

---

## 5. $G^+$ 的选择与负载分配

(4.3) 中的 $G^+$ 不唯一，它决定了“外力部分”怎么分给两臂，也决定了“什么算内力”。

**Moore–Penrose 伪逆**（最小 $\|h\|_2$）：

$$ G^{+} = G^\top(GG^\top)^{-1}. \tag{5.1}$$

**加权伪逆**（最小 $h^\top W h$，$W\succ0$）：

$$ G_W^{+} = W^{-1}G^\top\left(GW^{-1}G^\top\right)^{-1}. \tag{5.2}$$

**按负载能力分配**：取 $W=\mathrm{blkdiag}(\lambda_1^{-1}I_6,\ \lambda_2^{-1}I_6)$，$\lambda_1+\lambda_2=1$。
在纯力、$r_i\to0$ 的极限下 arm $i$ 承担 $\lambda_i$ 份额的负载：$h_i = \lambda_i w_o$（式 5.3）。例如一只手靠近奇异、力矩余量小，就给它小的 $\lambda$
（配置项 `controller.coop.load_share_left`）。

需要注意的几点：

1. **单位混合**：$h$ 里同时有 N 和 N·m，$\|h\|_2$ 与长度单位的选择有关（用 mm 还是 m 结果不同）。
   更稳妥的做法是用特征长度 $L$ 归一化：$W_i=\mathrm{blkdiag}(I_3,\,L^{-2}I_3)$ 这类权重。
2. **“内力”的定义依赖于 $G^+$**：$(I-G_W^+G)$ 在 $W\neq I$ 时是斜投影。同一个测量 $h$，用不同 $W$ 算出的
   “内力”不同。文献中有专门讨论“非挤压（non-squeezing）”分配与物理一致的内力定义的工作
   （见 §9 Walker 等、Erhart & Hirche）。做对比实验时**基线和协同控制必须用同一个内力定义**。
3. 对称的 (4.7) 给出的 $h_r$ 相当于“两臂平均分担外力后剩下的部分”：一只手扛全部重量、另一只手
   空着，$h_r=-\tfrac12w_o\neq0$。这在“负载不均”意义上也算内力。选哪个定义取决于你想控制什么。

---

## 6. 控制思路概览（只讲结构，不给实现）

### 6.1 力矩级分层结构

$$
\tau_i = J_i^\top h_{i,\text{cmd}} + h_i(q_i,\dot q_i) + N_i\,\tau_{0,i}, \tag{6.1}
$$

- $J_i^\top h_{i,\text{cmd}}$：让末端对物体施加期望 wrench（静力学对偶）。严格的加速度级
  版本应使用操作空间惯量 $\Lambda_i=(J_iM_i^{-1}J_i^\top)^{-1}$，并补偿 $\dot J_i\dot q_i$（`RobotModel::jacobianDotTimesQdot`）。
- $h_i=C\dot q+g$：偏置补偿（`RobotModel::bias`）。注意 plant 里还有关节阻尼 $D\dot q$（damping=1）。
- $N_i=I-J_i^\top\bar J_i^\top$，$\bar J_i=M_i^{-1}J_i^\top(J_iM_i^{-1}J_i^\top)^{-1}$：动力学一致零空间投影；
  $\tau_{0,i}$ 用于 7 自由度冗余（姿态保持 / 远离限位），否则零空间会漂移（§8）。

$h_\text{cmd}$ 由外力部分和内力部分组成：

$$
h_\text{cmd} = G_W^{+}\,w_{o,\text{cmd}} + V\,h_{r,\text{cmd}}. \tag{6.2}
$$

**物体级阻抗 / 运动跟踪**产生 $w_{o,\text{cmd}}$（$e_o$ 为物体位姿误差，§8 注意姿态误差的定义）：

$$
w_{o,\text{cmd}} = \hat M_o\dot\nu_{o,\text{des}} + \hat c_o - \hat w_g + K_o e_o + D_o(\nu_{o,\text{des}}-\nu_o). \tag{6.3}
$$

**内力调节**（两种思路，可以组合）：

$$
\text{力闭环：}\quad h_{r,\text{cmd}} = h_{r,\text{des}} + K_f\left(h_{r,\text{des}}-h_{r,\text{meas}}\right), \tag{6.4}
$$
$$
\text{相对空间柔顺：}\quad h_{r,\text{cmd}} = K_r e_r + D_r\,(0 - v_r). \tag{6.5}
$$

### 6.2 与速度级 QP 的区别

速度级（运动学）QP 以 $\dot q$ 为变量，输出给底层位置/速度伺服：它能很好地处理关节限位和
相对运动约束，但**底层伺服刚度决定了闭链内力**，控制器本身不管力。力矩级方法直接决定 $\tau$，
末端对物体的 wrench 是显式的设计量，内力可控——代价是依赖动力学模型精度。

### 6.3 力矩级 QP 的形式

决策变量（示例）：$x = [\ddot q\in\mathbb R^{14};\ \tau\in\mathbb R^{14};\ h\in\mathbb R^{12}]$。

$$
\begin{aligned}
\min_{x}\quad & \|J_a\ddot q + \dot J_a\dot q - \dot v_{a,\text{cmd}}\|^2_{Q_a}
              + \|h_r(h) - h_{r,\text{cmd}}\|^2_{Q_r}
              + \|\tau\|^2_{R} + \|\ddot q - \ddot q_{0}\|^2_{Q_0} \\
\text{s.t.}\quad & M\ddot q + b = \tau - J^\top h &&\text{(两臂动力学，}J=\mathrm{blkdiag}(J_1,J_2)\text{)}\\
& J_r\ddot q + \dot J_r\dot q = -K_{pr}e_r - K_{dr}v_r &&\text{(闭链约束的加速度级形式，带稳定项)}\\
& G h = \hat M_o\dot\nu_{o} + \hat c_o - \hat w_g &&\text{(物体动力学，或放进代价)}\\
& -\tau_\text{max}\le\tau\le\tau_\text{max} &&\text{(力矩限制)}\\
& \underline{\ddot q}(q,\dot q)\le\ddot q\le\overline{\ddot q}(q,\dot q) &&\text{(由位置/速度限位推出的加速度界)}\\
& n_{ab}^\top\big(J_{ab}\ddot q + \dot J_{ab}\dot q\big) \ge -\gamma_1 \dot d_{ab} - \gamma_2 (d_{ab}-d_\text{safe}) &&\text{(两臂防碰撞：最近点对距离 }d_{ab}\text{ 的二阶屏障，}J_{ab}\text{ 为两最近点的相对雅可比)}
\end{aligned} \tag{6.6}
$$

实现上通常把 $\ddot q$ 通过动力学消元只留 $(\tau, h)$ 或 $(\ddot q, h)$；等式可以做成硬约束或
加大权重的软约束（数值上更稳）；防碰撞的最近点对可以用 MuJoCo 的 `mj_geomDistance` 在
控制器模型上算。OSQP 的准备情况见 README“可选依赖”。

---

## 7. 实验设计建议

| | 基线 | 协同控制 |
|---|---|---|
| 控制器 | 两臂**独立**笛卡尔阻抗（或位置）控制：各自跟踪 (3.9)–(3.11) 算出的末端轨迹 | 本文 §6 结构 |
| 共用 | 同一条物体参考轨迹（`object_trajectory`）、同一个内力定义（§5）、同一组扰动 |

**自变量**

1. 标定误差：`calibration_error.right_base_offset_xyz/rpy_deg`，建议扫 0 / 1 / 3 / 5 mm 与 0 / 0.2 / 0.5°；
   `weld.init_mode=current`（无装配误差，只有运动中产生的内力）与 `nominal`（带装配预载）都做。
2. 外部扰动：`disturbances`（力 / 力矩脉冲、阶跃）。
3. 物体轨迹：`hold` / `min_jerk`（含旋转——纯平移时位置偏移类标定误差几乎不产生新的内力，
   旋转和 yaw 误差才会，见 §1）/ `sine`。

**指标**（全部可以从 CSV 算出）

- 内力峰值 $\max_t\|f_\text{int}\|$、$\max_t\|m_\text{int}\|$（`int_*` 列，或用 `*_weld_*` 真值离线算）；
- 内力稳态误差 $\|h_r-h_{r,\text{des}}\|$（扰动 / 运动结束后的均值）；
- 物体位姿误差：`obj_err_*`（RMS 与峰值，位置 mm、姿态 deg）；
- 单周期计算时间：`t_ctrl_us`（均值、p99、最大值，对比 1000 µs 预算）；
- 力矩饱和步数（run_sim 结束时打印）。

**注意**：日志中的 `*_weld_*` 是仿真真值；真实系统只有腕部 F/T（`*_ftw_*`）。报告两者的差别
（夹爪惯性、传感器噪声、1 周期延迟）本身就是有价值的结果。

---

## 8. 常见坑

1. **twist / wrench 参考点不一致**。$J_i$ 的线速度参考点是 `ee_site`，weld wrench 的参考点是抓取点，
   F/T 换算值的参考点是 `ee_site`，物体 twist 的参考点是箱子中心。相加之前先用 (2.2) 换到同一点。
   `init_mode=current` 且有标定误差时，`ee_site` 与抓取点会相差几毫米。
2. **$r_i$ 的方向**：本文 $r_i=p_o-p_i$。换成 $p_i-p_o$ 后 $G_i$ 中是 $+S(\cdot)$。
3. **腕部传感器读数包含夹爪自重，需要在控制器侧扣除**。F/T 传感器在 link7 与 hand 之间，测的是
   link7 对整个 hand 子树（hand 0.73 kg + 两指 2×0.015 kg = 0.76 kg）施加的力，其中有
   $m_\text{hand}g\approx7.5$ N 是夹爪自己的重量，还有对传感器原点约 $(p_\text{com}-p_\text{ft})\times m g$ 的力矩；
   加速时还有 $m_\text{hand}a$ 与 $I\dot\omega$。不扣除的话，这个偏置一部分会被当成物体负载
   （两臂各 7.5 N 向下 → 物体“变重” 15 N），另一部分（取决于两臂姿态与抓取几何）落进 $G$ 的零空间，
   被误当成内力。
   `ArmState::ft_raw` 是原始读数；`ft_ee_world` 用 hand 子树质量与子树质心扣除了**静态**重力并换到 TCP，
   惯性项没扣。真实系统中需要辨识夹爪质量/质心（通常在几个姿态下读空载 F/T 做最小二乘）。
4. **MuJoCo 3.12 的 F/T 传感器会把 weld 约束的力矩算成两倍**：`mj_rnePostConstraint` 把 weld 转动行的
   `efc_force` 直接当世界系力矩，而约束实际施加的是 $J^\top f$（转动雅可比带 0.5 系数）。
   `SimEnv` 默认按 $J^\top f$ 精确修正（`sensors.fix_weld_torque`），`tests/test_sensors_weld.cpp` 中
   `RawMujocoTorqueDoublesWeldContribution` 复现了这个问题——升级 MuJoCo 后若它失败，说明上游已修复。
   同理，**不要直接读 `efc_force` 当 wrench**，用 `SimEnv::weldWrench`。
5. **weld 是软约束**：刚度由 `solref`（时间常数，≥ 2·timestep）和 `solimp` 决定，且与有效质量有关，
   不是一个固定的 N/m。它相当于 (1.1) 里的 $k_o$——内力的绝对值受它影响，对比实验时保持不变。
6. **weld relpose 初始化**：MuJoCo 在 XML 未给 relpose 时按 `qpos0`（全零关节角）计算，完全不对。
   `SimEnv` 在 `mj_forward` 后重设；`nominal` 模式下有 0.5 s 过渡，否则 t=0 会有数百牛的冲击。
7. **Panda 7 自由度零空间漂移**：(6.1) 若不加零空间项，自运动不受控，肘部会慢慢漂向限位；
   两臂闭链后零空间只剩每臂 1 维，但仍需要 $\tau_0$。
8. **力矩饱和**：关节 5–7 上限只有 12 N·m，内力稍大就会饱和；饱和后 $J^\top h$ 不再成立，
   物体 wrench 分配被破坏——这是 QP 显式加力矩约束的主要理由。`SimEnv` 会截断到 `ctrlrange`。
9. **姿态误差定义**：$\log(R_\text{des}R^\top)$（世界系旋转向量，本工程 `rotationError`）与
   四元数矢部误差、欧拉角差在大角度时差别很大；阻抗控制中要与刚度矩阵的坐标系匹配。
10. **时间对齐**：`SimEnv::state()` 中的力来自上一步（1 ms 延迟），运动学来自当前步；
    日志行（`lastStep()`）是完全对齐的。
11. **只用名义模型**：控制器里的末端位姿、雅可比必须来自 `RobotModel`，不要用 `state` 里的
    `ee_pose`（仿真真值，包含标定误差），否则标定误差实验失去意义。
12. **zsh 下的 `--set` 参数**：值里有 `[` `]` 时要加引号，如 `--set 'disturbances=[]'`。
13. **为什么用 weld 而不做夹爪接触仿真**。本项目的研究对象是**闭链内力**——两臂与物体构成闭链后，
    位置 / 标定误差如何转化为内力、控制器如何调节它——而不是抓取稳定性。用 weld 把 hand 与箱子
    刚性连接，等价于“抓取足够牢、不打滑”的理想假设，好处是：
    - 抓取点的 6 个方向都能传递力 / 力矩，$G$ 恰好是 §4 的形式，不受摩擦锥限制（不会因为内力大到
      超出摩擦锥而打滑，把“内力控制”问题和“抓取失稳”问题混在一起）；
    - 接触仿真对参数（摩擦系数、solref/solimp、接触点数量、网格质量）非常敏感，平行夹爪夹方盒时
      的接触力分布数值上也不稳定，得到的内力大部分是接触模型的伪影；
    - weld 的约束力可以精确读出（`SimEnv::weldWrench`），作为内力的“真值”，用来评估只依赖腕部 F/T 的估计。

    因此场景中手指是**固定**的（无关节），开度 = 箱宽/2 + 1 mm，只保证视觉上“夹住”且不穿透；
    手指与箱子之间关闭了接触。若以后要研究抓取本身（摩擦锥约束、滑移检测、夹持力优化），
    应改用接触模型并把摩擦锥写进 QP 的约束，届时 $G$ 的列要换成接触点的“接触旋量基”。
14. **weld 与 hand 固连意味着理想刚性抓取**：真实夹爪的指面有柔度（橡胶垫、手指机构间隙），
    相当于在 (1.1) 中串联了一个 $k_\text{grip}$，会显著降低内力。仿真中用 `weld.solref` 近似这部分柔度。

---

## 9. 参考文献

以下是我确信存在的经典工作（卷期信息引用前请以出版社页面为准）：

1. M. Uchiyama, P. Dauchez, “A symmetric hybrid position/force control scheme for the coordination of two robots,” *Proc. IEEE ICRA*, 1988. ——对称协同任务空间（绝对/相对）的起点。
2. M. Uchiyama, P. Dauchez, “Symmetric kinematic formulation and non-master/slave coordinated control of two-arm robots,” *Advanced Robotics*, vol. 7, no. 4, 1993.
3. P. Chiacchio, S. Chiaverini, B. Siciliano, “Direct and inverse kinematics for coordinated motion tasks of a two-manipulator system,” *ASME J. Dynamic Systems, Measurement, and Control*, vol. 118, 1996.
4. F. Caccavale, P. Chiacchio, S. Chiaverini, “Task-space regulation of cooperative manipulators,” *Automatica*, vol. 36, 2000.
5. F. Caccavale, P. Chiacchio, A. Marino, L. Villani, “Six-DOF impedance control of dual-arm cooperative manipulators,” *IEEE/ASME Trans. Mechatronics*, vol. 13, no. 5, 2008. ——物体级阻抗 + 内部（相对）阻抗。
6. F. Caccavale, M. Uchiyama, “Cooperative manipulation,” in *Springer Handbook of Robotics*, 2nd ed., B. Siciliano, O. Khatib (eds.), Springer, 2016. ——综述，推荐先读。
7. M. W. Walker, R. A. Freeman, D. J. Marcus, “Analysis of motion and internal loading of objects grasped by multiple cooperating manipulators,” *Int. J. Robotics Research*, vol. 10, no. 4, 1991. ——内力定义与非挤压分配。
8. R. G. Bonitz, T. C. Hsia, “Internal force-based impedance control for cooperating manipulators,” *IEEE Trans. Robotics and Automation*, vol. 12, no. 1, 1996.
9. S. A. Schneider, R. H. Cannon, “Object impedance control for cooperative manipulation: theory and experimental results,” *IEEE Trans. Robotics and Automation*, vol. 8, no. 3, 1992.
10. S. Erhart, S. Hirche, “Internal force analysis and load distribution for cooperative multi-robot manipulation,” *IEEE Trans. Robotics*, vol. 31, no. 5, 2015. ——物理一致的内力定义与负载分配。
11. C. Smith, Y. Karayiannidis, L. Nalpantidis, X. Gratal, P. Qi, D. V. Dimarogonas, D. Kragic, “Dual arm manipulation—A survey,” *Robotics and Autonomous Systems*, vol. 60, no. 10, 2012.
12. O. Khatib, “A unified approach for motion and force control of robot manipulators: The operational space formulation,” *IEEE J. Robotics and Automation*, vol. 3, no. 1, 1987. ——动力学一致零空间。
13. A. Escande, N. Mansard, P.-B. Wieber, “Hierarchical quadratic programming: Fast online humanoid-robot motion generation,” *Int. J. Robotics Research*, vol. 33, no. 7, 2014.
14. B. Stellato, G. Banjac, P. Goulart, A. Bemporad, S. Boyd, “OSQP: an operator splitting solver for quadratic programs,” *Mathematical Programming Computation*, vol. 12, 2020.
15. B. Siciliano, L. Sciavicco, L. Villani, G. Oriolo, *Robotics: Modelling, Planning and Control*, Springer, 2009. ——雅可比、阻抗控制的基础记号。
