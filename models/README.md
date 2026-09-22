# models/

## 来源

| 路径 | 说明 |
|---|---|
| `franka_emika_panda/` | 从 [mujoco_menagerie](https://github.com/google-deepmind/mujoco_menagerie) 的 `franka_emika_panda/` **原样复制**（含 `assets/`、`LICENSE`、`README.md`、`CHANGELOG.md`），**不做任何修改**。 |
| `panda_hand_torque.xml` | 由 `franka_emika_panda/panda.xml`（带 Franka Hand）派生的力矩控制版本（改动见下）。 |
| `dual_panda_scene.xml` | 本工程的双臂场景。 |

- 复制来源：`/home/tt/colcon_ws/src/mujoco_menagerie/franka_emika_panda`
  （menagerie git commit `1b86ece576591213e2b666ebf59508454200ca97`，2025-11-06）
- 复制日期：2026-09-18
- 复制方式：`cp -a`，复制后用 `diff -r` 确认与原目录完全一致；原目录未做任何修改。
- 许可证：Apache License 2.0，见 [`franka_emika_panda/LICENSE`](franka_emika_panda/LICENSE)。
  模型由 Franka Emika 公开的 URDF（franka_ros）转换而来，详见 `franka_emika_panda/README.md`。
- menagerie 要求 MuJoCo ≥ 2.3.3；本工程的场景使用 `<model>`/`<attach>`，需要 MuJoCo ≥ 3.2（本机 3.12.0）。

## `panda_hand_torque.xml` 相对 `panda.xml` 的改动

1. `meshdir` 改为 `franka_emika_panda/assets`（文件位于 `models/`）。
2. 删除 `<option>`（由场景统一设置）与 worldbody 中的 trackcom 灯。
3. 7 个位置伺服执行器（`<general biastype="affine">`）替换为**力矩电机** `<motor>`，
   `ctrlrange` = Panda 关节力矩上限 **[87, 87, 87, 87, 12, 12, 12] N·m**；删除位置伺服的 default。
4. **夹爪**：删除两根手指的 slide joint（`finger_joint1/2`）及与之相关的 tendon `split`、
   手指联动 equality、`actuator8`、finger 关节 default。两根手指成为 `hand` 的固定子 body，
   固定在开度 **0.036 m = 箱子被夹宽度 0.07 m / 2 + 1 mm**（直接写在 `left_finger`/`right_finger` 的 `pos` 里）。
   hand（0.73 kg）与两指（各 0.015 kg）的质量、惯量保持原值。
5. `hand` 上加两个 site：`ft_site`（hand 原点 = link7 法兰面，腕部 F/T 安装点）、
   `ee_site`（**TCP**：两指之间、法兰前方 0.1034 m，与真实 Panda 的 TCP 约定一致；
   z = 接近方向，y = 手指开合方向）。加入挂在 `ft_site` 上的 `<force>`/`<torque>` 传感器
   （读数包含 hand + 两指共 0.76 kg 的重力与惯性力，需在控制器侧补偿）。
6. 删除 keyframe（由场景定义）。

关节名、link 名、惯性参数、关节阻尼（damping = 1）、转子惯量（armature = 0.1）、关节限位、
碰撞几何均保持原样。**如果改了箱子的被夹宽度，要同步修改两根手指的 `pos`**
（`tests/test_scene_geometry.cpp` 会检查指面间隙是否仍为 1 mm）。

## `dual_panda_scene.xml`

| 元素 | 说明 |
|---|---|
| 工作台 `table` | 台面 2.0 × 0.9 m，顶面高 0.80 m（实际 0.7999，避开 link0 网格底面 0.03 mm 的重叠），4 条腿；**所有 geom 的 contype = conaffinity = 0**，只作尺度参照 |
| 基座 `left_base` / `right_base` | 装在台面上：(−0.745, 0, 0.80) 朝 +x、(+0.745, 0, 0.80) 朝 −x；两次 `<attach model="panda" body="link0" prefix="left_\|right_">`。C++ 端通过 mjSpec 在编译前按 config 修改其位姿（右臂可叠加标定误差） |
| 箱子 `box` | 0.40 × 0.07 × 0.12 m、2 kg，free joint，中心 (0, 0, 0.885)，底面在台面上方 2.5 cm |
| 抓取点 `grasp_left/right` | 箱子系 (∓0.17, 0, 0.04)：距端面 3 cm、顶面下 2 cm；姿态与初始构型下的 TCP 重合（z 竖直向下，y 为手指开合方向） |
| weld `weld_left/right` | `body1 = <arm>_hand`，`body2 = box`，anchor = 抓取点；XML 中的 relpose 是名义值（抓取点在 hand 系的 (0, 0, 0.1034)），运行时由 `SimEnv` 按实际相对位姿重设，t=0 无预载内力 |
| 接触 | hand、两指、link6、link7 与箱子之间 `<exclude>`：夹持力全部由 weld 传递 |
| 相机 | `cam_iso`（等轴测）、`cam_front`（从 −y 正视两臂连线，左臂在画面左侧），都对准 (0, 0, 0.97) |
| 外观 | 素色地板 + 0.25 m 淡网格、浅色天空盒；离屏分辨率 1600 × 1200 |
| 其它 | `timestep = 0.001`，`integrator = implicitfast`，重力开启；关键帧 `grasp` 为初始构型 |

### 抓取方式与被夹尺寸

Franka Hand 的最大开度为 80 mm（每指 0–0.04 m），所以被夹的那个尺寸必须 < 78 mm。
箱子因此改为 0.40 × **0.07** × 0.12 m，夹爪**竖直向下**、在距两端面 3 cm 处夹住箱子的 ±y 侧面
（像两个人各抓木板一端）。检查结果（初始构型）：

- 手指碰撞盒与箱子侧面的距离 = 1.000 mm（`mj_geomDistance`）；手指视觉网格顶点离箱子最近 0.92 mm；
- hand 外壳在箱子顶面上方 17.8 mm，link7 离箱子 84 mm；两臂所有几何体都没有顶点落在箱子内部；
- 初始时刻 `ncon = 0`。

### 初始构型的选取

TCP 竖直向下、手指沿世界 y 开合时，取平面肘上构型族 `q = [0, q2, 0, q4, 0, q2−q4, π/4]`，
对 TCP 在基座系 (r, 0, 0.125) 扫描水平前伸 r：

| r [m] | 最小归一化关节裕度 | cond(J) | σ_min(J_v) | 箱子 ±0.15 m / ±20° 扰动下：最小裕度 / 最大 cond(J) |
|---|---|---|---|---|
| 0.500 | 0.223 | 8.9 | 0.245 | 0.062 / 14.7 |
| 0.550 | 0.282 | 9.3 | 0.256 | 0.114 / 13.1 |
| **0.575** | **0.314** | **9.6** | **0.259** | **0.141 / 14.8** |
| 0.600 | 0.340 | 10.0 | 0.261 | 0.168 / 17.6 |
| 0.625 | 0.352 | 10.5 | 0.260 | 0.184 / 23.2 |

取 r = 0.575 m（裕度与条件数的折中），基座间距 = 2 × (0.17 + 0.575) = 1.49 m，
`q_init = [0, 0.378146, 0, −2.130506, 0, 2.508653, 0.785398]`：肘部竖直向上、q1 = q3 = q5 = 0，
各关节到限位的裕度（度 / 占行程）：j1 166.0/50%、j2 79.3/39%、j3 166.0/50%、**j4 53.9/31%**、
j5 166.0/50%、j6 71.3/33%、j7 121.0/36%；cond(J) = 9.65（J 为 SI 单位的 6×7 几何雅可比），
cond(J_v) = 3.29，σ_min(J_v) = 0.259 m/rad。`run_sim` 启动时会打印这些值。

可以直接用 MuJoCo 自带的查看器打开场景检查：`simulate models/dual_panda_scene.xml`
（此时执行器 ctrl = 0，两臂会在重力下落下，这是正常的）。
