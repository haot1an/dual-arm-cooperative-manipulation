#!/usr/bin/env python3
"""场景 1（窄槽插入）的几何设计、可行性验算与判别性验证。

    python3 tools/design_slot.py --search    # 网格搜索几何参数（使用 config 中的布局），打印每组参数的指标
    python3 tools/design_slot.py --write     # 用 CHOSEN 参数生成 models/scene_slot.xml + config/scenes/slot.yaml
    python3 tools/design_slot.py --verify    # 对当前布局与 scene_slot.xml 做逐航点验算 + 判别性验证
                                             # （结果写 docs/slot_design_tables.md）

设计思路（详见 docs/slot_design.md）：
  - 夹爪沿长板长轴（±x）水平接近，在两端面中心、板厚方向（2 cm）夹住板 → 抓取点就在滚转轴上，
    滚转时 TCP 不平移，滚转几乎完全由 joint7（与接近方向同轴）完成。
  - 为了让下侧手指能伸到板下、并让 hand 外壳（沿手指开合方向 ±0.104 m）不碰台面，板架在支撑块上，
    两端各悬空 6 cm，板中心线离台面 ≥ 0.125 m。
  - 目标是前上方插槽座上的一道窄槽（宽 s_w），板必须滚转到接近竖直（厚度方向朝 y）才能插入深度 D。
"""
from __future__ import annotations

import argparse
import itertools
import math
import sys
from dataclasses import dataclass, asdict, replace
from pathlib import Path

import numpy as np
import yaml
from scipy.spatial.transform import Rotation as Rot

sys.path.insert(0, str(Path(__file__).resolve().parent))
from dual_arm_kin import (ROOT, ARMS, SEEDS, Scene, best_path, compose, deep_merge, fmt_q, load_config,  # noqa
                          mirror_q, pose, rpy_matrix)

Z_TABLE = 0.75


@dataclass(frozen=True)
class SlotParams:
    L: float = 0.60           # 板长（x）
    W: float = 0.20           # 板宽（平放时沿 y，竖直时沿 z）
    T: float = 0.02           # 板厚
    mass: float = 1.2
    grasp_inset: float = 0.01  # TCP 在端面内侧的距离（指垫中心）
    y_start: float = -0.45    # 板初始中心 y
    riser_h: float = 0.115    # 支撑块高度（板底 = 台面 + riser_h + 1 mm）
    lift_extra: float = 0.03  # 抬起高度 = W/2 + lift_extra（滚转时下边缘离开支撑块）
    roll_deg: float = 90.0    # 滚转目标角（绕板长轴 x）
    roll_sign: int = 1        # 滚转方向（+1: 绕 +x 正转）
    slot_dy: float = 0.15     # 插槽中心相对初始位置向前（−y）的距离
    slot_h: float = 0.20      # 槽口（槽壁顶面）离台面高度
    slot_w: float = 0.04      # 槽宽（y 方向）
    slot_D: float = 0.05      # 插入深度（槽底 = 槽口 − D）
    wall_t: float = 0.05      # 槽壁厚（y 方向）
    pre_insert: float = 0.03  # 插入前板底离槽口的高度
    post_x: float = 0.62      # 障碍柱 x 位置（±）
    post_dy: float = 0.08     # 障碍柱相对初始位置向前的距离
    post_r: float = 0.025
    post_top: float = 0.30    # 障碍柱顶离台面高度
    finger_sign_left: int = 1   # 左手 TCP y 轴 = ±板 z 轴（平放时）
    finger_sign_right: int = 1

    # ---- 派生量 ----
    @property
    def z_center0(self):
        return Z_TABLE + self.riser_h + 0.001 + self.T / 2

    @property
    def y_slot(self):
        return self.y_start - self.slot_dy

    @property
    def z_insert(self):  # 插入到底时板中心高度（板底在槽底上方 1 mm）
        return Z_TABLE + self.slot_h - self.slot_D + 0.001 + self.W / 2

    def waypoints(self):
        rs = self.roll_sign * self.roll_deg
        z_lift = self.z_center0 + self.W / 2 + self.lift_extra
        z_pre = self.z_insert + self.slot_D + self.pre_insert
        z_carry = max(z_lift, z_pre)
        return [
            dict(name="grasp", pos=[0.0, self.y_start, round(self.z_center0, 4)], rpy_deg=[0, 0, 0]),
            dict(name="lifted", pos=[0.0, self.y_start, round(z_carry, 4)], rpy_deg=[0, 0, 0]),
            dict(name="rolled", pos=[0.0, self.y_start, round(z_carry, 4)], rpy_deg=[rs, 0, 0]),
            dict(name="pre_insert", pos=[0.0, round(self.y_slot, 4), round(z_pre, 4)], rpy_deg=[rs, 0, 0]),
            dict(name="inserted", pos=[0.0, round(self.y_slot, 4), round(self.z_insert, 4)], rpy_deg=[rs, 0, 0]),
        ]

    def grasp_quat(self, arm):
        """TCP 在板坐标系中的姿态：z 指向板中心（沿 ∓x），y = ±板 z 轴"""
        s = self.finger_sign_left if arm == "left" else self.finger_sign_right
        z = np.array([-1.0, 0, 0]) if arm == "left" else np.array([1.0, 0, 0])
        y = np.array([0, 0, float(s)])
        R = np.column_stack([np.cross(y, z), y, z])
        x, yq, zq, w = Rot.from_matrix(R).as_quat()
        return [w, x, yq, zq]


# 搜索结果（--search，布局 = 标定前的名义布局 0.65 m / 10°）：slot_dy = 0.10/0.15 时平放的板或手臂与槽座干涉；
# slot_dy = 0.20、roll_sign = −1 时最优，y_start = −0.40 与 −0.45 得分相差 0.001，取 −0.45 与其余场景的物体初始位置一致；
# slot_h = 0.25 比 0.20 的 q7 裕度更大（0.227 vs 0.210），障碍柱间隙 4.1 cm vs 2.1 cm。
CHOSEN = SlotParams(y_start=-0.45, slot_dy=0.20, slot_h=0.25, roll_sign=-1)


def fmt(v):
    return " ".join(f"{x:.6g}" for x in v)


def camera_xyaxes(pos, target):
    pos, target = np.asarray(pos, float), np.asarray(target, float)
    f = target - pos
    f /= np.linalg.norm(f)
    x = np.cross(f, [0, 0, 1.0])
    x /= np.linalg.norm(x)
    y = np.cross(-f, x)
    return " ".join(f"{v:.4f}" for v in np.concatenate([x, y])).replace("-0.0000", "0.0000")


def make_xml(p: SlotParams) -> str:
    L, W, T = p.L, p.W, p.T
    gx = L / 2 - p.grasp_inset
    riser_hx = L / 2 - 0.06
    ys = p.y_slot
    wall_y = p.slot_w / 2 + p.wall_t / 2
    slot_hx = L / 2 + 0.02
    base_h = p.slot_h - p.slot_D
    post_y = p.y_start - p.post_dy
    tgt = (0.0, (p.y_start + p.y_slot) / 2, Z_TABLE + 0.18)
    iso, front = (1.45, -1.85, 1.62), (0.0, -2.45, 1.30)
    return f"""<!--
  scene_slot.xml —— 场景 1：长板转向 + 窄槽插入 + 避碰（由 tools/design_slot.py --write 生成，勿手改）
  =====================================================================================================
  长板 {L:.2f} × {W:.2f} × {T:.2f} m、{p.mass} kg，平放在支撑块上（两端各悬空 6 cm，便于手指从上下夹住板厚）。
  两臂沿板长轴水平接近，在两端面中心（滚转轴上）夹住板厚；抬起 → 绕长轴滚转 {p.roll_deg:.0f}° 到竖直 →
  平移到前上方插槽座 → 向下插入宽 {p.slot_w*1000:.0f} mm、深 {p.slot_D*1000:.0f} mm 的窄槽。
  平放的板宽 {W*1000:.0f} mm ≫ 槽宽，不滚转在几何上无法插入（判别性验证见 docs/slot_design.md）。
  障碍柱 post_L / post_R 位于两臂腕部 / 前臂的搬运路径旁（最小间隙见 docs/slot_design.md）。
-->
<mujoco model="scene_slot">
  <include file="workcell.xml"/>

  <worldbody>
    <camera name="cam_iso" pos="{fmt(iso)}" xyaxes="{camera_xyaxes(iso, tgt)}"/>
    <camera name="cam_front" pos="{fmt(front)}" xyaxes="{camera_xyaxes(front, tgt)}"/>

    <!-- 支撑块：板的初始位置 -->
    <body name="riser" pos="0 {p.y_start:.4f} {Z_TABLE + p.riser_h / 2:.4f}">
      <geom name="riser" class="prop" size="{riser_hx:.4f} {W / 2 - 0.02:.4f} {p.riser_h / 2:.4f}" material="prop_grey"/>
    </body>

    <!-- 插槽座：底座 + 两道槽壁；槽宽 {p.slot_w*1000:.0f} mm，槽口高 {Z_TABLE + p.slot_h:.3f} m，槽底高 {Z_TABLE + base_h:.3f} m -->
    <body name="slot_frame" pos="0 {ys:.4f} {Z_TABLE:.4f}">
      <geom name="slot_base" class="prop" size="{slot_hx:.4f} {p.slot_w / 2 + p.wall_t:.4f} {base_h / 2:.4f}"
        pos="0 0 {base_h / 2:.4f}" material="prop_dark"/>
      <geom name="slot_wall_front" class="prop" size="{slot_hx:.4f} {p.wall_t / 2:.4f} {p.slot_D / 2:.4f}"
        pos="0 {-wall_y:.4f} {base_h + p.slot_D / 2:.4f}" material="prop_blue"/>
      <geom name="slot_wall_back" class="prop" size="{slot_hx:.4f} {p.wall_t / 2:.4f} {p.slot_D / 2:.4f}"
        pos="0 {wall_y:.4f} {base_h + p.slot_D / 2:.4f}" material="prop_blue"/>
    </body>

    <!-- 障碍柱 -->
    <body name="post_L" pos="{p.post_x:.4f} {post_y:.4f} {Z_TABLE + p.post_top / 2:.4f}">
      <geom name="post_L" class="prop" type="cylinder" size="{p.post_r:.4f} {p.post_top / 2:.4f}" material="obstacle"/>
    </body>
    <body name="post_R" pos="{-p.post_x:.4f} {post_y:.4f} {Z_TABLE + p.post_top / 2:.4f}">
      <geom name="post_R" class="prop" type="cylinder" size="{p.post_r:.4f} {p.post_top / 2:.4f}" material="obstacle"/>
    </body>

    <body name="plate" pos="0 {p.y_start:.4f} {p.z_center0:.4f}">
      <freejoint name="plate_joint"/>
      <geom name="plate_geom" type="box" size="{L / 2:.4f} {W / 2:.4f} {T / 2:.4f}" mass="{p.mass}" material="wood"
        friction="0.8 0.02 0.001"/>
      <!-- 抓取点：两端面中心向内 {p.grasp_inset*1000:.0f} mm（在滚转轴上）；z 指向板中心，y 为手指开合方向（板厚方向） -->
      <site name="grasp_L" pos="{gx:.4f} 0 0" quat="{fmt(p.grasp_quat('left'))}" size="0.008" rgba="0 0.8 0 1"/>
      <site name="grasp_R" pos="{-gx:.4f} 0 0" quat="{fmt(p.grasp_quat('right'))}" size="0.008" rgba="0 0.8 0 1"/>
      <site name="plate_center" size="0.008" rgba="0 0 1 1"/>
    </body>
  </worldbody>

  <equality>
    <weld name="weld_L" body1="left_hand" body2="plate" anchor="{gx:.4f} 0 0" solref="0.004 1" solimp="0.95 0.99 0.001 0.5 2"/>
    <weld name="weld_R" body1="right_hand" body2="plate" anchor="{-gx:.4f} 0 0" solref="0.004 1" solimp="0.95 0.99 0.001 0.5 2"/>
  </equality>

  <contact>
    <exclude body1="left_hand" body2="plate"/>
    <exclude body1="left_left_finger" body2="plate"/>
    <exclude body1="left_right_finger" body2="plate"/>
    <exclude body1="left_link7" body2="plate"/>
    <exclude body1="right_hand" body2="plate"/>
    <exclude body1="right_left_finger" body2="plate"/>
    <exclude body1="right_right_finger" body2="plate"/>
    <exclude body1="right_link7" body2="plate"/>
  </contact>
</mujoco>
"""


def make_scene_yaml(p: SlotParams, q_init=None) -> dict:
    """场景描述（dict，供评估用；写文件用 scene_yaml_text，两者内容一致）"""
    return yaml.safe_load(scene_yaml_text(p, q_init))


def scene_yaml_text(p: SlotParams, q_init=None) -> str:
    q_init = q_init or {"left": [0.0] * 7, "right": [0.0] * 7}
    fo = round(p.T / 2 + 0.001, 4)
    wp = "\n".join(f"  - {{name: {w['name']+',':12s} pos: [{w['pos'][0]:.1f}, {w['pos'][1]:.4f}, {w['pos'][2]:.4f}], "
                   f"rpy_deg: [{w['rpy_deg'][0]:g}, 0, 0]}}" for w in p.waypoints())
    design = "\n".join(f"  {k}: {v}" for k, v in asdict(p).items())
    ql = ", ".join(f"{v:.6f}" for v in q_init["left"])
    qr = ", ".join(f"{v:.6f}" for v in q_init["right"])
    return f"""# 场景 1：长板转向 + 窄槽插入 + 避碰 —— SceneSpec（由 tools/design_slot.py --write 生成）
# 几何设计与可行性 / 判别性验证见 docs/slot_design.md。
name: slot
description: 长板滚转 {p.roll_deg:.0f}° 后插入窄槽（对称协作，刚性抓取，相对自由度 0）
model: models/scene_slot.xml

object:
  body: plate
  geom: plate_geom

# 两臂沿板长轴水平接近，在两端面中心夹住板厚（{p.T*1000:.0f} mm）：finger_opening = T/2 + 1 mm
grasps:
  left:  {{site: grasp_L, weld: weld_L, finger_opening: {fo}}}
  right: {{site: grasp_R, weld: weld_R, finger_opening: {fo}}}

# 两个 rigid 约束 → 两臂之间相对自由度 0 → 内力空间 6 维（其中绕长轴的“扭转”内力矩对姿态误差最敏感）
constraints:
  - {{name: weld_L, type: rigid, body1: left_hand, body2: plate, relative_dof: 0}}
  - {{name: weld_R, type: rigid, body1: right_hand, body2: plate, relative_dof: 0}}

obstacles:
  pairs:
    - [left_arm, right_arm]
    - [left_arm, table_top]
    - [right_arm, table_top]
    - [left_arm, post_L]
    - [right_arm, post_R]
    - [left_arm, slot_frame]
    - [right_arm, slot_frame]
    - [left_arm, riser]
    - [right_arm, riser]
    - [object, slot_frame]
    - [object, riser]
    - [object, post_L]
    - [object, post_R]

# 物体（板中心）航点。rolled 之后板绕世界 x 轴转 {p.roll_sign * p.roll_deg:g}°（竖直，厚度方向沿 y）
waypoints:
{wp}

cameras: [cam_iso, cam_front]

# 窄槽参数（供日志 / 绘图使用：滚转同步误差、插入深度）
slot:
  width: {p.slot_w}          # [m] 槽宽（y）
  depth: {p.slot_D}          # [m] 插入深度
  mouth_z: {Z_TABLE + p.slot_h:.4f}      # [m] 槽口高度（世界系）
  center_y: {p.y_slot:.4f}   # [m] 槽中心 y
  roll_axis: [1, 0, 0]       # 滚转轴（世界系，= 板长轴）
  roll_deg: {p.roll_sign * p.roll_deg:g}

# 初始关节角（抓取位姿，由 tools/calibrate_base.py --write 按最终布局求解后写入）
q_init:
  left:  [{ql}]
  right: [{qr}]

# 生成本场景所用的几何参数（tools/design_slot.py 的 SlotParams）
design:
{design}

overrides: {{}}
"""


# ---------------------------------------------------------------------------
# 评估
# ---------------------------------------------------------------------------
TMP_XML = ROOT / "models" / ".tmp_scene_slot_design.xml"


def load_scene(p: SlotParams, layout=None) -> tuple[Scene, dict]:
    """生成临时 XML（必须放在 models/ 下才能解析 include），加载后立即删除"""
    TMP_XML.write_text(make_xml(p))
    base = load_config("lift")  # 只借用 root 配置
    sc = make_scene_yaml(p)
    cfg = {"root": base["root"], "scene": sc, "name": "slot"}
    try:
        S = Scene.load(cfg, layout, model_path=TMP_XML)
    finally:
        TMP_XML.unlink(missing_ok=True)
    return S, cfg


def interp_path(wps, n_per_seg=8):
    """航点间插值（位置线性 + 姿态 slerp），返回 [(段名, 位姿), ...]"""
    out = []
    for a, b in zip(wps, wps[1:]):
        Ra, Rb = rpy_matrix(a["rpy_deg"]), rpy_matrix(b["rpy_deg"])
        key = Rot.from_matrix(np.stack([Ra, Rb]))
        from scipy.spatial.transform import Slerp
        sl = Slerp([0, 1], key)
        for k in range(n_per_seg):
            s = k / n_per_seg
            p = (1 - s) * np.array(a["pos"]) + s * np.array(b["pos"])
            out.append((f"{a['name']}→{b['name']}" if k else a["name"], (p, sl([s]).as_matrix()[0]), k == 0))
    last = wps[-1]
    out.append((last["name"], (np.array(last["pos"], float), rpy_matrix(last["rpy_deg"])), True))
    return out


FLIP = np.diag([-1.0, -1.0, 1.0])  # 绕 TCP z 轴转 180°：手指开合方向取反（夹爪对称，两种抓法等价）


def evaluate(S: Scene, cfg: dict, n_per_seg=8, choose_flip=True):
    """沿航点路径做 IK 连续求解（每条臂在两种等价手指方向中选最小裕度更大的），返回 (可行, 行, 选择)"""
    sc = cfg["scene"]
    obj = sc["object"]["body"]
    path = interp_path(sc["waypoints"], n_per_seg)
    sols, flips = {}, {}
    # 先解右臂，再用右臂解的镜像作为左臂的种子，最后用左臂解的镜像再给右臂一次机会
    for arm in ("right", "left", "right"):
        other = "left" if arm == "right" else "right"
        extra = [mirror_q(q) for q in (sols[other][:1] if other in sols else [])]
        Tg = S.site_in_body(sc["grasps"][arm]["site"])
        best = (None, -math.inf, None) if arm not in sols else (sols[arm], flips[arm][1], flips[arm][0])
        for flip in ((False, True) if choose_flip else (False,)):
            Tg2 = (Tg[0], Tg[1] @ FLIP) if flip else Tg
            targets = [compose(To, Tg2) for _, To, _ in path]
            qs, qual = best_path(S, arm, targets, extra)
            if qs is not None and qual > best[1]:
                best = (qs, qual, flip)
        if best[0] is None:
            return False, [], {}
        sols[arm], flips[arm] = best[0], (best[2], best[1])
    flips = {a: f[0] for a, f in flips.items()}
    geoms = {
        "left_arm": S.arm_geoms["left"], "right_arm": S.arm_geoms["right"],
        "table": [S.gid("table_top")], "frame": S.geoms_of_bodies(["slot_frame"]),
        "post_L": [S.gid("post_L")], "post_R": [S.gid("post_R")], "riser": [S.gid("riser")],
        "plate": [S.gid("plate_geom")],
    }
    rows = []
    for k, (name, To, is_wp) in enumerate(path):
        S.set_free_body(obj, To)
        row = dict(name=name, wp=is_wp, T_obj=To)
        for arm in ARMS:
            row[arm] = S.arm_metrics(arm, sols[arm][k])
        S.set_arm("left", row["left"]["q"]); S.set_arm("right", row["right"]["q"]); S.forward()
        md = S.min_distance
        row["d_arms"] = md(geoms["left_arm"], geoms["right_arm"])[0]
        row["d_table"] = min(md(geoms[a], geoms["table"])[0] for a in ("left_arm", "right_arm"))
        row["d_frame"] = min(md(geoms[a], geoms["frame"])[0] for a in ("left_arm", "right_arm"))
        row["d_post"] = min(md(geoms["left_arm"], geoms["post_L"])[0], md(geoms["right_arm"], geoms["post_R"])[0])
        row["d_riser"] = min(md(geoms[a], geoms["riser"])[0] for a in ("left_arm", "right_arm"))
        row["d_plate_frame"] = md(geoms["plate"], geoms["frame"])[0]
        row["d_plate_post"] = min(md(geoms["plate"], geoms["post_L"])[0], md(geoms["plate"], geoms["post_R"])[0])
        row["d_plate_riser"] = md(geoms["plate"], geoms["riser"])[0]
        rows.append(row)
    return True, rows, flips


def summarize(ok, rows):
    good = rows
    if not ok or not good:
        return dict(feasible=False)
    s = dict(feasible=True)
    s["margin"] = min(min(r[a]["margin_n"].min() for a in ARMS) for r in good)
    s["cond"] = max(max(r[a]["cond"] for a in ARMS) for r in good)
    s["smin_v"] = min(min(r[a]["smin_v"] for a in ARMS) for r in good)
    for k in ("d_arms", "d_table", "d_frame", "d_post", "d_riser"):
        s[k] = min(r[k] for r in good)
    s["d_plate_frame"] = min(r["d_plate_frame"] for r in good)
    s["d_plate_post"] = min(r["d_plate_post"] for r in good)
    # 板与支撑块：抓取位姿下板本来就压在支撑块上（1 mm），从第 2 个采样点起检查
    s["d_plate_riser"] = min(r["d_plate_riser"] for r in good[1:])
    return s


def score(s):
    if not s["feasible"]:
        return -1e9
    pen = 0.0
    for k, lim in (("d_arms", 0.05), ("d_table", 0.02), ("d_frame", 0.015), ("d_riser", 0.01),
                   ("d_plate_frame", 0.004), ("d_post", 0.02), ("d_plate_post", 0.02), ("d_plate_riser", 0.005)):
        if s[k] < lim:
            pen += 10.0 * (lim - s[k]) + (1.0 if s[k] < 0 else 0.0)
    return s["margin"] + 0.5 * s["smin_v"] - 0.01 * s["cond"] - pen


def _strip_min_z(center_yz, R, p: SlotParams):
    """板横截面（yz 平面内的 W×T 矩形）落在槽口 y 区间 |y − y_slot| ≤ slot_w/2 内那部分的最低 z；不相交返回 None。
    槽座与板都沿 x 拉伸（槽座长于板），所以插入问题可以在 yz 截面上判断。"""
    A = R[1:, 1:]
    loc = [(-p.W / 2, -p.T / 2), (p.W / 2, -p.T / 2), (p.W / 2, p.T / 2), (-p.W / 2, p.T / 2)]
    P = [np.asarray(center_yz) + A @ np.array(c) for c in loc]
    y0, y1 = p.y_slot - p.slot_w / 2, p.y_slot + p.slot_w / 2
    zs = [q[1] for q in P if y0 <= q[0] <= y1]
    for a, b in zip(P, P[1:] + P[:1]):
        for yb in (y0, y1):
            if (a[0] - yb) * (b[0] - yb) < 0:
                t = (yb - a[0]) / (b[0] - a[0])
                zs.append(a[1] + t * (b[1] - a[1]))
    return min(zs) if zs else None


def discriminative(S: Scene, p: SlotParams, angles=(0, 15, 30, 45, 60, 70, 75, 80, 85, 88, 90)):
    """判别性验证。对每个滚转角 a：
      depth = 板竖直下落时，在槽口 y 区间内的部分能到达槽口以下的最大深度——在板中心相对槽中心的 y 偏移
              （−6..+6 cm，1 mm 步长）上取最优，对每个偏移用 MuJoCo 有符号距离二分出最低无穿透高度；
              需要 depth ≥ slot_D − 1 mm 才算插到底；
      dist  = 板中心对准槽中心、板在槽口区间内的最低点放到“插入到底”（槽底上方 1 mm）时，与槽座的
              最小有符号距离（< 0 = 穿透深度，即不滚转/滚转不足时硬插的干涉量）。"""
    plate = S.gid("plate_geom")
    frame = S.geoms_of_bodies(["slot_frame"])
    mouth = Z_TABLE + p.slot_h
    z_bottom = mouth - p.slot_D + 0.001

    def dist(y, z, R):
        S.set_free_body("plate", (np.array([0.0, y, z]), R))
        S.forward()
        return min(S.m.geom_distance(plate, g, 1.0) for g in frame)

    res = []
    for a in angles:
        R = rpy_matrix([p.roll_sign * a, 0, 0])
        # 插入到底位姿（y 偏移 0）：在槽口区间内的最低点 = 槽底上方 1 mm
        m0 = _strip_min_z((p.y_slot, 0.0), R, p)
        d_ins = dist(p.y_slot, z_bottom - m0, R)
        best = 0.0
        for dy in np.arange(-0.06, 0.0605, 0.001):
            y = p.y_slot + dy
            m = _strip_min_z((y, 0.0), R, p)
            if m is None:
                continue
            lo_z, hi_z = z_bottom - m, mouth + 0.01 - m  # 中心高度：最低点在槽底 / 槽口上方 1 cm
            if dist(y, lo_z, R) >= 0:
                best = max(best, mouth - z_bottom)
                continue
            if dist(y, hi_z, R) < 0:
                continue
            for _ in range(30):
                mid = 0.5 * (lo_z + hi_z)
                if dist(y, mid, R) < 0:
                    lo_z = mid
                else:
                    hi_z = mid
            best = max(best, mouth - (hi_z + m))
        res.append(dict(angle=a, dist=d_ins, depth=best))
    return res


def print_table(rows, file=sys.stdout):
    hdr = f"{'sample':24s} {'arm':5s} {'q [rad]':58s} {'min margin':>10s} {'σmin(J)':>8s} {'cond':>6s}"
    print(hdr, file=file)
    for r in rows:
        if not r["wp"]:
            continue
        for arm in ARMS:
            mt = r[arm]
            if mt is None:
                print(f"{r['name']:24s} {arm:5s} IK FAIL", file=file)
                continue
            print(f"{r['name']:24s} {arm:5s} {fmt_q(mt['q']):58s} {mt['margin_n'].min():10.3f} {mt['smin']:8.3f} "
                  f"{mt['cond']:6.2f}", file=file)


def write_scene(p: SlotParams):
    xml = ROOT / "models" / "scene_slot.xml"
    yml = ROOT / "config" / "scenes" / "slot.yaml"
    xml.write_text(make_xml(p))
    q_init = None
    if yml.exists():  # 保留已标定的 q_init（重新 --write 几何时由 calibrate_base.py --write 刷新）
        q_init = yaml.safe_load(yml.read_text()).get("q_init")
    yml.write_text(scene_yaml_text(p, q_init))
    print(f"wrote {xml.relative_to(ROOT)} and {yml.relative_to(ROOT)}")


def verify(out_md: Path):
    cfg = load_config("slot")
    p = SlotParams(**cfg["scene"]["design"])
    S = Scene.load(cfg)
    ok, rows, flips = evaluate(S, cfg, n_per_seg=8)
    lay = cfg["root"]["layout"]
    L = []
    L.append(f"布局：左 base_pos {lay['left']['base_pos']} yaw {lay['left']['base_rpy_deg'][2]}°，"
             f"右 base_pos {lay['right']['base_pos']} yaw {lay['right']['base_rpy_deg'][2]}°  \n"
             f"几何：板 {p.L}×{p.W}×{p.T} m / {p.mass} kg，滚转 {p.roll_sign * p.roll_deg:g}°，"
             f"槽宽 {p.slot_w*1000:.0f} mm、深 {p.slot_D*1000:.0f} mm、槽口高 {Z_TABLE + p.slot_h:.3f} m、"
             f"槽中心 y = {p.y_slot:.3f} m")
    if not ok:
        L.append("\n**不可行：有航点 IK 无解或路径不连续**")
        out_md.write_text("\n".join(L) + "\n")
        print("\n".join(L))
        return False
    L.append(f"\n手指方向（TCP 绕 z 翻转 180°）：{flips}\n")
    L.append("### 逐航点：关节角与裕度\n")
    L.append("| 航点 | 臂 | q [rad] | 最小归一化裕度（关节） | σ_min(J) | cond(J) |")
    L.append("|---|---|---|---|---|---|")
    for r in rows:
        if not r["wp"]:
            continue
        for arm in ARMS:
            mt = r[arm]
            j = int(np.argmin(mt["margin_n"]))
            L.append(f"| {r['name']} | {arm} | `{fmt_q(mt['q'])}` | {mt['margin_n'][j]:.3f}（q{j+1}） | "
                     f"{mt['smin']:.3f} | {mt['cond']:.1f} |")
    L.append("\n### 逐航点：最小距离 [cm]\n")
    L.append("| 航点 | 臂—臂 | 臂—台面 | 臂—槽座 | 臂—立柱 | 臂—支撑块 | 板—槽座 | 板—立柱 |")
    L.append("|---|---|---|---|---|---|---|---|")
    for r in rows:
        if r["wp"]:
            L.append(f"| {r['name']} | {r['d_arms']*100:.1f} | {r['d_table']*100:.1f} | {r['d_frame']*100:.1f} | "
                     f"{r['d_post']*100:.1f} | {r['d_riser']*100:.1f} | {r['d_plate_frame']*100:.1f} | "
                     f"{r['d_plate_post']*100:.1f} |")
    sm = summarize(ok, rows)
    L.append(f"| **全路径最小**（{len(rows)} 个采样） | {sm['d_arms']*100:.1f} | {sm['d_table']*100:.1f} | "
             f"{sm['d_frame']*100:.1f} | {sm['d_post']*100:.1f} | {sm['d_riser']*100:.1f} | "
             f"{sm['d_plate_frame']*100:.1f} | {sm['d_plate_post']*100:.1f} |")
    L.append(f"\n全路径：最小归一化裕度 {sm['margin']:.3f}，最大 cond {sm['cond']:.1f}，"
             f"最小平动 σ_min {sm['smin_v']:.3f}（板—支撑块 {sm['d_plate_riser']*100:.1f} cm，从离开支撑块后算起）")
    # 滚转段关节范围
    i0 = next(i for i, r in enumerate(rows) if r["wp"] and r["name"] == "lifted")
    i1 = next(i for i, r in enumerate(rows) if r["wp"] and r["name"] == "rolled")
    L.append("\n### 滚转段（lifted → rolled）各关节范围 [rad]\n")
    L.append("| 臂 | " + " | ".join(f"q{k}" for k in range(1, 8)) + " |")
    L.append("|---|" + "---|" * 7)
    for arm in ARMS:
        Q = np.array([rows[i][arm]["q"] for i in range(i0, i1 + 1)])
        cells = []
        for k in range(7):
            lo, hi = Q[:, k].min(), Q[:, k].max()
            cells.append(f"{lo:+.3f} ~ {hi:+.3f}（Δ {hi - lo:.3f}）")
        L.append(f"| {arm} | " + " | ".join(cells) + " |")
    lim = S.hi
    L.append(f"\n关节限位 ±[{', '.join(f'{v:.4f}' for v in lim)}]（q4、q6 非对称，见 panda.xml）")
    # 判别性
    L.append("\n### 判别性：不同滚转角下竖直插入\n")
    L.append("| 滚转角 [°] | 板中心对准槽、放到插入深度时板—槽座有符号距离 [mm]（<0 = 干涉） | "
             f"竖直下落可达插入深度 [mm]（y 偏移取最优；需要 {p.slot_D*1000 - 1:.0f}） |")
    L.append("|---|---|---|")
    disc = discriminative(S, p)
    for d in disc:
        L.append(f"| {d['angle']:g} | {d['dist']*1000:+.1f} | {d['depth']*1000:.1f} |")
    feas = [d["angle"] for d in disc if d["depth"] >= p.slot_D - 0.0011]
    # 解析临界角：板以倾角 a 斜插，槽内（深 D' = D − 1 mm）那一段截面的水平宽度 D'·cot a + T / sin a 必须 ≤ 槽宽
    Dp = p.slot_D - 0.001
    crit = next((a for a in np.arange(1, 90.01, 0.1)
                 if Dp / math.tan(math.radians(a)) + p.T / math.sin(math.radians(a)) <= p.slot_w), None)
    L.append(f"\n网格上能插到底（≥ {Dp*1000:.0f} mm）的最小滚转角：{min(feas) if feas else '—'}°；"
             f"解析临界角（槽内截面宽度 D'·cot a + T/sin a ≤ 槽宽）≈ {crit:.1f}°。"
             f"不滚转时板宽 {p.W*1000:.0f} mm 横跨两道槽壁（外沿间距 {(p.slot_w + 2 * p.wall_t)*1000:.0f} mm），"
             f"被槽壁顶面托住，插入深度为 0；强行放到插入深度时与槽壁的有符号距离见上表 0° 一行。")
    text = "\n".join(L) + "\n"
    out_md.write_text(text)
    print(text)
    return True


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--search", action="store_true")
    ap.add_argument("--write", action="store_true")
    ap.add_argument("--verify", action="store_true")
    args = ap.parse_args()

    if args.write:
        write_scene(CHOSEN)
    if args.verify:
        ok = verify(ROOT / "docs" / "slot_design_tables.md")
        sys.exit(0 if ok else 1)
    if args.search:
        grid = dict(y_start=[-0.40, -0.45, -0.50], slot_dy=[0.10, 0.15, 0.20], slot_h=[0.15, 0.20, 0.25],
                    roll_sign=[1, -1])
        results = []
        base = SlotParams()
        for vals in itertools.product(*grid.values()):
            p = replace(base, **dict(zip(grid.keys(), vals)))
            S, cfg = load_scene(p)
            ok, rows, flips = evaluate(S, cfg, n_per_seg=4)
            s = summarize(ok, rows)
            results.append((score(s), p, s))
            tag = " ".join(f"{k}={v}" for k, v in zip(grid.keys(), vals))
            if s["feasible"]:
                print(f"{tag}: margin {s['margin']:.3f} cond {s['cond']:.1f} σv {s['smin_v']:.3f} "
                      f"d_arms {s['d_arms']*100:.1f} d_table {s['d_table']*100:.1f} d_frame {s['d_frame']*100:.1f} "
                      f"d_post {s['d_post']*100:.1f} d_riser {s['d_riser']*100:.1f} cm → score {score(s):.3f}", flush=True)
            else:
                print(f"{tag}: INFEASIBLE", flush=True)
        results.sort(key=lambda r: -r[0])
        print("\nBEST:", results[0][1])


if __name__ == "__main__":
    main()
