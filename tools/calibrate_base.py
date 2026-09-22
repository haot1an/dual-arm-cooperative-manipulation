#!/usr/bin/env python3
"""基座布局标定：在“肩距 × 内偏角”网格上搜索三台场景共用的基座位姿。

    python3 tools/calibrate_base.py                 # 搜索并打印评估表（写 docs/base_calibration_table.md）
    python3 tools/calibrate_base.py --write         # 另外把最优布局写进 config/default.yaml 与 models/workcell.xml，
                                                    # 并为每个场景求解 q_init（写进 config/scenes/*.yaml 与场景 XML 关键帧）

对每组候选布局、每个场景（lift / slot / assembly）：
  1. 由场景航点（物体位姿序列）+ 抓取点得到两臂 TCP 目标序列（航点之间插值，滚转段/拧螺钉段加密采样）；
  2. 每条臂：在多个起始构型（含另一条臂解的镜像、两种等价手指方向）中，选“沿路径最小关节运动跟踪”
     后最坏裕度最好的一条；
  3. 指标：全部航点可达、各关节到限位的最小归一化裕度、雅可比最小奇异值 / 最大条件数、
     两臂最小距离、两臂与台面最小距离、两臂与场景障碍（obstacles.pairs）的最小距离。
综合得分见 score()：可行性是硬约束，其余按“最坏情况”加权。
"""
from __future__ import annotations

import argparse
import math
import multiprocessing as mp
import re
import sys
from pathlib import Path

import numpy as np
import yaml
from scipy.spatial.transform import Rotation as Rot, Slerp

sys.path.insert(0, str(Path(__file__).resolve().parent))
from dual_arm_kin import (ARMS, OBJ_BODY, OBJ_SITE, ROOT, Scene, best_path, compose, load_config,  # noqa: E402
                          make_layout, mirror_q, path_quality, rpy_matrix, track_path)

SCENES = ("lift", "slot", "assembly")
SHOULDERS = (0.55, 0.60, 0.65, 0.70, 0.75)
INWARD_DEG = (0.0, 5.0, 10.0, 15.0, 20.0, 25.0, 30.0, 35.0, 40.0)
FLIP = np.diag([-1.0, -1.0, 1.0])
Q7_LO, Q7_HI = -2.8973, 2.8973


# ---------------------------------------------------------------------------
# TCP 目标
# ---------------------------------------------------------------------------
def object_path(wps, n_seg):
    """物体位姿插值（位置线性 + 姿态 slerp）；返回 [(名字, (p, R), 是否航点, 螺钉角)]"""
    out = []
    for a, b in zip(wps, wps[1:]):
        Ra, Rb = rpy_matrix(a["rpy_deg"]), rpy_matrix(b["rpy_deg"])
        sl = Slerp([0, 1], Rot.from_matrix(np.stack([Ra, Rb])))
        roll = np.linalg.norm(Rot.from_matrix(Rb @ Ra.T).as_rotvec())
        psi_a, psi_b = a.get("screw_angle_deg", 0.0), b.get("screw_angle_deg", 0.0)
        n = max(n_seg, int(math.ceil(max(roll, math.radians(abs(psi_b - psi_a))) / math.radians(15))))
        for k in range(n):
            s = k / n
            p = (1 - s) * np.array(a["pos"], float) + s * np.array(b["pos"], float)
            out.append((a["name"] if k == 0 else f"{a['name']}→{b['name']}", (p, sl([s]).as_matrix()[0]), k == 0,
                        (1 - s) * psi_a + s * psi_b))
    last = wps[-1]
    out.append((last["name"], (np.array(last["pos"], float), rpy_matrix(last["rpy_deg"])), True,
                last.get("screw_angle_deg", 0.0)))
    return out


def assembly_right_targets(S: Scene, path, q7_target):
    """装配场景右臂：工具尖端在螺钉头顶、工具轴竖直；起始 TCP 的偏航角由“q7 钉在 q7_target”决定，
    之后随螺钉转角 ψ 绕螺钉轴旋转并按导程下移。返回 (targets, q_start) 或 (None, None)。"""
    sc = S.cfg["scene"]
    lead = next((c["lead"] for c in sc["constraints"] if c["type"] == "screw"), 0.0)
    S.set_free_body(sc["object"]["body"], path[0][1])
    S.forward()
    p_head = S.m.site_xpos[S.m.id(OBJ_SITE, "screw_head_top")].copy()
    p_grasp, _ = S.site_in_body("grasp_R")  # 工具系中的抓取点（工具原点 = 尖端）
    p_tcp0 = p_head + np.array([0, 0, p_grasp[2]])
    R_down = np.diag([1.0, -1.0, -1.0])
    q, _ = S.ik("right", (p_tcp0, R_down), [np.array([0, 0.3, 0, -2.0, 0, 2.3, q7_target])],
                free_yaw=True, q7_target=q7_target)
    if q is None:
        return None, None
    S.set_arm("right", q)
    S.forward()
    _, R0 = S.tcp_pose("right")
    targets = []
    for _, _, _, psi in path:
        a = math.radians(psi)
        Rz = Rot.from_rotvec([0, 0, a]).as_matrix()
        targets.append((p_tcp0 + np.array([0, 0, lead * a / (2 * math.pi)]), Rz @ R0))
    return targets, q


def arm_paths(S: Scene, scene_name: str, n_seg=3, flips=None):
    """两臂沿场景任务路径的连续 IK 解；返回 (path, {arm: sols}) 或 (path, None)。
    flips（可选 dict）返回每条臂是否选用了“绕 TCP z 轴翻转 180°”的等价抓法。"""
    sc = S.cfg["scene"]
    path = object_path(sc["waypoints"], n_seg)
    sols = {}
    flips = {} if flips is None else flips
    grasp_body = {arm: S.site_body(sc["grasps"][arm]["site"]) for arm in ARMS}
    for arm in ("right", "left", "right"):
        other = "left" if arm == "right" else "right"
        extra = [mirror_q(sols[other][0])] if other in sols else []
        best = ((sols[arm], path_quality(S, sols[arm], arm), flips.get(arm, False)) if arm in sols
                else (None, -math.inf, False))
        if scene_name.startswith("assembly") and arm == "right":
            if "right" in sols:
                continue
            for q7t in (-math.pi / 2, math.pi / 2):  # 180° 旋拧时 q7 以 0 为中心
                targets, q0 = assembly_right_targets(S, path, q7t)
                if targets is None:
                    continue
                qs = track_path(S, arm, targets, q0)
                if qs is not None:
                    qual = path_quality(S, qs, arm)
                    if qual > best[1]:
                        best = (qs, qual, False)
        else:
            Tg = S.site_in_body(sc["grasps"][arm]["site"])
            if grasp_body[arm] != sc["object"]["body"]:
                raise RuntimeError(f"{scene_name}: {arm} grasps {grasp_body[arm]}, not the object")
            for flip in (False, True):
                Tg2 = (Tg[0], Tg[1] @ FLIP) if flip else Tg
                targets = [compose(To, Tg2) for _, To, _, _ in path]
                qs, qual = best_path(S, arm, targets, extra)
                if qs is not None and qual > best[1]:
                    best = (qs, qual, flip)
        if best[0] is None:
            return path, None
        sols[arm], flips[arm] = best[0], best[2]
    return path, sols


def place_tool(S: Scene, q_right):
    """把工具 body 放到右手里（抓取点与 TCP 重合）"""
    S.set_arm("right", q_right)
    S.forward()
    Ttcp = S.tcp_pose("right")
    pg, Rg = S.site_in_body("grasp_R")
    S.set_free_body("tool", compose(Ttcp, (-Rg.T @ pg, Rg.T)))


def evaluate_scene(S: Scene, scene_name: str, flips=None):
    path, sols = arm_paths(S, scene_name, flips=flips)
    if sols is None:
        return dict(feasible=False), None, None
    sc = S.cfg["scene"]
    pairs = [(S.resolve(a), S.resolve(b), f"{a}~{b}") for a, b in sc["obstacles"]["pairs"]]
    res = dict(feasible=True, margin=1.0, smin=1e9, cond=0.0, d_arms=1.0, d_table=1.0, d_obst=1.0, d_obst_pair="",
               worst_joint="")
    has_tool = S.m.id(OBJ_BODY, "tool") >= 0
    for k, (name, To, is_wp, psi) in enumerate(path):
        S.set_free_body(sc["object"]["body"], To)
        for arm in ARMS:
            S.set_arm(arm, sols[arm][k])
        if has_tool:
            place_tool(S, sols["right"][k])
        S.forward()
        for arm in ARMS:
            mt = S.arm_metrics(arm, sols[arm][k])
            j = int(np.argmin(mt["margin_n"]))
            if mt["margin_n"][j] < res["margin"]:
                res["margin"], res["worst_joint"] = mt["margin_n"][j], f"{arm} j{j + 1} @ {name}"
            res["smin"] = min(res["smin"], mt["smin"])
            res["cond"] = max(res["cond"], mt["cond"])
        for arm in ARMS:  # arm_metrics 会改 qpos，这里恢复两臂
            S.set_arm(arm, sols[arm][k])
        if has_tool:
            place_tool(S, sols["right"][k])
        S.forward()
        for ga, gb, tag in pairs:
            dist, _ = S.min_distance(ga, gb)
            if tag == "left_arm~right_arm":
                res["d_arms"] = min(res["d_arms"], dist)
            elif "table_top" in tag:
                res["d_table"] = min(res["d_table"], dist)
            elif "arm" in tag and dist < res["d_obst"]:
                res["d_obst"], res["d_obst_pair"] = dist, tag
    return res, path, sols


def score(r):
    """综合得分：不可行 → −∞；否则 最小裕度 + 0.5·σ_min − 0.01·cond，间隙低于阈值时线性扣分"""
    if not r["feasible"]:
        return -math.inf
    pen = 0.0
    for k, lim in (("d_arms", 0.05), ("d_table", 0.02), ("d_obst", 0.02)):
        if r[k] < lim:
            pen += 5.0 * (lim - r[k]) + (0.5 if r[k] < 0 else 0.0)
    return r["margin"] + 0.5 * r["smin"] - 0.01 * r["cond"] - pen


def eval_layout(args):
    shoulder, inward = args
    layout = make_layout(shoulder, inward)
    out = {}
    for name in SCENES:
        cfg = load_config(name)
        S = Scene.load(cfg, layout)
        out[name], _, _ = evaluate_scene(S, name)
    return shoulder, inward, out


def aggregate(res):
    if not all(r["feasible"] for r in res.values()):
        return dict(feasible=False, score=-math.inf)
    agg = dict(feasible=True)
    agg["margin"] = min(r["margin"] for r in res.values())
    agg["smin"] = min(r["smin"] for r in res.values())
    agg["cond"] = max(r["cond"] for r in res.values())
    for k in ("d_arms", "d_table", "d_obst"):
        agg[k] = min(r[k] for r in res.values())
    agg["score"] = score(agg)
    worst = min(res.items(), key=lambda kv: kv[1]["margin"])
    agg["worst"] = f"{worst[0]}: {worst[1]['worst_joint']}"
    return agg


def table_md(rows, best):
    lines = ["| 肩距 [m] | 内偏角 [°] | 可行 | 最小裕度 | σ_min(J) | max cond(J) | 两臂最小距离 [cm] | 臂—台面 [cm] | 臂—障碍 [cm] | 得分 | 最差关节 |",
             "|---|---|---|---|---|---|---|---|---|---|---|"]
    for (s, a), agg in rows:
        mark = " **←**" if (s, a) == best else ""
        if not agg["feasible"]:
            lines.append(f"| {s:.2f} | {a:.0f} | ✗ | – | – | – | – | – | – | – | 有航点不可达 |")
            continue
        lines.append(f"| {s:.2f}{mark} | {a:.0f} | ✓ | {agg['margin']:.3f} | {agg['smin']:.3f} | {agg['cond']:.1f} | "
                     f"{agg['d_arms']*100:.1f} | {agg['d_table']*100:.1f} | {agg['d_obst']*100:.1f} | "
                     f"{agg['score']:.3f} | {agg['worst']} |")
    return "\n".join(lines)


# ---------------------------------------------------------------------------
# 写回配置
# ---------------------------------------------------------------------------
def write_layout(layout):
    p = ROOT / "config" / "default.yaml"
    s = p.read_text()
    for arm in ARMS:
        pos, rpy = layout[arm]["base_pos"], layout[arm]["base_rpy_deg"]
        s = re.sub(rf"(  {arm}:\n    base_pos: )\[[^\]]*\]", lambda m: m.group(1) + f"[{pos[0]:.4f}, {pos[1]:.4f}, {pos[2]:.4f}]", s)
        s = re.sub(rf"(  {arm}:\n    base_pos: [^\n]*\n    base_rpy_deg: )\[[^\]]*\]",
                   lambda m: m.group(1) + f"[{rpy[0]:.1f}, {rpy[1]:.1f}, {rpy[2]:.2f}]", s)
    p.write_text(s)
    w = ROOT / "models" / "workcell.xml"
    x = w.read_text()
    for arm in ARMS:
        pos, yaw = layout[arm]["base_pos"], math.radians(layout[arm]["base_rpy_deg"][2])
        x = re.sub(rf'<body name="{arm}_base" pos="[^"]*" euler="[^"]*">',
                   f'<body name="{arm}_base" pos="{pos[0]:.4f} {pos[1]:.4f} {pos[2]:.4f}" euler="0 0 {yaw:.6f}">', x)
    w.write_text(x)


def flip_grasp_site(xml_text: str, site: str) -> str:
    """把抓取点 site 绕自身 z 轴转 180°（quat 右乘 (0,0,0,1)）：Franka Hand 关于 TCP z 轴 180° 对称，两种抓法等价，
    这里让场景文件中的抓取点姿态与所选 IK 解中的 TCP 姿态一致（控制器与 nominal 模式的 weld 都以它为目标）。"""
    def repl(m):
        w, x, y, z = (float(v) for v in m.group(2).split())
        q = Rot.from_quat([x, y, z, w]) * Rot.from_euler("z", 180, degrees=True)
        x, y, z, w = q.as_quat()
        vals = " ".join(f"{(0.0 if abs(v) < 1e-9 else v):.6g}" for v in (w, x, y, z))
        return m.group(1) + vals + m.group(3)
    out, n = re.subn(rf'(<site name="{site}"[^>]*?quat=")([^"]*)(")', repl, xml_text)
    if n != 1:
        raise RuntimeError(f"site {site}: expected one quat attribute, found {n}")
    return out


def write_q_init(name, S: Scene, sols, flips=None):
    p = ROOT / "config" / "scenes" / f"{name}.yaml"
    s = p.read_text()
    for arm in ARMS:
        q = ", ".join(f"{v:.6f}" for v in sols[arm][0])
        s, n = re.subn(rf"(q_init:\n(?:  \w+:\s*\[[^\]]*\]\n)*?  {arm}:\s*)\[[^\]]*\]", lambda m: m.group(1) + f"[{q}]", s)
        if n != 1:
            raise RuntimeError(f"{p}: q_init.{arm} not found")
    p.write_text(s)
    # 关键帧：两臂 q_init + 物体/工具初始位姿（便于用 simulate 直接查看）
    sc = S.cfg["scene"]
    w0 = sc["waypoints"][0]
    S.set_free_body(sc["object"]["body"], (np.array(w0["pos"], float), rpy_matrix(w0["rpy_deg"])))
    for arm in ARMS:
        S.set_arm(arm, sols[arm][0])
    if S.m.id(OBJ_BODY, "tool") >= 0:
        place_tool(S, sols["right"][0])
    qpos = " ".join(f"{v:.6g}" for v in S.m.qpos)
    xml = ROOT / sc["model"]
    x = xml.read_text()
    for arm, flipped in (flips or {}).items():
        if flipped:
            x = flip_grasp_site(x, sc["grasps"][arm]["site"])
            print(f"  {name}: {arm} grasp site {sc['grasps'][arm]['site']} flipped 180° about z to match the IK solution")
    key = f'  <keyframe>\n    <key name="init" qpos="{qpos}"/>\n  </keyframe>\n</mujoco>'
    if "<keyframe>" in x:
        x = re.sub(r"  <keyframe>.*?</keyframe>\n</mujoco>", lambda _: key, x, flags=re.S)
    else:
        x = x.replace("</mujoco>", key)
    xml.write_text(x)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--write", action="store_true", help="把最优布局与各场景 q_init 写回配置")
    ap.add_argument("--layout", nargs=2, type=float, metavar=("SHOULDER", "INWARD_DEG"),
                    help="不搜索，直接使用给定布局（配合 --write）")
    args = ap.parse_args()

    if args.layout:
        best = tuple(args.layout)
        rows = []
    else:
        grid = [(s, a) for s in SHOULDERS for a in INWARD_DEG]
        with mp.Pool(min(len(grid), mp.cpu_count())) as pool:
            results = pool.map(eval_layout, grid)
        rows = []
        for s, a, res in results:
            agg = aggregate(res)
            rows.append(((s, a), agg))
            if agg["feasible"]:
                print(f"shoulder {s:.2f} inward {a:4.0f}°: margin {agg['margin']:.3f} σmin {agg['smin']:.3f} "
                      f"cond {agg['cond']:.1f} d_arms {agg['d_arms']*100:.1f} d_table {agg['d_table']*100:.1f} "
                      f"d_obst {agg['d_obst']*100:.1f} cm score {agg['score']:.3f}  [{agg['worst']}]", flush=True)
            else:
                print(f"shoulder {s:.2f} inward {a:4.0f}°: INFEASIBLE", flush=True)
            per = "  ".join(f"{n}: " + (f"m {r['margin']:.2f} c {r['cond']:.1f}" if r["feasible"] else "✗")
                            for n, r in res.items())
            print("    " + per)
        best = max(rows, key=lambda r: r[1]["score"])[0]
        md = table_md(rows, best)
        print("\n" + md)
        out = ROOT / "docs" / "base_calibration_table.md"
        out.write_text(md + "\n")
        print(f"\nbest: shoulder {best[0]:.2f} m, inward {best[1]:.0f}°  (table written to {out})")

    if args.write:
        layout = make_layout(*best)
        write_layout(layout)
        for name in SCENES + ("assembly_simple",):
            cfg = load_config(name)
            S = Scene.load(cfg, layout)
            flips = {}
            res, path, sols = evaluate_scene(S, name, flips)
            if sols is None:
                print(f"{name}: INFEASIBLE with chosen layout")
                continue
            write_q_init(name, S, sols, flips)
            print(f"{name}: q_init written  (margin {res['margin']:.3f}, cond {res['cond']:.1f})")


if __name__ == "__main__":
    main()
