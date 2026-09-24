#!/usr/bin/env python3
"""执行层偏差实验：规划好的路径在执行时遇到规划器看不到的偏差，对比三种执行配置。

场景 slot_gate（刚性抓取、关闭接触；另加一组接触夹取）：
  S0 无偏差 / S1 障碍模型误差（规划器以为横梁高 3 cm）/ S2 穿梁时向上 20 N 推力 / S3 两者叠加
执行配置：
  plan_only   只按规划执行（关闭 CBF 与力矩 QP 碰撞约束）
  plan_qp     规划 + 力矩 QP 碰撞约束（关闭 CBF）
  plan_cbf_qp 规划 + CBF 参考滤波 + 力矩 QP（完整架构）

输出 artifacts/execution_deviation/<时间戳>/{summary.md, results.json, <run>_console.txt}。
"""
from __future__ import annotations

import argparse
import csv
import json
import math
import re
import subprocess
from datetime import datetime
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
GOAL = (0.0, -0.65, 1.051)
MODEL_ERROR = ["planner.model_error_body=gate", "planner.model_error_offset=[0,0,0.03]"]
PUSH = ("disturbances=[{enabled: true, t_start: 3.8, t_end: 4.8, ramp: 0.1, frame: world, "
        "force: [0.0, 0.0, 20.0], torque: [0.0, 0.0, 0.0]}]")
NO_PUSH = "disturbances=[]"
DEVIATIONS = {
    "S0": ("无偏差", [NO_PUSH]),
    "S1": ("障碍模型误差（规划器以为横梁高 3 cm）", MODEL_ERROR + [NO_PUSH]),
    "S2": ("穿梁时向上 20 N 推力", [PUSH]),
    "S3": ("模型误差 + 推力", MODEL_ERROR + [PUSH]),
}
CONFIGS = {
    "plan_only": ("只按规划执行", ["controller.torque_qp.reference_governor.enabled=false",
                                 "controller.torque_qp.collision_avoidance_enabled=false"]),
    "plan_qp": ("规划 + 力矩 QP", ["controller.torque_qp.reference_governor.enabled=false"]),
    "plan_cbf_qp": ("规划 + CBF + 力矩 QP", []),
}


def run(name: str, overrides: list[str], out: Path, contact: bool) -> tuple[Path | None, str]:
    command = [str(ROOT / "build" / "run_sim"), "--scene", "slot_gate", "--controller", "qp_coop",
               "--headless", "--duration", "16" if contact else "14"]
    if contact:
        command += ["--contact-grasp", "--check-contact"]
    else:
        command += ["--set", "simulation.contacts=false"]
    for o in overrides:
        command += ["--set", o]
    print(f"[deviation] run {name}", flush=True)
    process = subprocess.run(command, cwd=ROOT, capture_output=True, text=True)
    console = process.stdout + process.stderr
    (out / f"{name}_console.txt").write_text(console, encoding="utf-8")
    match = re.search(r"log: (\S+)", console)
    return (Path(match.group(1)) if match else None), console


def metrics(log_dir: Path, console: str) -> dict:
    with (log_dir / "log.csv").open() as f:
        rows = [{k: float(v) for k, v in r.items()} for r in csv.DictReader(f)]

    def norm(r, keys):
        return math.sqrt(sum(r[k] ** 2 for k in keys))

    t = [r["t"] for r in rows]
    goal_err = [math.dist((r["obj_x"], r["obj_y"], r["obj_z"]), GOAL) for r in rows]
    reach = next((ti for ti, e in zip(t, goal_err) if e < 0.002), None)
    window = [r for r in rows if 2.0 < r["t"] < 12.0]
    result = {
        "object_gate_min_mm": 1e3 * min(r["d_object~gate"] for r in rows),
        "arms_gate_min_mm": 1e3 * min(min(r["d_left_arm~gate"], r["d_right_arm~gate"]) for r in rows),
        "steps_below_9_5mm": sum(r["d_object~gate"] < 0.0095 for r in rows),
        "reach_time_s": reach,
        "final_error_mm": 1e3 * goal_err[-1],
        "max_tracking_mm": 1e3 * max(norm(r, ["obj_err_x", "obj_err_y", "obj_err_z"]) for r in rows),
        "max_internal_force_n": max(norm(r, ["int_l_fx", "int_l_fy", "int_l_fz"]) for r in window),
        "max_cbf_offset_mm": 1e3 * max(r["governor_offset"] for r in rows),
    }
    loss = re.search(r"grasp loss: (\d+) steps; max TCP-grasp position error ([\d.]+) mm", console)
    if loss:
        result["grasp_loss_steps"] = int(loss.group(1))
        result["max_tcp_grasp_mm"] = float(loss.group(2))
    if "CONTACT TASK CHECK" in console:
        result["contact_check"] = "PASSED" in console
    return result


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--out", help="artifact directory")
    args = parser.parse_args()
    out = Path(args.out) if args.out else ROOT / "artifacts" / "execution_deviation" / datetime.now().strftime("%Y%m%d_%H%M%S")
    out.mkdir(parents=True, exist_ok=True)

    results = {}
    for dev, (dev_label, dev_overrides) in DEVIATIONS.items():
        configs = {"plan_cbf_qp": CONFIGS["plan_cbf_qp"]} if dev == "S0" else CONFIGS
        for cfg, (cfg_label, cfg_overrides) in configs.items():
            name = f"{dev}_{cfg}"
            log_dir, console = run(name, dev_overrides + cfg_overrides, out, contact=False)
            results[name] = {"deviation": dev_label, "config": cfg_label,
                             **(metrics(log_dir, console) if log_dir else {"error": "run failed"})}
    for cfg in ("plan_qp", "plan_cbf_qp"):
        name = f"S1_contact_{cfg}"
        log_dir, console = run(name, MODEL_ERROR + [NO_PUSH] + CONFIGS[cfg][1], out, contact=True)
        results[name] = {"deviation": DEVIATIONS["S1"][0] + "，接触夹取", "config": CONFIGS[cfg][0],
                         **(metrics(log_dir, console) if log_dir else {"error": "run failed"})}

    (out / "results.json").write_text(json.dumps(results, indent=2, ensure_ascii=False), encoding="utf-8")

    def fmt(v, spec="{:.1f}"):
        return "—" if v is None else spec.format(v)

    lines = ["# 执行层偏差实验（slot_gate）", "",
             "规划器事先规划好路径；执行时出现规划器看不到的偏差。距离为 plant（真实环境）下的最小有符号距离，负值 = 穿透。", "",
             "| 偏差 | 执行配置 | 板~横梁 [mm] | 两臂~横梁 [mm] | < 9.5 mm 步数 | 最大跟踪误差 [mm] | 最大内力 [N] | 到达 [s] | CBF 最大修正 [mm] |",
             "|---|---|---:|---:|---:|---:|---:|---:|---:|"]
    for r in (v for k, v in results.items() if "contact" not in k and "error" not in v):
        lines.append(f"| {r['deviation']} | {r['config']} | {fmt(r['object_gate_min_mm'])} | {fmt(r['arms_gate_min_mm'])} | "
                     f"{r['steps_below_9_5mm']} | {fmt(r['max_tracking_mm'])} | {fmt(r['max_internal_force_n'])} | "
                     f"{fmt(r['reach_time_s'], '{:.2f}')} | {fmt(r['max_cbf_offset_mm'], '{:.0f}')} |")
    lines += ["", "接触夹取（S1）：", "",
              "| 执行配置 | 任务验收 | 失接触步数 | TCP–抓取点最大偏差 [mm] | 终态误差 [mm] |", "|---|---|---:|---:|---:|"]
    for k, r in results.items():
        if "contact" in k and "error" not in r:
            lines.append(f"| {r['config']} | {'通过' if r.get('contact_check') else '失败'} | {r.get('grasp_loss_steps', '—')} | "
                         f"{fmt(r.get('max_tcp_grasp_mm'))} | {fmt(r['final_error_mm'])} |")
    (out / "summary.md").write_text("\n".join(lines) + "\n", encoding="utf-8")
    print(f"[deviation] report: {out}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
