#!/usr/bin/env python3
"""汇总 slot_avoid A/B 日志，生成机器可读指标与作品集 Markdown。"""

from __future__ import annotations

import argparse
import csv
import json
import math
from pathlib import Path


def load_log(path: str) -> tuple[Path, list[dict[str, float]]]:
    source = Path(path)
    if source.is_dir():
        source = source / "log.csv"
    with source.open(newline="") as stream:
        rows = [
            {key: float(value) for key, value in row.items()}
            for row in csv.DictReader(stream)
        ]
    if not rows:
        raise RuntimeError(f"empty log: {source}")
    return source, rows


def norm(row: dict[str, float], names: list[str]) -> float:
    return math.sqrt(sum(row[name] ** 2 for name in names))


def percentile(values: list[float], probability: float) -> float:
    ordered = sorted(values)
    index = probability * (len(ordered) - 1)
    lower = int(math.floor(index))
    upper = int(math.ceil(index))
    if lower == upper:
        return ordered[lower]
    weight = index - lower
    return ordered[lower] * (1.0 - weight) + ordered[upper] * weight


def summarize(rows: list[dict[str, float]]) -> dict[str, float | int | list]:
    position_error = [
        norm(row, ["obj_err_x", "obj_err_y", "obj_err_z"])
        for row in rows
    ]
    orientation_error = [
        norm(row, ["obj_err_rx", "obj_err_ry", "obj_err_rz"])
        for row in rows
    ]
    weld_left = [
        norm(row, ["l_weld_fx", "l_weld_fy", "l_weld_fz"])
        for row in rows
    ]
    weld_right = [
        norm(row, ["r_weld_fx", "r_weld_fy", "r_weld_fz"])
        for row in rows
    ]
    timing = [row["t_ctrl_us"] for row in rows]
    phases = [int(round(row.get("governor_phase", 0.0))) for row in rows]
    offsets = [row.get("governor_offset", 0.0) for row in rows]
    transitions = []
    previous = None
    phase_names = ["NORMAL", "LIFT", "CROSS", "DESCEND"]
    for row, phase in zip(rows, phases):
        if phase != previous:
            transitions.append(
                {
                    "time_s": row["t"],
                    "phase": phase_names[phase],
                    "offset_mm": 1e3 * row.get("governor_offset", 0.0),
                    "virtual_time_s": row.get("governor_virtual_time", row["t"]),
                }
            )
            previous = phase
    triggers = sum(
        phase == 1 and (index == 0 or phases[index - 1] == 0)
        for index, phase in enumerate(phases)
    )
    dt = (rows[-1]["t"] - rows[0]["t"]) / max(len(rows) - 1, 1)
    return {
        "samples": len(rows),
        "duration_s": rows[-1]["t"] - rows[0]["t"] + dt,
        "object_barrier_min_mm": 1e3 * min(row["d_object~barrier"] for row in rows),
        "global_min_distance_mm": 1e3 * min(row["d_min"] for row in rows),
        "position_error_max_mm": 1e3 * max(position_error),
        "position_error_final_mm": 1e3 * position_error[-1],
        "orientation_error_final_deg": math.degrees(orientation_error[-1]),
        "weld_force_max_left_n": max(weld_left),
        "weld_force_max_right_n": max(weld_right),
        "controller_mean_us": sum(timing) / len(timing),
        "controller_p99_us": percentile(timing, 0.99),
        "controller_max_us": max(timing),
        "governor_triggers": triggers,
        "governor_max_offset_mm": 1e3 * max(offsets),
        "governor_active_s": dt * sum(phase != 0 for phase in phases),
        "governor_transitions": transitions,
    }


def fmt(value: float, digits: int = 2) -> str:
    return f"{value:.{digits}f}"


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("baseline", help="coop 运行目录或 log.csv")
    parser.add_argument("autonomous", help="qp_coop 运行目录或 log.csv")
    parser.add_argument("--out", required=True, help="输出目录")
    args = parser.parse_args()

    baseline_path, baseline_rows = load_log(args.baseline)
    autonomous_path, autonomous_rows = load_log(args.autonomous)
    baseline = summarize(baseline_rows)
    autonomous = summarize(autonomous_rows)

    checks = {
        "baseline_collides": baseline["object_barrier_min_mm"] < 0.0,
        "autonomous_clears_barrier": autonomous["object_barrier_min_mm"] >= 9.5,
        "single_governor_trigger": autonomous["governor_triggers"] == 1,
        "complete_governor_sequence": all(
            name in [item["phase"] for item in autonomous["governor_transitions"]]
            for name in ["LIFT", "CROSS", "DESCEND"]
        ),
        "final_position_error_below_2mm": autonomous["position_error_final_mm"] < 2.0,
        "controller_p99_below_1ms": autonomous["controller_p99_us"] < 1000.0,
    }
    passed = all(checks.values())
    output = {
        "baseline_log": str(baseline_path.resolve()),
        "autonomous_log": str(autonomous_path.resolve()),
        "baseline": baseline,
        "autonomous": autonomous,
        "acceptance_checks": checks,
        "passed": passed,
    }

    out = Path(args.out)
    out.mkdir(parents=True, exist_ok=True)
    (out / "metrics.json").write_text(
        json.dumps(output, indent=2, ensure_ascii=False) + "\n",
        encoding="utf-8",
    )

    status = "PASS" if passed else "FAIL"
    lines = [
        "# Slot Avoidance A/B Experiment",
        "",
        f"Overall result: **{status}**",
        "",
        "| Metric | Straight coop | Autonomous QP |",
        "|---|---:|---:|",
        f"| Object-barrier minimum distance [mm] | {fmt(baseline['object_barrier_min_mm'])} | {fmt(autonomous['object_barrier_min_mm'])} |",
        f"| Global minimum distance [mm] | {fmt(baseline['global_min_distance_mm'])} | {fmt(autonomous['global_min_distance_mm'])} |",
        f"| Maximum tracking error [mm] | {fmt(baseline['position_error_max_mm'])} | {fmt(autonomous['position_error_max_mm'])} |",
        f"| Final tracking error [mm] | {fmt(baseline['position_error_final_mm'])} | {fmt(autonomous['position_error_final_mm'])} |",
        f"| Maximum weld force L/R [N] | {fmt(baseline['weld_force_max_left_n'])} / {fmt(baseline['weld_force_max_right_n'])} | {fmt(autonomous['weld_force_max_left_n'])} / {fmt(autonomous['weld_force_max_right_n'])} |",
        f"| Controller mean / P99 [us] | {fmt(baseline['controller_mean_us'], 1)} / {fmt(baseline['controller_p99_us'], 1)} | {fmt(autonomous['controller_mean_us'], 1)} / {fmt(autonomous['controller_p99_us'], 1)} |",
        f"| Governor triggers / max offset [mm] | 0 / 0 | {autonomous['governor_triggers']} / {fmt(autonomous['governor_max_offset_mm'])} |",
        "",
        "## Governor transitions",
        "",
        "| Time [s] | Phase | Offset [mm] | Virtual time [s] |",
        "|---:|---|---:|---:|",
    ]
    for transition in autonomous["governor_transitions"]:
        lines.append(
            f"| {transition['time_s']:.3f} | {transition['phase']} | "
            f"{transition['offset_mm']:.1f} | {transition['virtual_time_s']:.3f} |"
        )
    lines += ["", "## Acceptance checks", ""]
    lines += [f"- [{'x' if ok else ' '}] `{name}`" for name, ok in checks.items()]
    (out / "summary.md").write_text("\n".join(lines) + "\n", encoding="utf-8")

    print(f"portfolio experiment: {status}")
    print(f"  metrics: {out / 'metrics.json'}")
    print(f"  summary: {out / 'summary.md'}")
    if not passed:
        raise SystemExit(1)


if __name__ == "__main__":
    main()
