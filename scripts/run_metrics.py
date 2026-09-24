#!/usr/bin/env python3
"""Extract controller, task, safety and timing metrics from a run_sim CSV log."""

from __future__ import annotations

import argparse
import csv
import json
import math
from pathlib import Path
from typing import Any

TORQUE_LIMITS = [87.0, 87.0, 87.0, 87.0, 12.0, 12.0, 12.0]
PHASE_NAMES = ["NORMAL", "LIFT", "CROSS", "DESCEND", "CBF"]


def resolve_log(path: str | Path) -> Path:
    source = Path(path)
    return source / "log.csv" if source.is_dir() else source


def load_log(path: str | Path) -> tuple[Path, list[dict[str, float]]]:
    source = resolve_log(path)
    with source.open(newline="") as stream:
        rows = [
            {key: float(value) for key, value in row.items()}
            for row in csv.DictReader(stream)
        ]
    if not rows:
        raise RuntimeError(f"empty log: {source}")
    return source, rows


def vector_norm(row: dict[str, float], names: list[str]) -> float:
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


def phase_transitions(rows: list[dict[str, float]]) -> list[dict[str, Any]]:
    result: list[dict[str, Any]] = []
    previous = None
    for row in rows:
        phase = int(round(row.get("governor_phase", 0.0)))
        if phase != previous:
            result.append(
                {
                    "time_s": row["t"],
                    "phase": PHASE_NAMES[phase],
                    "offset_mm": 1e3 * row.get("governor_offset", 0.0),
                    "virtual_time_s": row.get("governor_virtual_time", row["t"]),
                }
            )
            previous = phase
    return result


def analyze_rows(rows: list[dict[str, float]]) -> dict[str, Any]:
    names = set(rows[0])
    position_error = [
        vector_norm(row, ["obj_err_x", "obj_err_y", "obj_err_z"])
        for row in rows
    ]
    orientation_error = [
        vector_norm(row, ["obj_err_rx", "obj_err_ry", "obj_err_rz"])
        for row in rows
    ]
    object_position = [
        (row["obj_x"], row["obj_y"], row["obj_z"])
        for row in rows
    ]
    initial_position = object_position[0]
    object_drift = [
        math.sqrt(sum((value[i] - initial_position[i]) ** 2 for i in range(3)))
        for value in object_position
    ]
    timing = [row["t_ctrl_us"] for row in rows]
    weld_left = [
        vector_norm(row, ["l_weld_fx", "l_weld_fy", "l_weld_fz"])
        for row in rows
    ]
    weld_right = [
        vector_norm(row, ["r_weld_fx", "r_weld_fy", "r_weld_fz"])
        for row in rows
    ]
    dt = (rows[-1]["t"] - rows[0]["t"]) / max(len(rows) - 1, 1)

    saturation_steps = 0
    torque_peak_ratio = 0.0
    for row in rows:
        saturated = False
        for arm in ("l", "r"):
            for joint, limit in enumerate(TORQUE_LIMITS, 1):
                ratio = abs(row[f"{arm}_tau{joint}"]) / limit
                torque_peak_ratio = max(torque_peak_ratio, ratio)
                saturated = saturated or ratio >= 1.0 - 1e-8
        saturation_steps += int(saturated)

    phases = [int(round(row.get("governor_phase", 0.0))) for row in rows]
    offsets = [row.get("governor_offset", 0.0) for row in rows]
    transitions = phase_transitions(rows)
    phase_sequence = [item["phase"] for item in transitions]
    triggers = sum(
        phase == 1 and (index == 0 or phases[index - 1] == 0)
        for index, phase in enumerate(phases)
    )

    metrics: dict[str, Any] = {
        "samples": len(rows),
        "duration_s": rows[-1]["t"] - rows[0]["t"] + dt,
        "position_error_max_mm": 1e3 * max(position_error),
        "position_error_final_mm": 1e3 * position_error[-1],
        "orientation_error_max_deg": math.degrees(max(orientation_error)),
        "orientation_error_final_deg": math.degrees(orientation_error[-1]),
        "object_drift_max_mm": 1e3 * max(object_drift),
        "object_drift_final_mm": 1e3 * object_drift[-1],
        "weld_force_max_left_n": max(weld_left),
        "weld_force_max_right_n": max(weld_right),
        "torque_peak_ratio": torque_peak_ratio,
        "torque_saturation_steps": saturation_steps,
        "controller_mean_us": sum(timing) / len(timing),
        "controller_p99_us": percentile(timing, 0.99),
        "controller_p999_us": percentile(timing, 0.999),
        "controller_max_us": max(timing),
        "controller_deadline_miss_count": sum(value > 1000.0 for value in timing),
        "controller_deadline_miss_rate": sum(value > 1000.0 for value in timing) / len(timing),
        "distance_min_mm": {
            name[2:]: 1e3 * min(row[name] for row in rows)
            for name in sorted(names)
            if name.startswith("d_")
        },
        "governor_triggers": triggers,
        "governor_max_offset_mm": 1e3 * max(offsets),
        "governor_active_s": dt * sum(phase != 0 for phase in phases),
        "governor_phase_sequence": phase_sequence,
        "governor_transitions": transitions,
    }

    optional_final = {
        "assembly_phase_final": "assembly_phase",
        "tightening_torque_ref_final_nm": "tightening_tau_ref",
        "tightening_torque_meas_final_nm": "tightening_tau_meas",
        "preload_ref_final_n": "preload_ref",
        "preload_meas_final_n": "preload_meas",
        "screw_rate_final_rad_s": "screw_rate",
        "screw_feed_final_mm": "screw_feed",
        "screw_resisting_torque_final_nm": "screw_tau_resist",
        "screw_lead_error_final_mm": "screw_lead_err",
    }
    for metric, column in optional_final.items():
        if column in names:
            scale = 1e3 if metric.endswith("_mm") else 1.0
            metrics[metric] = scale * rows[-1][column]
    if "screw_angle" in names:
        metrics["screw_angle_final_deg"] = math.degrees(rows[-1]["screw_angle"])
    return metrics


def analyze_log(path: str | Path) -> tuple[Path, dict[str, Any]]:
    source, rows = load_log(path)
    return source, analyze_rows(rows)


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("run", help="run directory or log.csv")
    parser.add_argument("--out", help="write metrics JSON to this path")
    args = parser.parse_args()
    source, metrics = analyze_log(args.run)
    result = {"log": str(source.resolve()), "metrics": metrics}
    encoded = json.dumps(result, indent=2, ensure_ascii=False) + "\n"
    if args.out:
        Path(args.out).write_text(encoded, encoding="utf-8")
    else:
        print(encoded, end="")


if __name__ == "__main__":
    main()
