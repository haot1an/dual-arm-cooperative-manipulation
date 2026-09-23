#!/usr/bin/env python3
"""Run a reproducible randomized robustness sweep for autonomous transport."""

from __future__ import annotations

import argparse
import json
import math
import os
import random
import subprocess
from datetime import datetime
from pathlib import Path
from typing import Any

from check_acceptance import evaluate_scenario
from run_acceptance_suite import generated_run
from run_metrics import analyze_log

ROOT = Path(__file__).resolve().parents[1]
DEFAULT_SEED = 20260923


def vector_text(values: list[float]) -> str:
    return "[" + ",".join(f"{value:.8f}" for value in values) + "]"


def sample_unit_vector(rng: random.Random) -> list[float]:
    azimuth = rng.uniform(-math.pi, math.pi)
    z = rng.uniform(-1.0, 1.0)
    radial = math.sqrt(max(0.0, 1.0 - z * z))
    return [radial * math.cos(azimuth), radial * math.sin(azimuth), z]


def sample_trial(rng: random.Random, index: int) -> dict[str, Any]:
    position = [
        rng.uniform(-0.004, 0.004),
        rng.uniform(-0.004, 0.004),
        rng.uniform(-0.003, 0.003),
    ]
    orientation = [
        rng.uniform(-0.20, 0.20),
        rng.uniform(-0.20, 0.20),
        rng.uniform(-0.50, 0.50),
    ]
    force_magnitude = rng.uniform(2.0, 6.0)
    force_direction = sample_unit_vector(rng)
    force = [force_magnitude * value for value in force_direction]
    torque = [rng.uniform(-0.25, 0.25) for _ in range(3)]
    start = rng.uniform(8.5, 12.0)
    duration = rng.uniform(0.6, 1.2)
    return {
        "trial": index,
        "calibration_position_m": position,
        "calibration_rpy_deg": orientation,
        "disturbance_force_n": force,
        "disturbance_torque_nm": torque,
        "disturbance_start_s": start,
        "disturbance_end_s": start + duration,
        "disturbance_ramp_s": min(0.1, 0.25 * duration),
    }


def overrides(trial: dict[str, Any]) -> list[str]:
    disturbance = (
        "disturbances=[{enabled: true, "
        f"t_start: {trial['disturbance_start_s']:.8f}, "
        f"t_end: {trial['disturbance_end_s']:.8f}, "
        f"ramp: {trial['disturbance_ramp_s']:.8f}, frame: world, "
        f"force: {vector_text(trial['disturbance_force_n'])}, "
        f"torque: {vector_text(trial['disturbance_torque_nm'])}}}]"
    )
    return [
        "calibration_error.enabled=true",
        "calibration_error.right_base_offset_xyz="
        + vector_text(trial["calibration_position_m"]),
        "calibration_error.right_base_offset_rpy_deg="
        + vector_text(trial["calibration_rpy_deg"]),
        disturbance,
        "log.decimation=10",
    ]


def run_trial(
    trial: dict[str, Any],
    acceptance: dict[str, Any],
    output_dir: Path,
) -> dict[str, Any]:
    run = acceptance["run"]
    command = [
        str(ROOT / "build" / "run_sim"),
        "--scene", run["scene"],
        "--controller", run["controller"],
        "--headless",
        "--duration", str(run["duration"]),
    ]
    for override in run.get("overrides", []) + overrides(trial):
        command += ["--set", override]

    completed = subprocess.run(
        command,
        cwd=ROOT,
        text=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        check=False,
    )
    console_path = output_dir / f"trial_{trial['trial']:03d}_console.txt"
    console_path.write_text(completed.stdout, encoding="utf-8")
    if completed.returncode:
        return {
            **trial,
            "passed": False,
            "process_exit_code": completed.returncode,
            "error": "run_sim failed",
            "console": str(console_path.resolve()),
        }

    log_path = generated_run(completed.stdout)
    source, metrics = analyze_log(log_path)
    evaluation = evaluate_scenario("slot_avoid", metrics, acceptance)
    return {
        **trial,
        "passed": evaluation["passed"],
        "process_exit_code": 0,
        "log": str(source.resolve()),
        "console": str(console_path.resolve()),
        "checks": evaluation["checks"],
        "metrics": metrics,
    }


def aggregate(results: list[dict[str, Any]]) -> dict[str, Any]:
    completed = [result for result in results if "metrics" in result]
    passed = sum(result["passed"] for result in results)
    summary: dict[str, Any] = {
        "trials": len(results),
        "passed_trials": passed,
        "pass_rate": passed / len(results),
        "process_failures": sum(result["process_exit_code"] != 0 for result in results),
    }
    if completed:
        summary.update(
            {
                "worst_barrier_distance_mm": min(
                    result["metrics"]["distance_min_mm"]["object~barrier"]
                    for result in completed
                ),
                "worst_slot_distance_mm": min(
                    result["metrics"]["distance_min_mm"]["object~slot_frame"]
                    for result in completed
                ),
                "worst_final_position_error_mm": max(
                    result["metrics"]["position_error_final_mm"]
                    for result in completed
                ),
                "worst_final_orientation_error_deg": max(
                    result["metrics"]["orientation_error_final_deg"]
                    for result in completed
                ),
                "worst_controller_p99_us": max(
                    result["metrics"]["controller_p99_us"]
                    for result in completed
                ),
                "torque_saturation_steps": sum(
                    result["metrics"]["torque_saturation_steps"]
                    for result in completed
                ),
            }
        )
    return summary


def markdown(
    seed: int,
    minimum_pass_rate: float,
    results: list[dict[str, Any]],
    summary: dict[str, Any],
    accepted: bool,
) -> str:
    lines = [
        "# Randomized Robustness Sweep",
        "",
        f"Result: **{'PASS' if accepted else 'FAIL'}**",
        "",
        f"- Seed: `{seed}`",
        f"- Trials: {summary['trials']}",
        f"- Pass rate: {100.0 * summary['pass_rate']:.1f}% "
        f"(required: {100.0 * minimum_pass_rate:.1f}%)",
        "- Calibration envelope: xyz = +/-[4, 4, 3] mm; "
        "rpy = +/-[0.2, 0.2, 0.5] deg",
        "- Disturbance envelope: force magnitude 2--6 N; "
        "each torque component +/-0.25 N m; randomized time and direction",
        "",
        "| Trial | Result | |dp| [mm] | |dR| [deg] | |F| [N] | "
        "Barrier [mm] | Final error [mm] | P99 [us] |",
        "|---:|---|---:|---:|---:|---:|---:|---:|",
    ]
    for result in results:
        dp = 1e3 * math.sqrt(sum(value * value for value in result["calibration_position_m"]))
        dr = math.sqrt(sum(value * value for value in result["calibration_rpy_deg"]))
        force = math.sqrt(sum(value * value for value in result["disturbance_force_n"]))
        if "metrics" not in result:
            lines.append(
                f"| {result['trial']} | ERROR | {dp:.2f} | {dr:.2f} | "
                f"{force:.2f} | n/a | n/a | n/a |"
            )
            continue
        metrics = result["metrics"]
        lines.append(
            f"| {result['trial']} | {'PASS' if result['passed'] else 'FAIL'} | "
            f"{dp:.2f} | {dr:.2f} | {force:.2f} | "
            f"{metrics['distance_min_mm']['object~barrier']:.2f} | "
            f"{metrics['position_error_final_mm']:.3f} | "
            f"{metrics['controller_p99_us']:.1f} |"
        )
    lines += [
        "",
        "## Worst case",
        "",
        f"- Barrier distance: {summary.get('worst_barrier_distance_mm', float('nan')):.3f} mm",
        f"- Slot-frame distance: {summary.get('worst_slot_distance_mm', float('nan')):.3f} mm",
        f"- Final position error: {summary.get('worst_final_position_error_mm', float('nan')):.3f} mm",
        f"- Final orientation error: {summary.get('worst_final_orientation_error_deg', float('nan')):.3f} deg",
        f"- Controller P99: {summary.get('worst_controller_p99_us', float('nan')):.1f} us",
        f"- Torque saturation steps: {summary.get('torque_saturation_steps', 0)}",
        "",
        "> This is a deterministic simulation sweep over a declared envelope, "
        "not a statistical guarantee for hardware.",
    ]
    return "\n".join(lines) + "\n"


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--trials", type=int, default=12)
    parser.add_argument("--seed", type=int, default=DEFAULT_SEED)
    parser.add_argument("--min-pass-rate", type=float, default=1.0)
    parser.add_argument("--skip-build", action="store_true")
    parser.add_argument("--out", help="artifact directory")
    args = parser.parse_args()
    if args.trials <= 0:
        raise SystemExit("trials must be positive")
    if not 0.0 <= args.min_pass_rate <= 1.0:
        raise SystemExit("min-pass-rate must be in [0, 1]")

    if not args.skip_build:
        subprocess.run(
            ["cmake", "-S", str(ROOT), "-B", str(ROOT / "build"), "-DCMAKE_BUILD_TYPE=Release"],
            check=True,
        )
        subprocess.run(
            ["cmake", "--build", str(ROOT / "build"), f"-j{os.cpu_count() or 1}"],
            check=True,
        )

    configuration = json.loads(
        (ROOT / "config" / "acceptance.json").read_text(encoding="utf-8")
    )
    acceptance = configuration["scenarios"]["slot_avoid"]
    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    output_dir = (
        Path(args.out)
        if args.out
        else ROOT / "artifacts" / "robustness_sweep" / stamp
    )
    output_dir.mkdir(parents=True, exist_ok=True)

    rng = random.Random(args.seed)
    results = []
    for index in range(args.trials):
        trial = sample_trial(rng, index)
        print(f"[robustness] trial {index + 1}/{args.trials}")
        result = run_trial(trial, acceptance, output_dir)
        results.append(result)
        print(f"[robustness] trial {index}: {'PASS' if result['passed'] else 'FAIL'}")

    summary = aggregate(results)
    accepted = summary["pass_rate"] >= args.min_pass_rate
    report = {
        "passed": accepted,
        "seed": args.seed,
        "minimum_pass_rate": args.min_pass_rate,
        "envelope": {
            "calibration_position_m": [[-0.004, 0.004], [-0.004, 0.004], [-0.003, 0.003]],
            "calibration_rpy_deg": [[-0.2, 0.2], [-0.2, 0.2], [-0.5, 0.5]],
            "disturbance_force_magnitude_n": [2.0, 6.0],
            "disturbance_torque_component_nm": [-0.25, 0.25],
            "disturbance_start_s": [8.5, 12.0],
            "disturbance_duration_s": [0.6, 1.2],
        },
        "summary": summary,
        "results": results,
    }
    (output_dir / "results.json").write_text(
        json.dumps(report, indent=2, ensure_ascii=False) + "\n",
        encoding="utf-8",
    )
    (output_dir / "summary.md").write_text(
        markdown(args.seed, args.min_pass_rate, results, summary, accepted),
        encoding="utf-8",
    )
    print(
        f"[robustness] {'PASS' if accepted else 'FAIL'}: "
        f"{summary['passed_trials']}/{summary['trials']} -> {output_dir}"
    )
    if not accepted:
        raise SystemExit(1)


if __name__ == "__main__":
    main()
