#!/usr/bin/env python3
"""Run obstacle-avoidance ablations and robustness experiments."""

from __future__ import annotations

import argparse
import json
import os
import subprocess
import sys
from datetime import datetime
from pathlib import Path
from typing import Any

from run_metrics import analyze_log

ROOT = Path(__file__).resolve().parents[1]

CASES = [
    {
        "name": "straight_coop",
        "controller": "coop",
        "category": "ablation",
        "overrides": [],
    },
    {
        "name": "qp_only",
        "controller": "qp_coop",
        "category": "ablation",
        "overrides": ["controller.torque_qp.reference_governor.enabled=false"],
    },
    {
        "name": "governor_without_collision_damper",
        "controller": "qp_coop",
        "category": "ablation",
        "overrides": ["controller.torque_qp.collision_avoidance_enabled=false"],
    },
    {
        "name": "full_qp_governor",
        "controller": "qp_coop",
        "category": "ablation",
        "overrides": [],
    },
    {
        "name": "full_with_calibration_error",
        "controller": "qp_coop",
        "category": "robustness",
        "overrides": ["calibration_error.enabled=true"],
    },
    {
        "name": "full_with_disturbance",
        "controller": "qp_coop",
        "category": "robustness",
        "overrides": [
            "disturbances=[{enabled: true, t_start: 10.7, t_end: 11.7, ramp: 0.1, frame: world, force: [4.0, 0.0, 0.0], torque: [0.0, 0.0, 0.2]}]"
        ],
    },
    {
        "name": "full_combined_uncertainty",
        "controller": "qp_coop",
        "category": "robustness",
        "overrides": [
            "calibration_error.enabled=true",
            "disturbances=[{enabled: true, t_start: 10.7, t_end: 11.7, ramp: 0.1, frame: world, force: [4.0, 0.0, 0.0], torque: [0.0, 0.0, 0.2]}]",
        ],
    },
]


def run_case(case: dict[str, Any], out: Path) -> tuple[Path, dict[str, Any]]:
    command = [
        str(ROOT / "build" / "run_sim"),
        "--scene", "slot_avoid",
        "--controller", case["controller"],
        "--headless",
        "--duration", "20",
        "--set", "simulation.contacts=false",
        "--set", "disturbances=[]",
        "--set", "log.decimation=5",
        # 本矩阵研究的是状态机 governor 的消融；CBF 模式另见 acceptance 的 slot_avoid。
        "--set", "controller.torque_qp.reference_governor.mode=state_machine",
    ]
    for override in case["overrides"]:
        command += ["--set", override]

    console_path = out / f"{case['name']}_console.txt"
    lines = []
    print(f"[matrix] run {case['name']}")
    with console_path.open("w", encoding="utf-8") as stream:
        process = subprocess.Popen(
            command,
            cwd=ROOT,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            text=True,
            bufsize=1,
        )
        assert process.stdout is not None
        for line in process.stdout:
            print(line, end="")
            stream.write(line)
            lines.append(line)
        code = process.wait()
    if code:
        raise RuntimeError(f"experiment {case['name']} failed with exit code {code}")
    prefix = "[run_sim] log: "
    paths = [line[len(prefix):].strip() for line in lines if line.startswith(prefix)]
    if not paths:
        raise RuntimeError(f"experiment {case['name']} did not produce a log")
    source, metrics = analyze_log(paths[-1])
    return source, metrics


def classify(metrics: dict[str, Any]) -> dict[str, bool]:
    return {
        "barrier_clear": metrics["distance_min_mm"]["object~barrier"] >= 9.5,
        "slot_clear": metrics["distance_min_mm"]["object~slot_frame"] >= 0.4,
        "task_complete": (
            metrics["position_error_final_mm"] <= 2.0
            and metrics["orientation_error_final_deg"] <= 0.6
        ),
        "single_governor_trigger": metrics["governor_triggers"] == 1,
        "within_1khz_p99": metrics["controller_p99_us"] < 1000.0,
        "no_torque_saturation": metrics["torque_saturation_steps"] == 0,
    }


def make_markdown(results: list[dict[str, Any]], passed: bool) -> str:
    lines = [
        "# Obstacle Avoidance Ablation and Robustness Matrix",
        "",
        f"Overall result: **{'PASS' if passed else 'FAIL'}**",
        "",
        "`governor_without_collision_damper` still uses the QP for closed-chain and joint safety, "
        "but contributes zero collision-damper inequalities.",
        "",
        "| Case | Category | Barrier min [mm] | Final error [mm] | Gov. triggers | P99 [us] | Task |",
        "|---|---|---:|---:|---:|---:|---|",
    ]
    for result in results:
        metrics = result["metrics"]
        checks = result["classification"]
        lines.append(
            f"| {result['name']} | {result['category']} | "
            f"{metrics['distance_min_mm']['object~barrier']:.2f} | "
            f"{metrics['position_error_final_mm']:.2f} | "
            f"{metrics['governor_triggers']} | {metrics['controller_p99_us']:.1f} | "
            f"{'PASS' if checks['task_complete'] and checks['barrier_clear'] else 'FAIL'} |"
        )
    lines += [
        "",
        "## Interpretation",
        "",
        "- `straight_coop` demonstrates that the unmodified task reference intersects the barrier.",
        "- `qp_only` shows whether a local acceleration damper can complete a topologically blocked path.",
        "- `governor_without_collision_damper` isolates online reference modification from collision inequalities.",
        "- `full_qp_governor` combines path adaptation with joint-level safety constraints.",
        "- Robustness cases inject plant/controller base mismatch, an external wrench, or both.",
    ]
    return "\n".join(lines) + "\n"


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--skip-build", action="store_true")
    parser.add_argument("--ablation-only", action="store_true")
    parser.add_argument("--out", help="artifact directory")
    args = parser.parse_args()

    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    out = Path(args.out) if args.out else ROOT / "artifacts" / "experiment_matrix" / stamp
    out.mkdir(parents=True, exist_ok=True)
    if not args.skip_build:
        subprocess.run(
            ["cmake", "-S", str(ROOT), "-B", str(ROOT / "build"), "-DCMAKE_BUILD_TYPE=Release"],
            check=True,
        )
        subprocess.run(
            ["cmake", "--build", str(ROOT / "build"), f"-j{os.cpu_count() or 1}"],
            check=True,
        )

    cases = [case for case in CASES if not args.ablation_only or case["category"] == "ablation"]
    results = []
    for case in cases:
        log, metrics = run_case(case, out)
        results.append(
            {
                "name": case["name"],
                "category": case["category"],
                "log": str(log.resolve()),
                "metrics": metrics,
                "classification": classify(metrics),
            }
        )

    by_name = {item["name"]: item for item in results}
    required = ["full_qp_governor"] + [
        case["name"] for case in cases if case["category"] == "robustness"
    ]
    passed = (
        by_name["straight_coop"]["metrics"]["distance_min_mm"]["object~barrier"] < 0.0
        and all(
            by_name[name]["classification"]["barrier_clear"]
            and by_name[name]["classification"]["slot_clear"]
            and by_name[name]["classification"]["task_complete"]
            and by_name[name]["classification"]["single_governor_trigger"]
            and by_name[name]["classification"]["within_1khz_p99"]
            and by_name[name]["classification"]["no_torque_saturation"]
            for name in required
        )
    )
    report = {"passed": passed, "results": results}
    (out / "results.json").write_text(
        json.dumps(report, indent=2, ensure_ascii=False) + "\n",
        encoding="utf-8",
    )
    (out / "summary.md").write_text(make_markdown(results, passed), encoding="utf-8")

    ablations = [item for item in results if item["category"] == "ablation"]
    subprocess.run(
        [
            sys.executable,
            str(ROOT / "scripts" / "plot_log.py"),
            *[str(Path(item["log"]).parent) for item in ablations],
            "--labels",
            *[item["name"] for item in ablations],
            "--out",
            str(out / "plots"),
        ],
        check=True,
    )
    print(f"[matrix] {'PASS' if passed else 'FAIL'}: {out}")
    if not passed:
        raise SystemExit(1)


if __name__ == "__main__":
    main()
