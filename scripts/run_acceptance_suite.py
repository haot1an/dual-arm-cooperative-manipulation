#!/usr/bin/env python3
"""Run all configured task-level scenarios and evaluate their acceptance criteria."""

from __future__ import annotations

import argparse
import json
import subprocess
import sys
from datetime import datetime
from pathlib import Path
from typing import Any

from check_acceptance import evaluate_scenario
from run_metrics import analyze_log

ROOT = Path(__file__).resolve().parents[1]


def run_and_tee(command: list[str], output: Path) -> str:
    with output.open("w", encoding="utf-8") as stream:
        process = subprocess.Popen(
            command,
            cwd=ROOT,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            text=True,
            bufsize=1,
        )
        assert process.stdout is not None
        lines = []
        for line in process.stdout:
            print(line, end="")
            stream.write(line)
            lines.append(line)
        code = process.wait()
    if code:
        raise RuntimeError(f"command failed ({code}): {' '.join(command)}")
    return "".join(lines)


def generated_run(output: str) -> Path:
    prefix = "[run_sim] log: "
    paths = [line[len(prefix):].strip() for line in output.splitlines() if line.startswith(prefix)]
    if not paths:
        raise RuntimeError("run_sim output did not contain a log directory")
    return Path(paths[-1])


def markdown(results: list[dict[str, Any]]) -> str:
    lines = [
        "# Task-level Acceptance Suite",
        "",
        f"Overall result: **{'PASS' if all(item['passed'] for item in results) else 'FAIL'}**",
        "",
        "| Scenario | Result | Final position error [mm] | Global minimum distance [mm] | P99 [us] |",
        "|---|---|---:|---:|---:|",
    ]
    for result in results:
        metrics = result["metrics"]
        lines.append(
            f"| {result['scenario']} | {'PASS' if result['passed'] else 'FAIL'} | "
            f"{metrics['position_error_final_mm']:.3f} | "
            f"{metrics['distance_min_mm']['min']:.3f} | "
            f"{metrics['controller_p99_us']:.1f} |"
        )
    lines += ["", "## Checks", ""]
    for result in results:
        lines += [f"### {result['scenario']}", ""]
        for check in result["checks"]:
            mark = "x" if check["passed"] else " "
            lines.append(
                f"- [{mark}] `{check['metric']} {check['op']} {check['value']}` "
                f"(actual: `{check['actual']}`)"
            )
        lines.append("")
    return "\n".join(lines)


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", default="config/acceptance.json")
    parser.add_argument("--skip-build", action="store_true")
    parser.add_argument("--scenario", action="append", help="run only this named scenario")
    parser.add_argument("--out", help="artifact directory")
    args = parser.parse_args()

    config_path = ROOT / args.config
    configuration = json.loads(config_path.read_text(encoding="utf-8"))
    selected = args.scenario or list(configuration["scenarios"])
    unknown = [name for name in selected if name not in configuration["scenarios"]]
    if unknown:
        raise SystemExit(f"unknown scenarios: {', '.join(unknown)}")

    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    out = Path(args.out) if args.out else ROOT / "artifacts" / "acceptance" / stamp
    out.mkdir(parents=True, exist_ok=True)

    if not args.skip_build:
        subprocess.run(
            ["cmake", "-S", str(ROOT), "-B", str(ROOT / "build"), "-DCMAKE_BUILD_TYPE=Release"],
            check=True,
        )
        subprocess.run(
            ["cmake", "--build", str(ROOT / "build"), f"-j{max(1, (os_cpu_count()))}"],
            check=True,
        )

    results = []
    for name in selected:
        spec = configuration["scenarios"][name]
        run = spec["run"]
        command = [
            str(ROOT / "build" / "run_sim"),
            "--scene", run["scene"],
            "--controller", run["controller"],
            "--headless",
            "--duration", str(run["duration"]),
        ]
        for override in run.get("overrides", []):
            command += ["--set", override]
        print(f"[acceptance] run {name}")
        console = run_and_tee(command, out / f"{name}_console.txt")
        log_path = generated_run(console)
        source, metrics = analyze_log(log_path)
        result = evaluate_scenario(name, metrics, spec)
        result["log"] = str(source.resolve())
        results.append(result)
        print(f"[acceptance] {name}: {'PASS' if result['passed'] else 'FAIL'}")

    report = {
        "passed": all(item["passed"] for item in results),
        "configuration": str(config_path.resolve()),
        "results": results,
    }
    (out / "results.json").write_text(
        json.dumps(report, indent=2, ensure_ascii=False) + "\n",
        encoding="utf-8",
    )
    (out / "summary.md").write_text(markdown(results) + "\n", encoding="utf-8")
    print(f"[acceptance] report: {out}")
    if not report["passed"]:
        raise SystemExit(1)


def os_cpu_count() -> int:
    import os
    return os.cpu_count() or 1


if __name__ == "__main__":
    main()
