#!/usr/bin/env python3
"""Run a sustained soft-real-time benchmark for the 1 kHz qp_coop loop."""

from __future__ import annotations

import argparse
import json
import os
import platform
import subprocess
from datetime import datetime
from pathlib import Path

from run_metrics import analyze_rows, load_log

ROOT = Path(__file__).resolve().parents[1]


def cpu_model() -> str:
    try:
        for line in Path("/proc/cpuinfo").read_text().splitlines():
            if line.startswith("model name"):
                return line.split(":", 1)[1].strip()
    except OSError:
        pass
    return "unknown"


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--duration", type=float, default=60.0)
    parser.add_argument("--warmup", type=float, default=2.0)
    parser.add_argument("--cpu", type=int, help="pin run_sim to this CPU with taskset")
    parser.add_argument("--skip-build", action="store_true")
    parser.add_argument("--out", help="artifact directory")
    args = parser.parse_args()
    if args.duration <= args.warmup:
        raise SystemExit("duration must be greater than warmup")

    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    out = Path(args.out) if args.out else ROOT / "artifacts" / "realtime" / stamp
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

    command = [
        str(ROOT / "build" / "run_sim"),
        "--scene", "slot_avoid",
        "--controller", "qp_coop",
        "--headless",
        "--duration", str(args.duration),
        "--set", "simulation.contacts=false",
        "--set", "disturbances=[]",
        "--set", "log.decimation=1",
    ]
    if args.cpu is not None:
        command = ["taskset", "-c", str(args.cpu), *command]

    console = subprocess.run(
        command,
        cwd=ROOT,
        check=True,
        text=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
    ).stdout
    print(console, end="")
    (out / "console.txt").write_text(console, encoding="utf-8")
    prefix = "[run_sim] log: "
    logs = [line[len(prefix):].strip() for line in console.splitlines() if line.startswith(prefix)]
    if not logs:
        raise RuntimeError("run_sim did not report a log directory")
    source, rows = load_log(logs[-1])
    measured_rows = [row for row in rows if row["t"] >= args.warmup]
    metrics = analyze_rows(measured_rows)
    passed = (
        metrics["controller_p999_us"] < 1000.0
        and metrics["controller_deadline_miss_rate"] < 0.001
    )
    result = {
        "passed": passed,
        "classification": "soft-real-time measurement; not a hard-real-time guarantee",
        "log": str(source.resolve()),
        "duration_s": args.duration,
        "warmup_s": args.warmup,
        "cpu_affinity": args.cpu,
        "host": {
            "platform": platform.platform(),
            "cpu": cpu_model(),
            "logical_cpus": os.cpu_count(),
        },
        "metrics": metrics,
    }
    (out / "benchmark.json").write_text(
        json.dumps(result, indent=2, ensure_ascii=False) + "\n",
        encoding="utf-8",
    )
    lines = [
        "# Sustained 1 kHz Control Benchmark",
        "",
        f"Result: **{'PASS' if passed else 'FAIL'}**",
        "",
        "> Soft-real-time measurement on ordinary Linux; this is not a hard-real-time guarantee.",
        "",
        f"- Measured duration after warmup: {metrics['duration_s']:.1f} s",
        f"- Samples: {metrics['samples']}",
        f"- Mean: {metrics['controller_mean_us']:.1f} us",
        f"- P99: {metrics['controller_p99_us']:.1f} us",
        f"- P99.9: {metrics['controller_p999_us']:.1f} us",
        f"- Maximum: {metrics['controller_max_us']:.1f} us",
        f"- Deadline misses (>1000 us): {metrics['controller_deadline_miss_count']} "
        f"({100.0 * metrics['controller_deadline_miss_rate']:.4f}%)",
        f"- CPU affinity: {args.cpu if args.cpu is not None else 'not pinned'}",
        f"- CPU: {result['host']['cpu']}",
    ]
    (out / "summary.md").write_text("\n".join(lines) + "\n", encoding="utf-8")
    print(f"[benchmark] {'PASS' if passed else 'FAIL'}: {out}")
    if not passed:
        raise SystemExit(1)


if __name__ == "__main__":
    main()
