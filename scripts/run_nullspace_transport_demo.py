#!/usr/bin/env python3
"""Reproduce moving slot-avoid nullspace OFF/ON metrics and optionally a synchronized comparison video.

python3 scripts/run_nullspace_transport_demo.py --record --media-dir docs/img
Requires the viewer build, an available DISPLAY, ffmpeg, numpy, matplotlib and Pillow.
"""
from __future__ import annotations

import argparse
import json
import math
import shutil
import subprocess
from datetime import datetime
from pathlib import Path
from zoneinfo import ZoneInfo

import numpy as np

from plot_log import load_run
from run_metrics import analyze_log

ROOT = Path(__file__).resolve().parents[1]
# 插槽阶段：越墙后回到插槽上方、下插并保持（名义 pre_insert 之后）。肘部此时贴近两根圆柱。
INSERT_START_S = 14.0
COLORS = ("#3975bc", "#008873")          # 曲线图（白底）
VIDEO_COLORS = ("#5b9be6", "#2bb58f")    # 视频（深色底）
BG, INK, INK2, GRID = "#15181d", "#e8ecf1", "#a9b4c2", "#3a414b"


def run_case(out: Path, enabled: bool, record: bool, duration: float) -> dict:
    name = "enabled" if enabled else "baseline"
    command = [str(ROOT / "build/run_sim"), "--scene", "slot_avoid_nullspace", "--controller", "qp_coop",
               "--headless", "--duration", str(duration), "--no-calib-error", "--set",
               f"controller.coop.nullspace.enabled={'true' if enabled else 'false'}",
               "--set", f"log.dir={out / name}"]
    if record:
        command += ["--record", str(out / f"{name}.mp4"), "--record-fps", "30", "--camera", "cam_transport"]
    print(f"[nullspace] running {name}", flush=True)
    result = subprocess.run(command, cwd=ROOT, text=True, stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
    (out / f"{name}_console.txt").write_text(result.stdout, encoding="utf-8")
    if result.returncode:
        raise RuntimeError(f"{name} failed; see {out / (name + '_console.txt')}")
    prefix = "[run_sim] log: "
    paths = [line[len(prefix):] for line in result.stdout.splitlines() if line.startswith(prefix)]
    if not paths:
        raise RuntimeError(f"{name}: run_sim did not report a log")
    path, metrics = analyze_log(paths[-1])
    data = load_run(str(path), name)
    metrics["clearance_final_mm"] = {arm: float(data[f"d_{arm}_arm~pillar_{suffix}"][-1] * 1000)
                                    for arm, suffix in [("left", "L"), ("right", "R")]}
    insert = data.t >= INSERT_START_S
    metrics["clearance_insert_min_mm"] = {arm: float(np.min(data[f"d_{arm}_arm~pillar_{suffix}"][insert]) * 1000)
                                         for arm, suffix in [("left", "L"), ("right", "R")]}
    metrics["joint_displacement_norm_deg"] = {
        arm: float(np.degrees(np.linalg.norm([data[f"{prefix}_q{k}"][-1] - data[f"{prefix}_q{k}"][0]
                                              for k in range(1, 8)])))
        for arm, prefix in [("left", "l"), ("right", "r")]
    }
    # Task tolerances differ from a stationary object-hold experiment.
    checks = {
        "position_error": metrics["position_error_max_mm"] < 20,
        "orientation_error": metrics["orientation_error_max_deg"] < 2,
        "final_position": metrics["position_error_final_mm"] < 2,
        "final_orientation": metrics["orientation_error_final_deg"] < 0.6,
        "wall_clearance": metrics["distance_min_mm"]["object~barrier"] > 9.5,
        "slot_clearance": metrics["distance_min_mm"]["object~slot_frame"] > 0.4,
        "all_pairs_positive": min(metrics["distance_min_mm"].values()) > 0,
        "pillar_clearance": min(metrics["distance_min_mm"][f"{arm}_arm~pillar_{suffix}"]
                                 for arm, suffix in [("left", "L"), ("right", "R")]) > 10,
        "no_saturation": metrics["torque_saturation_steps"] == 0,
        "closed_chain": metrics["nullspace"]["closed_chain_residual_max"] < 0.002,
        "collision_slack": metrics["nullspace"]["collision_slack_max"] < 0.002,
        "projection": metrics["nullspace"]["acceleration_leak_max"] < 1e-10,
        "qp_valid": metrics["qp"]["invalid_steps"] == 0,
        "qp_final_solved": metrics["qp"]["status_final"] == 0,
        "qp_feasibility": metrics["qp"]["constraint_violation_max"] < 0.002,
        "cbf_active": metrics["governor_active_s"] > 1,
        "roll_completed": abs(abs(float(np.degrees(data["roll_obj"][-1]))) - 90) < 0.6,
        "inserted": abs(float(data["obj_y"][-1]) + 0.65) < 0.002
                    and abs(float(data["obj_z"][-1]) - 1.051) < 0.002,
    }
    if enabled:
        checks["insert_pillar_clearance"] = min(metrics["clearance_insert_min_mm"].values()) > 80
        checks["nullspace_active"] = metrics["nullspace"]["torque_norm_max_nm"] > 0.1
    print(f"[nullspace] {name}: max object error {metrics['position_error_max_mm']:.3f} mm; "
          f"insertion-phase min gaps {metrics['clearance_insert_min_mm']}", flush=True)
    return {"log": str(path), "command": command, "metrics": metrics, "checks": checks,
            "passed": all(checks.values())}


def series(run):
    gap = np.minimum(run["d_left_arm~pillar_L"], run["d_right_arm~pillar_R"]) * 1000
    error = np.sqrt(sum(run[f"obj_err_{k}"] ** 2 for k in "xyz")) * 1000
    return gap, error


def plot_comparison(out: Path, runs: list) -> None:
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    fig, axes = plt.subplots(2, 2, figsize=(11, 6), constrained_layout=True)
    for run, color in zip(runs, COLORS):
        gap, error = series(run)
        axes[0, 0].plot(run.t, gap, color=color, label=f"Nullspace {run.label}")
        axes[0, 1].plot(run.t, error, color=color)
        axes[1, 0].plot(run.t, run["ns_torque_norm"], color=color)
        axes[1, 1].semilogy(run.t, np.maximum(run["ns_acceleration_leak"], 1e-18), color=color)
    titles = ["Minimum arm-to-pillar clearance (capped at 120 mm)", "Object position error",
              "Projected nullspace torque", "Acceleration leak before torque QP"]
    units = ["Clearance [mm]", "Error [mm]", "Torque norm [N m]", "||J M^-1 tau_ns||"]
    for ax, title, unit in zip(axes.flat, titles, units):
        ax.set_title(title)
        ax.set_ylabel(unit)
        ax.set_xlabel("Simulation time [s]")
        ax.grid(alpha=.2)
    axes[0, 0].axhline(120, color="gray", linestyle="--", linewidth=.8)
    axes[0, 0].axvspan(INSERT_START_S, runs[0].t[-1], color="#f0b251", alpha=.12, label="Insertion phase")
    axes[0, 0].legend()
    fig.suptitle("Lift / roll / cross wall / insert + two pillars; only the nullspace switch changes")
    fig.savefig(out / "comparison.png", dpi=160)
    plt.close(fig)


def format_gap(value: float) -> str:
    return ">=120" if value >= 119.999 else f"{value:.1f}"


def compose_video(out: Path, runs: list, duration: float) -> None:
    from PIL import Image, ImageDraw, ImageFont
    width, height, fps = 1600, 960, 30
    font_path = "/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf"
    large = ImageFont.truetype(font_path, 28)
    medium = ImageFont.truetype(font_path, 22)
    small = ImageFont.truetype(font_path, 18)
    decoders = [subprocess.Popen(["ffmpeg", "-v", "error", "-i", str(out / f"{name}.mp4"),
                 "-vf", "scale=800:600", "-pix_fmt", "rgb24", "-f", "rawvideo", "pipe:1"],
                 stdout=subprocess.PIPE) for name in ["baseline", "enabled"]]
    encoder = subprocess.Popen(["ffmpeg", "-v", "error", "-y", "-f", "rawvideo", "-pixel_format", "rgb24",
                 "-video_size", f"{width}x{height}", "-framerate", str(fps), "-i", "pipe:0",
                 "-an", "-c:v", "libx264", "-threads", "2", "-preset", "fast", "-crf", "20", "-pix_fmt", "yuv420p",
                 "-movflags", "+faststart", str(out / "comparison.mp4")], stdin=subprocess.PIPE)
    histories = [series(run) for run in runs]
    charts = [(60, 815, 760, 922, "Arm-to-pillar gap [mm] (capped at 120)", 130.0, 0),
              (865, 815, 1545, 922, "Object position error [mm]", 20.0, 1)]
    try:
        for frame in range(round(duration * fps) + 1):
            chunks = [decoder.stdout.read(800 * 600 * 3) for decoder in decoders]
            if any(len(chunk) != 800 * 600 * 3 for chunk in chunks):
                raise RuntimeError(f"recording ended before frame {frame}; check ffmpeg input")
            time = frame / fps
            canvas = Image.new("RGB", (width, height), BG)
            draw = ImageDraw.Draw(canvas)
            draw.text((40, 10), "NULLSPACE  /  LIFT - ROLL - CROSS WALL - INSERT", font=large, fill=INK)
            draw.text((40, 48), "Same nominal waypoints + CBF + torque QP | rigid grasps | physical contacts ON", font=small, fill=INK2)
            draw.text((1370, 16), f"t = {time:05.2f} s", font=medium, fill=INK)
            for i, (chunk, run, color) in enumerate(zip(chunks, runs, VIDEO_COLORS)):
                x = i * 800
                canvas.paste(Image.frombytes("RGB", (800, 600), chunk), (x, 96))
                draw.rectangle((x, 76, x + 799, 104), fill=color)
                draw.text((x + 22, 77), "BASELINE / NULLSPACE OFF" if i == 0 else "AVOIDANCE / NULLSPACE ON",
                          font=small, fill="white")
                left = np.interp(time, run.t, run["d_left_arm~pillar_L"]) * 1000
                right = np.interp(time, run.t, run["d_right_arm~pillar_R"]) * 1000
                error = np.interp(time, run.t, histories[i][1])
                torque = np.interp(time, run.t, run["ns_torque_norm"])
                draw.text((x + 30, 699), f"L / R pillar gap: {format_gap(left)} / {format_gap(right)} mm", font=medium, fill=color)
                draw.text((x + 30, 730), f"Object error: {error:.2f} mm   |   NS torque: {torque:.2f} N m",
                          font=small, fill=INK)
                wall = np.interp(time, run.t, run["d_object~barrier"]) * 1000
                roll = np.degrees(np.interp(time, run.t, run["roll_obj"]))
                draw.text((x + 30, 756), f"Plate-to-wall: {format_gap(wall)} mm  |  Plate roll: {roll:.1f} deg",
                          font=small, fill=INK2)
            for x0, y0, x1, y1, title, maximum, index in charts:
                draw.text((x0, y0 - 34), title, font=small, fill=INK)
                for fraction in [0, .5, 1]:
                    y = y1 - fraction * (y1 - y0)
                    draw.line((x0, y, x1, y), fill=GRID)
                    draw.text((x0 - 44, y - 8), f"{maximum * fraction:g}", font=small, fill=INK2)
                for run, history, color in zip(runs, histories, VIDEO_COLORS):
                    ts = np.linspace(0, duration, 500)
                    values = np.interp(ts, run.t, history[index])
                    points = [(x0 + t/duration*(x1-x0), y1-v/maximum*(y1-y0)) for t, v in zip(ts, values)]
                    draw.line(points, fill=color, width=3)
                cursor = x0 + time/duration*(x1-x0)
                draw.line((cursor, y0, cursor, y1), fill=INK, width=2)
                draw.text((x1 - 40, y1 + 7), f"{duration:g}s", font=small, fill=INK2)
            encoder.stdin.write(canvas.tobytes())
        encoder.stdin.close()
        if encoder.wait() != 0:
            raise RuntimeError("comparison video encoding failed")
    finally:
        for decoder in decoders:
            if decoder.stdout: decoder.stdout.close()
            if decoder.poll() is None: decoder.terminate()
            decoder.wait()
        if encoder.poll() is None:
            encoder.terminate()
            encoder.wait()
    subprocess.run(["ffmpeg", "-v", "error", "-y", "-i", str(out / "comparison.mp4"),
                    "-filter_complex", "fps=8,scale=800:-1:flags=lanczos,split[a][b];"
                    "[a]palettegen=max_colors=128:stats_mode=diff[p];[b][p]paletteuse=dither=bayer:bayer_scale=3",
                    str(out / "comparison.gif")], check=True)


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--record", action="store_true", help="record MP4 and GIF (requires DISPLAY)")
    parser.add_argument("--media-dir", type=Path, help="copy comparison GIF, MP4 and chart to this directory")
    parser.add_argument("--out", type=Path, help="output directory")
    parser.add_argument("--duration", type=float, default=22)
    args = parser.parse_args()
    if not math.isfinite(args.duration) or args.duration < 22:
        parser.error("duration must be at least 22 s to complete the moving insertion task")
    if args.media_dir and not args.record:
        parser.error("--media-dir requires --record")
    out = (args.out or ROOT / "artifacts/nullspace_transport_demo" /
           datetime.now(ZoneInfo("Asia/Shanghai")).strftime("%Y%m%d_%H%M%S")).resolve()
    out.mkdir(parents=True, exist_ok=True)
    # Preserve the scene assets alongside effective per-run configurations.
    for source in ["config/scenes/slot_avoid_nullspace.yaml", "models/scene_slot_avoid_nullspace.xml"]:
        shutil.copy2(ROOT / source, out / Path(source).name)
    cases = {name: run_case(out, enabled, args.record, args.duration)
             for name, enabled in [("baseline", False), ("enabled", True)]}
    on, off = (cases[n]["metrics"]["clearance_insert_min_mm"] for n in ("enabled", "baseline"))
    comparison_checks = {f"{arm}_insert_gap_improvement_over_25_mm": on[arm] - off[arm] > 25 for arm in ("left", "right")}
    report = {"passed": all(case["passed"] for case in cases.values()) and all(comparison_checks.values()),
              "cases": cases, "comparison_checks": comparison_checks}
    (out / "results.json").write_text(json.dumps(report, indent=2, ensure_ascii=False) + "\n")
    lines = ["# Moving slot-avoid task: joint nullspace OFF / ON", "", f"Acceptance: {'PASS' if report['passed'] else 'FAIL'}", "",
             f"| Case | Insertion (t >= {INSERT_START_S:g} s) min L/R pillar gap [mm] | Final L/R gap [mm] | Max position error [mm] | Max orientation error [deg] | Max leak |",
             "|---|---:|---:|---:|---:|---:|"]
    for name, case in cases.items():
        m = case["metrics"]; gap = m["clearance_final_mm"]; ins = m["clearance_insert_min_mm"]
        lines.append(f"| {name} | {ins['left']:.2f} / {ins['right']:.2f} | {gap['left']:.2f} / {gap['right']:.2f} | "
                     f"{m['position_error_max_mm']:.3f} | {m['orientation_error_max_deg']:.4f} | "
                     f"{m['nullspace']['acceleration_leak_max']:.2e} |")
    (out / "summary.md").write_text("\n".join(lines) + "\n")
    if not report["passed"]:
        raise RuntimeError(f"acceptance failed; inspect {out / 'results.json'}")
    runs = [load_run(case["log"], name) for name, case in cases.items()]
    plot_comparison(out, runs)
    if args.record:
        print("[nullspace] composing synchronized comparison video", flush=True)
        compose_video(out, runs, args.duration)
    if args.media_dir:
        destination = args.media_dir.resolve()
        destination.mkdir(parents=True, exist_ok=True)
        for extension in ["gif", "mp4", "png"]:
            shutil.copy2(out / f"comparison.{extension}", destination / f"nullspace_transport_comparison.{extension}")
    print(f"[nullspace] PASS; artifacts: {out}", flush=True)


if __name__ == "__main__":
    main()
