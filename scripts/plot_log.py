#!/usr/bin/env python3
"""画 run_sim 生成的 CSV 日志，支持多次运行对比（例如 基线 vs 协同）。

用法：
    python3 scripts/plot_log.py                          # 画 logs/ 下最新的一次运行
    python3 scripts/plot_log.py --latest 2               # 最新两次运行画在同一组图上
    python3 scripts/plot_log.py logs/A logs/B --labels baseline coop
    python3 scripts/plot_log.py logs/A --show            # 弹窗显示（默认只保存 PNG）

输出（单次运行默认保存到 <run>/plots/，多次运行保存到 logs/compare_<...>/）：
    joint_torques.png    关节力矩（虚线为力矩上限）
    wrist_wrench.png     腕部 F/T 换算的夹爪→物体 wrench（世界系，TCP，已扣夹爪重力）
    weld_wrench.png      weld 约束 wrench（仿真真值，世界系，抓取点）
    box.png              物体位移 / 相对参考轨迹的位姿误差（灰色阴影 = 施加扰动的时间段）
    timing.png           单周期控制器计算耗时
    internal_force.png   【预留】内力 h_int（int_* 列，由 coop::internalWrenchForLogging 填入）

只依赖 numpy + matplotlib。
"""
from __future__ import annotations

import argparse
import os
import sys
from dataclasses import dataclass, field

import numpy as np

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
LOG_DIR = os.path.join(ROOT, "logs")

# 颜色：按固定顺序分配的分类色（x/y/z 分量，或不同运行），不循环、不按大小重排
SERIES = ["#2a78d6", "#eb6834", "#1baf7a"]  # 蓝、橙、青
RUN_STYLES = ["-", "--", ":"]               # 多次运行用线型区分（颜色留给分量）
TEXT = "#0b0b0b"
TEXT_2 = "#52514e"
GRID = "#e4e3df"
SURFACE = "#fcfcfb"
SHADE = "#d9d8d3"
TAU_LIMIT = np.array([87, 87, 87, 87, 12, 12, 12], dtype=float)

WRENCH = ["fx", "fy", "fz", "mx", "my", "mz"]
ARMS = [("l", "left"), ("r", "right")]


@dataclass
class Run:
    path: str                   # log.csv 路径
    label: str
    cols: dict = field(default_factory=dict)

    def __getitem__(self, key: str) -> np.ndarray:
        return self.cols[key]

    def has(self, key: str) -> bool:
        return key in self.cols

    @property
    def t(self) -> np.ndarray:
        return self.cols["t"]


# ---------------------------------------------------------------------------
# 读取
# ---------------------------------------------------------------------------
def resolve_csv(p: str) -> str:
    if os.path.isdir(p):
        p = os.path.join(p, "log.csv")
    if not os.path.isfile(p):
        sys.exit(f"找不到日志文件: {p}")
    return p


def latest_runs(n: int) -> list[str]:
    if not os.path.isdir(LOG_DIR):
        sys.exit(f"没有日志目录 {LOG_DIR}，先运行 ./build/run_sim")
    dirs = [os.path.join(LOG_DIR, d) for d in os.listdir(LOG_DIR)
            if os.path.isfile(os.path.join(LOG_DIR, d, "log.csv"))]
    if not dirs:
        sys.exit(f"{LOG_DIR} 下没有运行记录")
    dirs.sort(key=lambda d: os.path.getmtime(os.path.join(d, "log.csv")))
    return dirs[-n:]


def load_run(path: str, label: str | None = None) -> Run:
    csv = resolve_csv(path)
    with open(csv) as f:
        header = f.readline().strip().split(",")
    data = np.loadtxt(csv, delimiter=",", skiprows=1, ndmin=2)
    if data.shape[1] != len(header):
        sys.exit(f"{csv}: 列数 {data.shape[1]} 与表头 {len(header)} 不一致")
    run_dir = os.path.basename(os.path.dirname(os.path.abspath(csv)))
    return Run(path=csv, label=label or run_dir, cols={h: data[:, i] for i, h in enumerate(header)})


# ---------------------------------------------------------------------------
# 绘图工具
# ---------------------------------------------------------------------------
def setup_matplotlib(show: bool):
    import matplotlib
    if not show:
        matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    plt.rcParams.update({
        "figure.facecolor": SURFACE, "axes.facecolor": SURFACE, "savefig.facecolor": SURFACE,
        "axes.edgecolor": GRID, "axes.labelcolor": TEXT_2, "axes.titlecolor": TEXT,
        "axes.titlesize": 10, "axes.labelsize": 9, "xtick.color": TEXT_2, "ytick.color": TEXT_2,
        "xtick.labelsize": 8, "ytick.labelsize": 8, "axes.grid": True, "grid.color": GRID,
        "grid.linewidth": 0.6, "lines.linewidth": 1.4, "legend.fontsize": 8,
        "legend.frameon": False, "axes.spines.top": False, "axes.spines.right": False,
        "font.family": ["DejaVu Sans"],
        "axes.unicode_minus": False,
    })
    return plt


def shade_disturbance(ax, run: Run):
    """把施加扰动的时间段画成浅灰色背景"""
    if not run.has("dist_fx"):
        return
    mag = np.zeros_like(run.t)
    for k in WRENCH:
        mag += np.abs(run[f"dist_{k}"])
    active = mag > 1e-9
    if not active.any():
        return
    edges = np.flatnonzero(np.diff(active.astype(int)))
    starts = list(run.t[edges[active[edges + 1]] + 1]) if len(edges) else []
    ends = list(run.t[edges[~active[edges + 1]]]) if len(edges) else []
    if active[0]:
        starts.insert(0, run.t[0])
    if active[-1]:
        ends.append(run.t[-1])
    for s, e in zip(starts, ends):
        ax.axvspan(s, e, color=SHADE, alpha=0.5, lw=0, zorder=0)


def run_legend(fig, runs: list[Run], extra: list | None = None):
    """图例：分量颜色 + 运行线型"""
    from matplotlib.lines import Line2D
    handles = list(extra or [])
    if len(runs) > 1:
        handles += [Line2D([], [], color=TEXT_2, ls=RUN_STYLES[i % 3], label=r.label)
                    for i, r in enumerate(runs)]
    if handles:
        # 图例单独占一行，位于总标题下方
        fig.legend(handles=handles, loc="upper center", ncol=len(handles), bbox_to_anchor=(0.5, 0.965))


def component_handles(names):
    from matplotlib.lines import Line2D
    return [Line2D([], [], color=SERIES[i], label=n) for i, n in enumerate(names)]


def save(fig, out_dir: str, name: str, show: bool):
    path = os.path.join(out_dir, name)
    fig.savefig(path, dpi=130, bbox_inches="tight")
    print(f"  saved {path}")
    if not show:
        import matplotlib.pyplot as plt
        plt.close(fig)


# ---------------------------------------------------------------------------
# 各类图
# ---------------------------------------------------------------------------
def plot_joint_torques(plt, runs: list[Run], out_dir: str, show: bool):
    fig, axes = plt.subplots(2, 7, figsize=(18, 5.2), sharex=True)
    for row, (p, name) in enumerate(ARMS):
        for j in range(7):
            ax = axes[row, j]
            for i, r in enumerate(runs):
                ax.plot(r.t, r[f"{p}_tau{j + 1}"], color=SERIES[i % 3], ls="-", label=r.label)
            for s in (-1, 1):
                ax.axhline(s * TAU_LIMIT[j], color=TEXT_2, ls="--", lw=0.8)
            ax.set_title(f"{name} joint {j + 1}")
            if j == 0:
                ax.set_ylabel("τ [N·m]")
            if row == 1:
                ax.set_xlabel("t [s]")
            # 纵轴只放大到数据范围，限位线超出时不强行显示
            lo = min(np.nanmin(r[f"{p}_tau{j + 1}"]) for r in runs)
            hi = max(np.nanmax(r[f"{p}_tau{j + 1}"]) for r in runs)
            pad = 0.1 * max(hi - lo, 1.0)
            ax.set_ylim(max(lo - pad, -1.05 * TAU_LIMIT[j]), min(hi + pad, 1.05 * TAU_LIMIT[j]))
    fig.suptitle("Joint torques (dashed = torque limit)", color=TEXT, y=0.995)
    if len(runs) > 1:
        from matplotlib.lines import Line2D
        fig.legend(handles=[Line2D([], [], color=SERIES[i % 3], label=r.label) for i, r in enumerate(runs)],
                   loc="upper center", ncol=len(runs), bbox_to_anchor=(0.5, 0.955))
        fig.tight_layout(rect=(0, 0, 1, 0.9))
    else:
        fig.tight_layout(rect=(0, 0, 1, 0.95))
    save(fig, out_dir, "joint_torques.png", show)


def plot_wrench(plt, runs: list[Run], out_dir: str, show: bool, source: str):
    """source = 'ftw'（腕部 F/T 换算）或 'weld'（weld 约束真值）"""
    titles = {"ftw": "Wrist F/T → wrench applied by gripper on object (world, @TCP, hand gravity removed)",
              "weld": "Weld constraint wrench applied by hand on box (world, @grasp point) — ground truth"}
    fig, axes = plt.subplots(2, 2, figsize=(13, 6.5), sharex=True)
    for col, (p, name) in enumerate(ARMS):
        for row, (keys, unit) in enumerate([(WRENCH[:3], "force [N]"), (WRENCH[3:], "torque [N·m]")]):
            ax = axes[row, col]
            shade_disturbance(ax, runs[0])
            for i, r in enumerate(runs):
                for c, k in enumerate(keys):
                    ax.plot(r.t, r[f"{p}_{source}_{k}"], color=SERIES[c], ls=RUN_STYLES[i % 3])
            ax.set_title(f"{name} arm")
            ax.set_ylabel(unit)
            if row == 1:
                ax.set_xlabel("t [s]")
    fig.suptitle(titles[source], color=TEXT, y=0.995)
    run_legend(fig, runs, component_handles(["x", "y", "z"]))
    fig.tight_layout(rect=(0, 0, 1, 0.92))
    save(fig, out_dir, "wrist_wrench.png" if source == "ftw" else "weld_wrench.png", show)


def plot_box(plt, runs: list[Run], out_dir: str, show: bool):
    fig, axes = plt.subplots(3, 1, figsize=(11, 8), sharex=True)
    for ax in axes:
        shade_disturbance(ax, runs[0])
    for i, r in enumerate(runs):
        ls = RUN_STYLES[i % 3]
        for c, k in enumerate("xyz"):
            axes[0].plot(r.t, 1e3 * (r[f"obj_{k}"] - r[f"obj_{k}"][0]), color=SERIES[c], ls=ls)
            axes[1].plot(r.t, 1e3 * r[f"obj_err_{k}"], color=SERIES[c], ls=ls)
            axes[2].plot(r.t, np.degrees(r[f"obj_err_r{k}"]), color=SERIES[c], ls=ls)
    axes[0].set_ylabel("object position − initial [mm]")
    axes[0].set_title("Object position (grey = disturbance active)")
    axes[1].set_ylabel("p_ref − p [mm]")
    axes[1].set_title("Object position error w.r.t. reference trajectory")
    axes[2].set_ylabel("log(R_ref Rᵀ) [deg]")
    axes[2].set_title("Object orientation error (rotation vector, world frame)")
    axes[2].set_xlabel("t [s]")
    fig.suptitle("Object trajectory", color=TEXT, y=0.995)
    run_legend(fig, runs, component_handles(["x", "y", "z"]))
    fig.tight_layout(rect=(0, 0, 1, 0.94))
    save(fig, out_dir, "box.png", show)


def plot_timing(plt, runs: list[Run], out_dir: str, show: bool):
    fig, (ax0, ax1) = plt.subplots(1, 2, figsize=(13, 4), gridspec_kw={"width_ratios": [2, 1]})
    hi = max(np.percentile(r["t_ctrl_us"], 99.9) for r in runs)
    bins = np.linspace(0, max(hi, 1e-3), 60)
    for i, r in enumerate(runs):
        c = SERIES[i % 3]
        x = r["t_ctrl_us"]
        ax0.plot(r.t, x, color=c, lw=0.6, label=r.label)
        ax1.hist(np.clip(x, 0, bins[-1]), bins=bins, color=c, alpha=0.55, label=r.label)
        print(f"  [{r.label}] controller time: mean {x.mean():.1f} us, p99 {np.percentile(x, 99):.1f} us, "
              f"max {x.max():.1f} us | sim step mean {r['t_step_us'].mean():.1f} us")
    ax0.axhline(1000.0, color=TEXT_2, ls="--", lw=0.8)
    ax0.set_ylim(0, bins[-1] * 1.1)
    ax0.set_xlabel("t [s]")
    ax0.set_ylabel("controller compute time [µs]")
    ax0.set_title("Per-cycle controller compute time (1 kHz budget = 1000 µs)")
    ax1.set_xlabel("controller compute time [µs]")
    ax1.set_ylabel("cycles")
    ax1.set_title("Distribution (clipped at p99.9)")
    if len(runs) > 1:
        ax0.legend(loc="upper right")
    fig.tight_layout()
    save(fig, out_dir, "timing.png", show)


def plot_internal_force(plt, runs: list[Run], out_dir: str, show: bool):
    """【预留】画内力 h_int（int_{l,r}_{fx..mz} 列）。

    这些列由 coop::internalWrenchForLogging() 填入；未实现时全为 NaN，此时跳过。
    实现后建议关注：‖f_int‖ 的峰值与稳态值（基线 vs 协同），以及沿两抓取点连线方向（x）的
    分量——负值表示挤压、正值表示拉伸（取决于你的符号约定，见 docs/cooperative_control.md §4）。
    """
    valid = [r for r in runs if r.has("int_l_fx") and np.isfinite(r["int_l_fx"]).any()]
    if not valid:
        print("  internal force: int_* columns are NaN (coop::internalWrenchForLogging not implemented) - skipped")
        return
    fig, axes = plt.subplots(2, 2, figsize=(13, 6.5), sharex=True)
    for col, (p, name) in enumerate(ARMS):
        ax_f, ax_n = axes[0, col], axes[1, col]
        shade_disturbance(ax_f, valid[0])
        shade_disturbance(ax_n, valid[0])
        for i, r in enumerate(valid):
            for c, k in enumerate(WRENCH[:3]):
                ax_f.plot(r.t, r[f"int_{p}_{k}"], color=SERIES[c], ls=RUN_STYLES[i % 3])
            fn = np.sqrt(sum(r[f"int_{p}_{k}"] ** 2 for k in WRENCH[:3]))
            ax_n.plot(r.t, fn, color=TEXT_2, ls=RUN_STYLES[i % 3], label=r.label)
            print(f"  [{r.label}] {name}: max |f_int| = {np.nanmax(fn):.2f} N, "
                  f"final |f_int| = {fn[-1]:.2f} N")
        ax_f.set_title(f"{name} arm: internal force components")
        ax_f.set_ylabel("f_int [N]")
        ax_n.set_title(f"{name} arm: |f_int|")
        ax_n.set_ylabel("|f_int| [N]")
        ax_n.set_xlabel("t [s]")
    fig.suptitle("Internal wrench h_int = (I − G⁺G) h (projection onto null space of G)", color=TEXT, y=0.995)
    run_legend(fig, valid, component_handles(["x", "y", "z"]))
    fig.tight_layout(rect=(0, 0, 1, 0.92))
    save(fig, out_dir, "internal_force.png", show)


def print_summary(runs: list[Run]):
    for r in runs:
        drift = np.sqrt(sum((r[f"obj_{k}"] - r[f"obj_{k}"][0]) ** 2 for k in "xyz"))
        err = np.sqrt(sum(r[f"obj_err_{k}"] ** 2 for k in "xyz"))
        weld = [np.sqrt(sum(r[f"{p}_weld_{k}"] ** 2 for k in WRENCH[:3])) for p, _ in ARMS]
        print(f"  [{r.label}] {len(r.t)} rows, t = {r.t[0]:.3f}..{r.t[-1]:.3f} s | "
              f"max object drift {1e3 * drift.max():.2f} mm | max |p_err| {1e3 * err.max():.2f} mm | "
              f"max |f_weld| L {weld[0].max():.2f} N, R {weld[1].max():.2f} N")


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("runs", nargs="*", help="运行目录或 log.csv（默认：最新一次）")
    ap.add_argument("--latest", type=int, default=0, help="使用 logs/ 下最新的 N 次运行")
    ap.add_argument("--labels", nargs="*", help="每次运行的图例名称")
    ap.add_argument("--out", help="输出目录")
    ap.add_argument("--tmax", type=float, help="只画 t <= tmax 的部分")
    ap.add_argument("--show", action="store_true", help="弹窗显示")
    args = ap.parse_args()

    paths = args.runs or latest_runs(args.latest or 1)
    if args.latest and args.runs:
        sys.exit("不要同时给出运行路径和 --latest")
    if len(paths) > 3:
        sys.exit("最多同时对比 3 次运行（线型只有 3 种）")
    labels = args.labels or [None] * len(paths)
    if len(labels) != len(paths):
        sys.exit("--labels 的个数必须与运行个数相同")
    runs = [load_run(p, l) for p, l in zip(paths, labels)]
    if args.tmax is not None:
        for r in runs:
            keep = r.t <= args.tmax
            r.cols = {k: v[keep] for k, v in r.cols.items()}

    if args.out:
        out_dir = args.out
    elif len(runs) == 1:
        out_dir = os.path.join(os.path.dirname(runs[0].path), "plots")
    else:
        names = "__".join(os.path.basename(os.path.dirname(r.path)) for r in runs)
        out_dir = os.path.join(LOG_DIR, f"compare_{names}")
    os.makedirs(out_dir, exist_ok=True)

    plt = setup_matplotlib(args.show)
    print_summary(runs)
    plot_joint_torques(plt, runs, out_dir, args.show)
    plot_wrench(plt, runs, out_dir, args.show, "ftw")
    plot_wrench(plt, runs, out_dir, args.show, "weld")
    plot_box(plt, runs, out_dir, args.show)
    plot_timing(plt, runs, out_dir, args.show)
    plot_internal_force(plt, runs, out_dir, args.show)
    if args.show:
        plt.show()


if __name__ == "__main__":
    main()
