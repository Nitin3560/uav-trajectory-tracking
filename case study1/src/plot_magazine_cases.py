from __future__ import annotations

import argparse
from pathlib import Path
from typing import Iterable

import matplotlib.pyplot as plt
import numpy as np
import pandas as pd


CONTROLLER_ORDER = ["openloop", "pid", "agentic"]
CONTROLLER_COLORS = {"openloop": "#4C78A8", "pid": "#F58518", "agentic": "#54A24B"}


def parse_args() -> argparse.Namespace:
    ap = argparse.ArgumentParser(description="Generate magazine-style plots for Case Study 1 and 3.")
    ap.add_argument("--case1-dir", default="outputs/case_study_1")
    ap.add_argument("--case3-dir", default="outputs/case_study_3")
    ap.add_argument("--out-dir", default="outputs/magazine_figs")
    return ap.parse_args()


def _load_case_csv(case_dir: Path) -> pd.DataFrame:
    csv_dir = case_dir / "csv"
    files = sorted(csv_dir.glob("*_seed*.csv"))
    rows = []
    for p in files:
        stem = p.stem
        ctrl, seed_part = stem.split("_seed")
        seed = int(seed_part)
        df = pd.read_csv(p)
        df["controller"] = ctrl
        df["seed"] = seed
        rows.append(df)
    if not rows:
        raise FileNotFoundError(f"No per-seed CSV files found in {csv_dir}")
    return pd.concat(rows, ignore_index=True)


def _mean_ci(df: pd.DataFrame, metric: str) -> pd.DataFrame:
    g = df.groupby(["controller", "t"])[metric]
    out = g.agg(["mean", "std", "count"]).reset_index()
    out["ci95"] = 1.96 * out["std"] / np.sqrt(out["count"].clip(lower=1))
    out["ci95"] = out["ci95"].fillna(0.0)
    return out


def _savefig(path: Path) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    plt.tight_layout()
    plt.savefig(path, dpi=320, bbox_inches="tight")
    plt.close()


def _plot_timeseries_compare(
    df: pd.DataFrame,
    metric: str,
    ylabel: str,
    title: str,
    outpath: Path,
    *,
    shade: tuple[float, float] | None = None,
) -> None:
    data = _mean_ci(df, metric)
    plt.figure(figsize=(8.5, 4.6))
    for ctrl in CONTROLLER_ORDER:
        d = data[data["controller"] == ctrl]
        if d.empty:
            continue
        t = d["t"].to_numpy()
        y = d["mean"].to_numpy()
        ci = d["ci95"].to_numpy()
        plt.plot(t, y, label=ctrl, color=CONTROLLER_COLORS[ctrl], linewidth=2.0)
        plt.fill_between(t, y - ci, y + ci, color=CONTROLLER_COLORS[ctrl], alpha=0.16)
    if shade is not None:
        plt.axvspan(shade[0], shade[1], color="gray", alpha=0.13)
        plt.axvline(shade[0], color="black", linestyle="--", linewidth=1.0)
    plt.xlabel("Time (s)")
    plt.ylabel(ylabel)
    plt.title(title)
    plt.legend(frameon=False, ncol=3, loc="best")
    _savefig(outpath)


def _plot_grouped_bar(
    df: pd.DataFrame,
    x_col: str,
    y_col: str,
    title: str,
    ylabel: str,
    outpath: Path,
) -> None:
    if df.empty or y_col not in df.columns:
        return
    piv = df.pivot(index=x_col, columns="controller", values=y_col)
    piv = piv[[c for c in CONTROLLER_ORDER if c in piv.columns]]
    if piv.empty:
        return
    x = np.arange(len(piv.index))
    width = 0.24
    plt.figure(figsize=(8.5, 4.6))
    for i, ctrl in enumerate(piv.columns):
        plt.bar(x + (i - 1) * width, piv[ctrl].to_numpy(), width=width, color=CONTROLLER_COLORS[ctrl], label=ctrl)
    plt.xticks(x, [str(v) for v in piv.index], rotation=0)
    plt.ylabel(ylabel)
    plt.title(title)
    plt.legend(frameon=False, ncol=3)
    _savefig(outpath)


def _run_level(df: pd.DataFrame, metrics: Iterable[str]) -> pd.DataFrame:
    cols = [m for m in metrics if m in df.columns]
    g = df.groupby(["controller", "seed"], as_index=False)[cols].mean()
    return g


def _boxplot_runs(df_run: pd.DataFrame, metric: str, ylabel: str, title: str, outpath: Path) -> None:
    if metric not in df_run.columns:
        return
    vals = [df_run[df_run["controller"] == c][metric].to_numpy() for c in CONTROLLER_ORDER]
    if all(len(v) == 0 for v in vals):
        return
    plt.figure(figsize=(7.2, 4.6))
    bp = plt.boxplot(vals, labels=CONTROLLER_ORDER, patch_artist=True, showfliers=False)
    for patch, c in zip(bp["boxes"], CONTROLLER_ORDER):
        patch.set_facecolor(CONTROLLER_COLORS[c])
        patch.set_alpha(0.35)
    for i, c in enumerate(CONTROLLER_ORDER):
        pts = df_run[df_run["controller"] == c][metric].to_numpy()
        if len(pts):
            jitter = np.random.default_rng(0).normal(loc=0.0, scale=0.04, size=len(pts))
            plt.scatter(np.full(len(pts), i + 1) + jitter, pts, color=CONTROLLER_COLORS[c], s=20, alpha=0.7)
    plt.ylabel(ylabel)
    plt.title(title)
    _savefig(outpath)


def _scatter_tradeoff(df_run: pd.DataFrame, x: str, y: str, title: str, outpath: Path, xlabel: str, ylabel: str) -> None:
    if x not in df_run.columns or y not in df_run.columns:
        return
    plt.figure(figsize=(6.8, 5.0))
    for c in CONTROLLER_ORDER:
        d = df_run[df_run["controller"] == c]
        if d.empty:
            continue
        plt.scatter(d[x], d[y], color=CONTROLLER_COLORS[c], label=c, s=35, alpha=0.8)
    plt.xlabel(xlabel)
    plt.ylabel(ylabel)
    plt.title(title)
    plt.legend(frameon=False)
    _savefig(outpath)


def _generate_case1(case_dir: Path, out_dir: Path) -> None:
    df = _load_case_csv(case_dir)
    case_out = out_dir / "case_study_1"
    case_out.mkdir(parents=True, exist_ok=True)

    # Systematic time-series comparison plots.
    metric_specs = [
        ("mean_err_m", "Mean Tracking Error (m)"),
        ("max_err_m", "Max Tracking Error (m)"),
        ("formation_err_rel", "Formation Error (m)"),
        ("connectivity_rate", "Connectivity Rate"),
        ("throughput_ratio", "Throughput Ratio"),
        ("latency_proxy", "Latency Proxy"),
        ("control_overhead", "Control Overhead"),
        ("task_completion_proxy", "Task Completion Proxy"),
        ("system_utility", "System Utility"),
    ]
    for metric, ylabel in metric_specs:
        if metric not in df.columns:
            continue
        _plot_timeseries_compare(
            df=df,
            metric=metric,
            ylabel=ylabel,
            title=f"Case Study 1: {ylabel} (Mean ± 95% CI)",
            outpath=case_out / f"ts_{metric}.png",
        )

    # Window/bar plots using summary tables.
    windows_p = case_dir / "summary_windows.csv"
    bins_p = case_dir / "summary_bins.csv"
    if windows_p.exists():
        win = pd.read_csv(windows_p)
        for metric, ylabel in metric_specs[:6]:
            if metric in win.columns:
                _plot_grouped_bar(
                    win,
                    x_col="wind_phase",
                    y_col=metric,
                    ylabel=ylabel,
                    title=f"Case Study 1: Wind-Phase {ylabel}",
                    outpath=case_out / f"bar_phase_{metric}.png",
                )
    if bins_p.exists():
        bins = pd.read_csv(bins_p)
        bins["time_bin"] = bins["time_bin"].astype(str)
        for metric, ylabel in metric_specs[:6]:
            if metric in bins.columns:
                _plot_grouped_bar(
                    bins,
                    x_col="time_bin",
                    y_col=metric,
                    ylabel=ylabel,
                    title=f"Case Study 1: Time-Bin {ylabel}",
                    outpath=case_out / f"bar_bin_{metric}.png",
                )

    # Seed-level distribution plots.
    run = _run_level(df, [m for m, _ in metric_specs])
    for metric, ylabel in metric_specs:
        if metric in run.columns:
            _boxplot_runs(
                run,
                metric=metric,
                ylabel=ylabel,
                title=f"Case Study 1: Seed Distribution of {ylabel}",
                outpath=case_out / f"box_{metric}.png",
            )

    # Trade-off scatter plots.
    _scatter_tradeoff(
        run, x="mean_err_m", y="connectivity_rate",
        title="Case Study 1 Trade-off: Tracking vs Connectivity",
        xlabel="Mean Tracking Error (m)", ylabel="Connectivity Rate",
        outpath=case_out / "scatter_err_vs_conn.png",
    )
    _scatter_tradeoff(
        run, x="mean_err_m", y="throughput_ratio",
        title="Case Study 1 Trade-off: Tracking vs Throughput",
        xlabel="Mean Tracking Error (m)", ylabel="Throughput Ratio",
        outpath=case_out / "scatter_err_vs_tp.png",
    )
    _scatter_tradeoff(
        run, x="latency_proxy", y="throughput_ratio",
        title="Case Study 1 Trade-off: Latency vs Throughput",
        xlabel="Latency Proxy", ylabel="Throughput Ratio",
        outpath=case_out / "scatter_latency_vs_tp.png",
    )


def _generate_case3(case_dir: Path, out_dir: Path) -> None:
    df = _load_case_csv(case_dir)
    case_out = out_dir / "case_study_3"
    case_out.mkdir(parents=True, exist_ok=True)

    fault_summary_p = case_dir / "summary_faults.csv"
    fault_win_p = case_dir / "summary_fault_windows.csv"
    fault_degr_p = case_dir / "summary_fault_degradation.csv"

    # Fault window shading.
    t0 = 30.0
    t1 = 45.0
    if fault_summary_p.exists():
        fs = pd.read_csv(fault_summary_p)
        if "fault_t0_s" in fs.columns and not fs["fault_t0_s"].isna().all():
            t0 = float(fs["fault_t0_s"].dropna().iloc[0])

    core_metrics = [
        ("mean_err_m", "Mean Tracking Error (m)"),
        ("max_err_m", "Max Tracking Error (m)"),
        ("connectivity_rate", "Connectivity Rate"),
        ("formation_err_rel", "Formation Error (m)"),
        ("throughput_ratio", "Throughput Ratio"),
        ("latency_proxy", "Latency Proxy"),
        ("control_overhead", "Control Overhead"),
        ("task_completion_proxy", "Task Completion Proxy"),
        ("system_utility", "System Utility"),
    ]
    for metric, ylabel in core_metrics:
        if metric not in df.columns:
            continue
        _plot_timeseries_compare(
            df=df,
            metric=metric,
            ylabel=ylabel,
            title=f"Case Study 3: {ylabel} Under Comm Fault (Mean ± 95% CI)",
            outpath=case_out / f"ts_fault_{metric}.png",
            shade=(t0, t1),
        )

    # Fault-window grouped bars.
    if fault_win_p.exists():
        fw = pd.read_csv(fault_win_p)
        fw["fault_window"] = fw["fault_window"].astype(str)
        for metric, ylabel in core_metrics:
            if metric in fw.columns:
                _plot_grouped_bar(
                    fw, x_col="fault_window", y_col=metric, ylabel=ylabel,
                    title=f"Case Study 3: Pre/Fault/Post {ylabel}",
                    outpath=case_out / f"bar_faultwindow_{metric}.png",
                )

    # Degradation bars from summary_fault_degradation.
    if fault_degr_p.exists():
        fd = pd.read_csv(fault_degr_p)
        degr_specs = [
            ("err_post_vs_pre_pct", "Tracking Degradation (%)"),
            ("conn_post_vs_pre_pct", "Connectivity Change (%)"),
            ("task_post_vs_pre_pct", "Task Completion Change (%)"),
            ("utility_post_abs_degradation_pct", "Utility Abs Degradation (%)"),
            ("recovery_time_s", "Recovery Time (s)"),
            ("max_err_fault", "Peak Fault Error (m)"),
        ]
        for col, ylabel in degr_specs:
            if col not in fd.columns:
                continue
            d = fd[["controller", col]].copy()
            plt.figure(figsize=(6.8, 4.6))
            xs = np.arange(len(CONTROLLER_ORDER))
            ys = [float(d[d["controller"] == c][col].iloc[0]) if not d[d["controller"] == c].empty else np.nan for c in CONTROLLER_ORDER]
            bars = plt.bar(xs, ys, color=[CONTROLLER_COLORS[c] for c in CONTROLLER_ORDER], alpha=0.85)
            plt.xticks(xs, CONTROLLER_ORDER)
            plt.ylabel(ylabel)
            plt.title(f"Case Study 3: {ylabel}")
            for b, y in zip(bars, ys):
                if np.isfinite(y):
                    plt.text(b.get_x() + b.get_width() / 2.0, y, f"{y:.2f}", ha="center", va="bottom", fontsize=8)
            _savefig(case_out / f"bar_degradation_{col}.png")

    # Seed-distribution by phase for fault windows from raw time-series.
    if "t" in df.columns and "mean_err_m" in df.columns:
        d = df.copy()
        d["fault_window"] = pd.cut(
            d["t"],
            bins=[t0 - 5.0, t0, t0 + 5.0, t0 + 15.0],
            labels=["pre", "fault", "post"],
            right=False,
            include_lowest=True,
        )
        d = d[d["fault_window"].notna()]
        run = d.groupby(["controller", "seed", "fault_window"], as_index=False).agg(
            mean_err_m=("mean_err_m", "mean"),
            connectivity_rate=("connectivity_rate", "mean"),
            system_utility=("system_utility", "mean"),
        )
        for metric, ylabel in [("mean_err_m", "Mean Tracking Error (m)"), ("connectivity_rate", "Connectivity Rate"), ("system_utility", "System Utility")]:
            if metric not in run.columns:
                continue
            for win in ["pre", "fault", "post"]:
                sub = run[run["fault_window"].astype(str) == win]
                if sub.empty:
                    continue
                _boxplot_runs(
                    sub[["controller", "seed", metric]].copy(),
                    metric=metric,
                    ylabel=ylabel,
                    title=f"Case Study 3: {ylabel} Seed Distribution ({win})",
                    outpath=case_out / f"box_{metric}_{win}.png",
                )


def main() -> None:
    args = parse_args()
    case1_dir = Path(args.case1_dir)
    case3_dir = Path(args.case3_dir)
    out_dir = Path(args.out_dir)

    _generate_case1(case1_dir, out_dir)
    _generate_case3(case3_dir, out_dir)

    print(f"Saved magazine figures in: {out_dir.resolve()}")


if __name__ == "__main__":
    main()

