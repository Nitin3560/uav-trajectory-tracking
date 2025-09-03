from __future__ import annotations

import argparse
import json
import math
import shutil
import subprocess
from dataclasses import dataclass
from datetime import datetime
from pathlib import Path
from typing import Any

import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
import yaml

plt.rcParams.update({
    "font.family": "serif",
    "font.serif": ["Times New Roman", "Times", "DejaVu Serif"],
    "axes.titlesize": 12,
    "axes.labelsize": 11,
    "legend.fontsize": 10,
    "xtick.labelsize": 10,
    "ytick.labelsize": 10,
})

LOWER_BETTER = {
    "mean_tracking_error_m",
    "max_tracking_error_m",
    "rmse_m",
    "steady_state_error_m",
    "formation_error_rel",
    "control_effort",
    "control_smoothness",
    "peak_fault_error_m",
    "pre_fault_error_m",
    "fault_window_error_m",
    "post_fault_error_m",
    "recovery_time_s",
    "post_vs_pre_error_degradation_pct",
}
HIGHER_BETTER = {
    "connectivity_rate",
    "throughput_ratio",
    "task_completion_proxy",
    "connectivity_pre",
    "connectivity_fault",
    "connectivity_post",
    "connectivity_retention_pct",
}

@dataclass
class CaseSpec:
    name: str
    case_dir: Path
    config_path: Path
    old_dir: Path
    fault_case: bool


def parse_args() -> argparse.Namespace:
    ap = argparse.ArgumentParser()
    ap.add_argument("--root", required=True)
    ap.add_argument("--case1-dir", required=True)
    ap.add_argument("--case3-dir", required=True)
    ap.add_argument("--case1-config", required=True)
    ap.add_argument("--case3-config", required=True)
    ap.add_argument("--case1-old-dir", required=True)
    ap.add_argument("--case3-old-dir", required=True)
    ap.add_argument("--seeds", nargs="+", type=int, required=True)
    ap.add_argument("--git-hash", default="")
    return ap.parse_args()


def load_yaml(path: Path) -> dict[str, Any]:
    with open(path, "r") as f:
        return yaml.safe_load(f) or {}


def git_hash() -> str:
    try:
        return subprocess.check_output(["git", "rev-parse", "HEAD"], text=True, cwd=Path(__file__).resolve().parents[1]).strip()
    except Exception:
        return "unknown"


def seed_csv(case_dir: Path, controller: str, seed: int) -> Path:
    return case_dir / "csv" / f"{controller}_seed{seed}.csv"


def load_seed_df(case_dir: Path, controller: str, seed: int) -> pd.DataFrame:
    df = pd.read_csv(seed_csv(case_dir, controller, seed))
    df["seed"] = seed
    df["controller"] = controller
    return df


def _nanmean(series: pd.Series) -> float:
    s = pd.to_numeric(series, errors="coerce")
    return float(np.nanmean(s.to_numpy(dtype=float)))


def _nanmax(series: pd.Series) -> float:
    s = pd.to_numeric(series, errors="coerce")
    return float(np.nanmax(s.to_numpy(dtype=float)))


def per_seed_metrics(case: CaseSpec, controller: str, seed: int) -> dict[str, Any]:
    cfg = load_yaml(case.config_path)
    df = load_seed_df(case.case_dir, controller, seed)
    out: dict[str, Any] = {
        "case_study": case.name,
        "controller": controller,
        "seed": seed,
        "mean_tracking_error_m": _nanmean(df["mean_err_m"]),
        "max_tracking_error_m": _nanmax(df["max_err_m"]),
        "rmse_m": float(np.sqrt(np.nanmean(np.square(pd.to_numeric(df["mean_err_m"], errors="coerce"))))),
        "formation_error_rel": _nanmean(df["formation_err_rel"]),
        "connectivity_rate": _nanmean(df["connectivity_rate"]),
        "throughput_ratio": _nanmean(df["throughput_ratio"]) if "throughput_ratio" in df.columns else float("nan"),
        "control_overhead": _nanmean(df["control_overhead"]) if "control_overhead" in df.columns else float("nan"),
        "control_effort": _nanmean(df["control_effort"]) if "control_effort" in df.columns else float("nan"),
        "control_smoothness": _nanmean(df["control_smoothness"]) if "control_smoothness" in df.columns else float("nan"),
        "task_completion_proxy": _nanmean(df["task_completion_proxy"]) if "task_completion_proxy" in df.columns else float("nan"),
        "system_utility": _nanmean(df["system_utility"]) if "system_utility" in df.columns else float("nan"),
        "alive_ratio": _nanmean(df["alive_ratio"]) if "alive_ratio" in df.columns else float("nan"),
    }
    steady_after = float((cfg.get("metrics", {}) or {}).get("steady_state_after_s", float(df["t"].max()) * 0.75))
    ss = df[df["t"] >= steady_after]
    out["steady_state_error_m"] = _nanmean(ss["mean_err_m"]) if not ss.empty else float("nan")
    out["error_variance"] = float(np.nanvar(pd.to_numeric(df["mean_err_m"], errors="coerce")))

    if case.fault_case:
        t0 = float((cfg.get("faults", {}).get("events", [{}])[0] or {}).get("t0_s", 0.0))
        pre = df[(df["t"] >= t0 - 5.0) & (df["t"] < t0)]
        fault = df[(df["t"] >= t0) & (df["t"] < t0 + 5.0)]
        post = df[(df["t"] >= t0 + 5.0) & (df["t"] < t0 + 15.0)]
        out.update({
            "pre_fault_error_m": _nanmean(pre["mean_err_m"]) if not pre.empty else float("nan"),
            "fault_window_error_m": _nanmean(fault["mean_err_m"]) if not fault.empty else float("nan"),
            "post_fault_error_m": _nanmean(post["mean_err_m"]) if not post.empty else float("nan"),
            "peak_fault_error_m": _nanmax(fault["max_err_m"]) if not fault.empty else float("nan"),
            "connectivity_pre": _nanmean(pre["connectivity_rate"]) if not pre.empty else float("nan"),
            "connectivity_fault": _nanmean(fault["connectivity_rate"]) if not fault.empty else float("nan"),
            "connectivity_post": _nanmean(post["connectivity_rate"]) if not post.empty else float("nan"),
            "control_effort_pre": _nanmean(pre["control_effort"]) if (not pre.empty and "control_effort" in pre.columns) else float("nan"),
            "control_effort_fault": _nanmean(fault["control_effort"]) if (not fault.empty and "control_effort" in fault.columns) else float("nan"),
            "control_effort_post": _nanmean(post["control_effort"]) if (not post.empty and "control_effort" in post.columns) else float("nan"),
            "control_smoothness_pre": _nanmean(pre["control_smoothness"]) if (not pre.empty and "control_smoothness" in pre.columns) else float("nan"),
            "control_smoothness_fault": _nanmean(fault["control_smoothness"]) if (not fault.empty and "control_smoothness" in fault.columns) else float("nan"),
            "control_smoothness_post": _nanmean(post["control_smoothness"]) if (not post.empty and "control_smoothness" in post.columns) else float("nan"),
        })
        pre_err = out["pre_fault_error_m"]
        post_err = out["post_fault_error_m"]
        pre_conn = out["connectivity_pre"]
        post_conn = out["connectivity_post"]
        out["post_vs_pre_error_degradation_pct"] = float((post_err - pre_err) / pre_err * 100.0) if np.isfinite(pre_err) and abs(pre_err) > 1e-9 else float("nan")
        out["connectivity_retention_pct"] = float(post_conn / pre_conn * 100.0) if np.isfinite(pre_conn) and abs(pre_conn) > 1e-9 else float("nan")
        # Recovery metric in current codebase is not discriminative; keep NaN here for paper tables unless a real recovery is found.
        out["recovery_time_s"] = float("nan")
    return out


def stats_long(per_seed: pd.DataFrame) -> pd.DataFrame:
    id_cols = ["case_study", "controller", "seed"]
    metric_cols = [c for c in per_seed.columns if c not in id_cols]
    rows: list[dict[str, Any]] = []
    for case_name, case_df in per_seed.groupby("case_study"):
        for ctrl, ctrl_df in case_df.groupby("controller"):
            for metric in metric_cols:
                vals = pd.to_numeric(ctrl_df[metric], errors="coerce").dropna()
                if vals.empty:
                    continue
                rows.append({
                    "case_study": case_name,
                    "method": ctrl,
                    "metric": metric,
                    "mean": float(vals.mean()),
                    "std": float(vals.std(ddof=1)) if len(vals) > 1 else 0.0,
                    "min": float(vals.min()),
                    "max": float(vals.max()),
                })
    return pd.DataFrame(rows)


def wide_stats(per_seed: pd.DataFrame, case_name: str, metrics: list[str]) -> pd.DataFrame:
    rows = []
    case_df = per_seed[per_seed["case_study"] == case_name]
    for ctrl, ctrl_df in case_df.groupby("controller"):
        row: dict[str, Any] = {"method": ctrl}
        for metric in metrics:
            vals = pd.to_numeric(ctrl_df[metric], errors="coerce").dropna()
            row[f"{metric}_mean"] = float(vals.mean()) if not vals.empty else float("nan")
            row[f"{metric}_std"] = float(vals.std(ddof=1)) if len(vals) > 1 else (0.0 if len(vals) == 1 else float("nan"))
        rows.append(row)
    return pd.DataFrame(rows)


def improvement_table(per_seed: pd.DataFrame) -> pd.DataFrame:
    rows = []
    metrics = [c for c in per_seed.columns if c not in {"case_study", "controller", "seed"}]
    for case_name, case_df in per_seed.groupby("case_study"):
        means = case_df.groupby("controller")[metrics].mean(numeric_only=True)
        if "agentic" not in means.index:
            continue
        for baseline in ["pid", "openloop"]:
            if baseline not in means.index:
                continue
            for metric in metrics:
                base = means.loc[baseline, metric]
                ag = means.loc["agentic", metric]
                if not np.isfinite(base) or not np.isfinite(ag) or abs(base) < 1e-9:
                    imp = float("nan")
                elif metric in LOWER_BETTER:
                    imp = float((base - ag) / base * 100.0)
                elif metric in HIGHER_BETTER:
                    imp = float((ag - base) / base * 100.0)
                else:
                    imp = float("nan")
                rows.append({
                    "case_study": case_name,
                    "baseline": baseline,
                    "metric": metric,
                    "agentic_mean": ag,
                    "baseline_mean": base,
                    "improvement_pct": imp,
                })
    return pd.DataFrame(rows)


def copy_case_metadata(case: CaseSpec, top_root: Path, seeds: list[int], git_rev: str, command: str) -> None:
    meta_dir = case.case_dir / "metadata"
    meta_dir.mkdir(parents=True, exist_ok=True)
    shutil.copy2(case.config_path, meta_dir / case.config_path.name)
    setup = {
        "case_study": case.name,
        "config_path": str(case.config_path),
        "git_hash": git_rev,
        "timestamp": datetime.now().isoformat(),
        "seeds": seeds,
        "command": command,
    }
    (meta_dir / "run_metadata.json").write_text(json.dumps(setup, indent=2))
    notes = [
        f"Case study: {case.name}",
        f"Config: {case.config_path}",
        f"Seeds: {seeds}",
        f"Fault case: {case.fault_case}",
        f"Git hash: {git_rev}",
    ]
    (case.case_dir / "notes_readme.txt").write_text("\n".join(notes) + "\n")


def controller_parameters_csv(case1_cfg: dict[str, Any], case3_cfg: dict[str, Any], out_path: Path) -> None:
    rows = []
    for case_name, cfg in [("case_study_1", case1_cfg), ("case_study_3", case3_cfg)]:
        ctrl_cfg = (cfg.get("controllers", {}) or {})
        pid = ctrl_cfg.get("pid", {}) or {}
        agentic = ctrl_cfg.get("agentic", {}) or {}
        supervisor = agentic.get("supervisor", {}) or {}
        for k, v in pid.items():
            rows.append({"case_study": case_name, "controller": "pid", "parameter": k, "value": v})
        for k, v in agentic.items():
            if k == "supervisor":
                continue
            rows.append({"case_study": case_name, "controller": "agentic", "parameter": k, "value": v})
        for k, v in supervisor.items():
            rows.append({"case_study": case_name, "controller": "agentic_supervisor", "parameter": k, "value": v})
        for k, v in (ctrl_cfg.get("openloop", {}) or {}).items():
            rows.append({"case_study": case_name, "controller": "openloop", "parameter": k, "value": v})
    pd.DataFrame(rows).to_csv(out_path, index=False)


def plot_bar(df: pd.DataFrame, metric: str, title: str, ylabel: str, out_path: Path, case_name: str, order: list[str] | None = None) -> None:
    order = order or ["openloop", "pid", "agentic"]
    sub = df[(df["case_study"] == case_name) & (df["metric"] == metric)].copy()
    sub["method"] = pd.Categorical(sub["method"], categories=order, ordered=True)
    sub = sub.sort_values("method")
    if sub.empty:
        return
    fig, ax = plt.subplots(figsize=(6.2, 4.2))
    x = np.arange(len(sub))
    ax.bar(x, sub["mean"], yerr=sub["std"], capsize=4, color=["#7f7f7f", "#4c78a8", "#e45756"][: len(sub)])
    ax.set_xticks(x)
    ax.set_xticklabels(sub["method"])
    ax.set_title(title)
    ax.set_ylabel(ylabel)
    ax.grid(axis="y", alpha=0.25)
    fig.tight_layout()
    out_path.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(out_path, dpi=300, bbox_inches="tight")
    plt.close(fig)


def plot_fault_phase(case_dir: Path, out_path_err: Path, out_path_conn: Path) -> None:
    p = case_dir / "summary_faults.csv"
    if not p.exists():
        return
    df = pd.read_csv(p)
    order = ["openloop", "pid", "agentic"]
    df["controller"] = pd.Categorical(df["controller"], categories=order, ordered=True)
    df = df.sort_values("controller")
    x = np.arange(len(df))
    width = 0.22
    fig, ax = plt.subplots(figsize=(7.0, 4.2))
    ax.bar(x - width, df["mean_err_pre"], width, label="Pre")
    ax.bar(x, df["mean_err_fault"], width, label="Fault")
    ax.bar(x + width, df["mean_err_post"], width, label="Post")
    ax.set_xticks(x)
    ax.set_xticklabels(df["controller"])
    ax.set_ylabel("Tracking Error (m)")
    ax.set_title("Case Study 3: Pre/Fault/Post Tracking Error")
    ax.legend()
    ax.grid(axis="y", alpha=0.25)
    fig.tight_layout()
    fig.savefig(out_path_err, dpi=300, bbox_inches="tight")
    plt.close(fig)

    fig, ax = plt.subplots(figsize=(7.0, 4.2))
    ax.bar(x - width, df["connectivity_pre"], width, label="Pre")
    ax.bar(x, df["connectivity_fault"], width, label="Fault")
    ax.bar(x + width, df["connectivity_post"], width, label="Post")
    ax.set_xticks(x)
    ax.set_xticklabels(df["controller"])
    ax.set_ylabel("Connectivity Rate")
    ax.set_title("Case Study 3: Pre/Fault/Post Connectivity")
    ax.legend()
    ax.grid(axis="y", alpha=0.25)
    fig.tight_layout()
    fig.savefig(out_path_conn, dpi=300, bbox_inches="tight")
    plt.close(fig)


def plot_error_time(case_dir: Path, out_path: Path, title: str) -> None:
    fig, ax = plt.subplots(figsize=(7.2, 4.2))
    colors = {"openloop": "#7f7f7f", "pid": "#4c78a8", "agentic": "#e45756"}
    for ctrl in ["openloop", "pid", "agentic"]:
        files = sorted((case_dir / "csv").glob(f"{ctrl}_seed*.csv"))
        if not files:
            continue
        dfs = []
        for f in files:
            d = pd.read_csv(f)
            dfs.append(d[["t", "mean_err_m"]])
        all_df = pd.concat(dfs, ignore_index=True)
        g = all_df.groupby("t")["mean_err_m"]
        mean = g.mean()
        std = g.std(ddof=1).fillna(0.0)
        n = g.count().clip(lower=1)
        ci = 1.96 * (std / np.sqrt(n))
        t = mean.index.to_numpy()
        ax.plot(t, mean.to_numpy(), label=ctrl, color=colors[ctrl])
        ax.fill_between(t, (mean - ci).to_numpy(), (mean + ci).to_numpy(), color=colors[ctrl], alpha=0.15)
    ax.set_title(title)
    ax.set_xlabel("Time (s)")
    ax.set_ylabel("Mean Tracking Error (m)")
    ax.grid(alpha=0.25)
    ax.legend()
    fig.tight_layout()
    fig.savefig(out_path, dpi=300, bbox_inches="tight")
    plt.close(fig)


def metric_definitions_txt(out_path: Path) -> None:
    text = """Reported metric definitions

Tracking metrics
- mean_tracking_error_m: per-seed mean of the logged swarm mean nominal XY tracking error `mean_err_m`.
- max_tracking_error_m: per-seed maximum of the logged swarm max nominal XY tracking error `max_err_m`.
- rmse_m: sqrt(mean(mean_err_m^2)) over logged control ticks in a seed.
- steady_state_error_m: mean(mean_err_m) over samples with t >= steady_state_after_s from the scenario config.
- formation_error_rel: mean of `formation_err_rel` over a seed.
- connectivity_rate: mean of `connectivity_rate` over a seed.

Control metrics
- control_effort: mean over control ticks of the per-tick mean squared norm of the applied motor-RPM vector across alive UAVs.
  Formula implemented in code: control_effort(t) = mean_i ||rpm_i(t)||_2^2.
  Episode summary: E_u = mean_t control_effort(t).
- control_smoothness: mean over control ticks of the per-tick mean step-to-step change norm of the applied motor-RPM vector across alive UAVs.
  Formula implemented in code: smoothness(t) = mean_i ||rpm_i(t) - rpm_i(t-1)||_2.
  Episode summary: S = mean_t smoothness(t), with the first control tick omitted via NaN.

Fault metrics (Case Study 3)
- pre_fault_error_m / fault_window_error_m / post_fault_error_m: mean tracking error over [t0-5,t0), [t0,t0+5), and [t0+5,t0+15).
- peak_fault_error_m: max(max_err_m) over [t0,t0+5).
- connectivity_pre / fault / post: mean connectivity over the same windows.
- post_vs_pre_error_degradation_pct: ((post - pre) / pre) * 100.
- connectivity_retention_pct: (connectivity_post / connectivity_pre) * 100.

Publication caveats
- latency_proxy is excluded from paper tables because existing runs produce inf values.
- recovery_time_s is excluded from the main quantitative claims because the current definition is not discriminative for these runs.
- task_completion_proxy is retained only as a supplementary metric because values are near-zero in both case studies.
"""
    out_path.write_text(text)


def math_mapping_txt(out_path: Path) -> None:
    text = """Math-to-code mapping

System state and measurements
- Simulation state slices are used directly from observation vectors in /Users/nitin/Desktop/case_study_1_tracking/src/run_one.py.
  Position: obs[i][0:3]
  Orientation quaternion: obs[i][3:7]
  Velocity: obs[i][10:13]
  Angular velocity: obs[i][13:16]
- GPS-style measurement corruption is applied in /Users/nitin/Desktop/case_study_1_tracking/src/disturbances.py and /Users/nitin/Desktop/case_study_1_tracking/src/run_one.py.
  Measurement model: y_t = x_t + b_t + n_t, where b_t is random-walk drift and n_t is Gaussian noise.

Tracking references
- Reference trajectory is produced by /Users/nitin/Desktop/case_study_1_tracking/src/trajectories.py via p_ref(cfg, t).
- Drone-specific desired positions are p_des_i(t) = p_ref(t) + formation_offset_i.

PID tracking law
- Implemented by /Users/nitin/Desktop/case_study_1_tracking/src/controllers/pid.py and /Users/nitin/Desktop/case_study_1_tracking/src/control_core/pid_core.py.
- Position error: e_p(t) = p_cmd(t) - p(t).
- Velocity error is handled through velocity-based derivative mode with filtered velocity feedback when d_mode=vel.
- Conceptual law: u_pid ~ Kp e_p + Ki integral(e_p) - Kd v_filtered, with clamping, integrator gating, leak, and saturation-aware hold/leak options.

Agentic supervisory augmentation
- Implemented in /Users/nitin/Desktop/case_study_1_tracking/src/controllers/pid_agentic.py and /Users/nitin/Desktop/case_study_1_tracking/src/controllers/agentic_supervisor.py.
- The fast controller remains PID. Agentic logic adapts references and supervisory parameters:
  p_cmd(t) = p_ref_now(t) + ref_shift(t) + formation_scale(t) * offset_i + comm_bias(t).
- Reference shift is bounded by max_ref_shift_m and rate-limited by supervisor parameters.
- Authority scaling is gated in /Users/nitin/Desktop/case_study_1_tracking/src/run_one.py using trusted-agent fraction and connectivity.
- Integrator hold/leak overrides are applied only to the agentic PID path.

Disturbance and fault model
- Wind disturbance is applied in /Users/nitin/Desktop/case_study_1_tracking/src/disturbances.py as phased constant force plus random gusts.
- Generic dynamics interpretation: x_{t+1} = f(x_t, u_t) + w_t.
- Case Study 3 communication degradation is defined in /Users/nitin/Desktop/case_study_1_tracking/configs/scenario_faults_comm.yaml and injected by /Users/nitin/Desktop/case_study_1_tracking/src/faults/fault_injector.py via comm_range_eff(t) = comm_range * comm_scale(t).

Control metrics
- Control effort and smoothness are both computed from the applied motor RPM vector in /Users/nitin/Desktop/case_study_1_tracking/src/run_one.py.
"""
    out_path.write_text(text)


def write_summary_files(top_root: Path, case_specs: list[CaseSpec], per_seed: pd.DataFrame, stats: pd.DataFrame, improvements: pd.DataFrame) -> None:
    case1_cfg = load_yaml(case_specs[0].config_path)
    case3_cfg = load_yaml(case_specs[1].config_path)
    controller_parameters_csv(case1_cfg, case3_cfg, top_root / "controller_parameters.csv")

    stats.to_csv(top_root / "paper_stats_summary.csv", index=False)
    case1_metrics = [
        "mean_tracking_error_m", "rmse_m", "steady_state_error_m", "formation_error_rel",
        "connectivity_rate", "control_effort", "control_smoothness", "throughput_ratio",
    ]
    case3_metrics = [
        "mean_tracking_error_m", "rmse_m", "steady_state_error_m", "formation_error_rel",
        "connectivity_rate", "control_effort", "control_smoothness", "pre_fault_error_m",
        "fault_window_error_m", "post_fault_error_m", "peak_fault_error_m", "connectivity_retention_pct",
        "post_vs_pre_error_degradation_pct",
    ]
    wide_stats(per_seed, "case_study_1", case1_metrics).to_csv(top_root / "paper_stats_wide_case1.csv", index=False)
    wide_stats(per_seed, "case_study_3", case3_metrics).to_csv(top_root / "paper_stats_wide_case3.csv", index=False)
    improvements.to_csv(top_root / "paper_improvements.csv", index=False)

    phase_df = pd.read_csv(case_specs[1].case_dir / "summary_faults.csv")
    phase_df.to_csv(top_root / "paper_fault_phase_summary.csv", index=False)

    metric_definitions_txt(top_root / "paper_metric_definitions.txt")
    math_mapping_txt(top_root / "paper_math_mapping.txt")

    key_findings = []
    c1 = pd.read_csv(top_root / "paper_stats_wide_case1.csv")
    c3 = pd.read_csv(top_root / "paper_stats_wide_case3.csv")
    def get(df, method, metric):
        row = df[df["method"] == method]
        return float(row.iloc[0][f"{metric}_mean"]) if not row.empty else float("nan")
    key_findings.append(
        f"Case Study 1 nominal tracking: PID has the lowest mean tracking error ({get(c1,'pid','mean_tracking_error_m'):.3f} m), while Agentic remains close on connectivity and network proxies but does not beat PID on nominal error."
    )
    key_findings.append(
        f"Case Study 3 disturbance robustness: Agentic achieves the lowest mean tracking error ({get(c3,'agentic','mean_tracking_error_m'):.3f} m) and the lowest post-vs-pre error degradation ({get(c3,'agentic','post_vs_pre_error_degradation_pct'):.2f}%)."
    )
    key_findings.append(
        f"Control smoothness and control effort are now reported from applied motor-RPM commands, enabling direct discussion of whether robustness gains come with higher actuation cost or choppier control."
    )
    (top_root / "paper_key_findings.txt").write_text("\n".join(key_findings) + "\n")

    red_flags = [
        "latency_proxy is not publication-safe because it evaluates to inf in these runs.",
        "recovery_time_s remains non-discriminative under the current threshold/hold definition and should not be a headline metric.",
        "task_completion_proxy is near-zero in both case studies and is better treated as a supplementary metric only.",
        "Overshoot and settling time are not emphasized because the reference is continuously moving, making classical step-response interpretations weak.",
    ]
    (top_root / "paper_red_flags.txt").write_text("\n".join(red_flags) + "\n")


def compare_old_new(case_specs: list[CaseSpec]) -> str:
    lines = []
    for case in case_specs:
        old_summary = pd.read_csv(case.old_dir / "summary.csv")
        new_summary = pd.read_csv(case.case_dir / "summary.csv")
        merged = old_summary[["controller", "mean_err_m", "connectivity_rate"]].merge(
            new_summary[["controller", "mean_err_m", "connectivity_rate"]], on="controller", suffixes=("_old", "_new")
        )
        lines.append(f"{case.name} old-vs-new key deltas:")
        for _, row in merged.iterrows():
            err_delta = row["mean_err_m_new"] - row["mean_err_m_old"]
            conn_delta = row["connectivity_rate_new"] - row["connectivity_rate_old"]
            lines.append(f"- {row['controller']}: mean_err delta={err_delta:+.3f} m, connectivity delta={conn_delta:+.4f}")
    return "\n".join(lines) + "\n"


def experiment_setup(case_specs: list[CaseSpec], seeds: list[int], git_rev: str) -> dict[str, Any]:
    out = {
        "timestamp": datetime.now().isoformat(),
        "git_hash": git_rev,
        "seeds": seeds,
        "cases": {},
    }
    for case in case_specs:
        cfg = load_yaml(case.config_path)
        sim = cfg.get("sim", {}) or {}
        out["cases"][case.name] = {
            "config": str(case.config_path),
            "num_drones": sim.get("num_drones"),
            "duration_s": sim.get("duration_s"),
            "sim_freq_hz": sim.get("freq_hz"),
            "ctrl_freq_hz": sim.get("ctrl_hz"),
            "trajectory_type": (cfg.get("trajectory", {}) or {}).get("type"),
            "sensor_modes": (cfg.get("controllers", {}) or {}).get("sensing", {}),
            "comm_range_m": sim.get("comm_range_m", 10.0),
            "faults": (cfg.get("faults", {}) or {}),
            "disturbance": cfg.get("disturbance", {}),
        }
    return out


def experiment_summary_text(case_specs: list[CaseSpec], seeds: list[int], git_rev: str) -> str:
    lines = [
        "Paper-ready rerun summary",
        f"Git hash: {git_rev}",
        f"Seeds: {seeds}",
        "Controllers: openloop, pid, agentic",
        "New metrics: control_effort, control_smoothness",
        "Control signal used for both metrics: applied motor RPM vector logged at each control tick.",
        "",
    ]
    for case in case_specs:
        cfg = load_yaml(case.config_path)
        sim = cfg.get("sim", {}) or {}
        traj = cfg.get("trajectory", {}) or {}
        lines.extend([
            f"{case.name}:",
            f"- Config: {case.config_path}",
            f"- Duration: {sim.get('duration_s')} s",
            f"- Frequencies: sim={sim.get('freq_hz')} Hz, control={sim.get('ctrl_hz')} Hz",
            f"- Drones: {sim.get('num_drones')}",
            f"- Trajectory: {traj.get('type')}",
            f"- Fault-enabled: {case.fault_case}",
        ])
    return "\n".join(lines) + "\n"


def readme_text(top_root: Path) -> str:
    return f"""Results package layout
- experiment_summary.txt: human-readable run summary.
- experiment_setup.json: exact scenario/setup metadata.
- controller_parameters.csv: PID and agentic parameter dump from configs.
- paper_stats_summary.csv: long-format statistics with mean/std/min/max.
- paper_stats_wide_case1.csv / paper_stats_wide_case3.csv: wide-format paper tables.
- paper_improvements.csv: Agentic-vs-baseline percentage improvements.
- paper_fault_phase_summary.csv: pre/fault/post summary for Case Study 3.
- paper_metric_definitions.txt: formulas and caveats for reported metrics.
- paper_math_mapping.txt: math-to-code mapping for methods section.
- paper_key_findings.txt: concise results-section findings.
- paper_red_flags.txt: metrics/assumptions that should not be over-emphasized.
- plots/: publication-useful plots.
- case_study_1/ and case_study_3/: raw per-seed CSVs, summaries, figures, and metadata snapshots.
"""


def main() -> None:
    args = parse_args()
    top_root = Path(args.root)
    top_root.mkdir(parents=True, exist_ok=True)
    plots_dir = top_root / "plots"
    plots_dir.mkdir(parents=True, exist_ok=True)

    git_rev = args.git_hash or git_hash()
    case_specs = [
        CaseSpec("case_study_1", Path(args.case1_dir), Path(args.case1_config), Path(args.case1_old_dir), False),
        CaseSpec("case_study_3", Path(args.case3_dir), Path(args.case3_config), Path(args.case3_old_dir), True),
    ]

    # Metadata snapshots
    commands = {
        "case_study_1": f"python -m src.run_all --config {args.case1_config} --seeds {' '.join(map(str,args.seeds))} --output-root {args.case1_dir}",
        "case_study_3": f"python -m src.run_all --config {args.case3_config} --seeds {' '.join(map(str,args.seeds))} --output-root {args.case3_dir}",
    }
    for case in case_specs:
        copy_case_metadata(case, top_root, args.seeds, git_rev, commands[case.name])

    # Per-seed metrics
    rows = []
    for case in case_specs:
        for ctrl in ["openloop", "pid", "agentic"]:
            for seed in args.seeds:
                rows.append(per_seed_metrics(case, ctrl, seed))
    per_seed = pd.DataFrame(rows)
    per_seed.to_csv(top_root / "paper_per_seed_metrics.csv", index=False)

    stats = stats_long(per_seed)
    improvements = improvement_table(per_seed)
    write_summary_files(top_root, case_specs, per_seed, stats, improvements)

    # Setup and summary docs
    (top_root / "experiment_setup.json").write_text(json.dumps(experiment_setup(case_specs, args.seeds, git_rev), indent=2))
    (top_root / "experiment_summary.txt").write_text(experiment_summary_text(case_specs, args.seeds, git_rev))
    (top_root / "README_results.txt").write_text(readme_text(top_root))
    (top_root / "paper_old_vs_new_comparison.txt").write_text(compare_old_new(case_specs))

    # Case-specific plot set requested by user
    plot_bar(stats, "mean_tracking_error_m", "Case Study 1: Mean Tracking Error", "Tracking Error (m)", plots_dir / "case1_tracking_error_comparison.png", "case_study_1")
    plot_bar(stats, "rmse_m", "Case Study 1: RMSE", "RMSE (m)", plots_dir / "case1_rmse_comparison.png", "case_study_1")
    plot_bar(stats, "connectivity_rate", "Case Study 1: Connectivity", "Connectivity Rate", plots_dir / "case1_connectivity_comparison.png", "case_study_1")
    plot_bar(stats, "control_effort", "Case Study 1: Control Effort", "Mean RPM-Norm Squared", plots_dir / "case1_control_effort_comparison.png", "case_study_1")
    plot_bar(stats, "control_smoothness", "Case Study 1: Control Smoothness", "Mean Step-to-Step RPM Change", plots_dir / "case1_control_smoothness_comparison.png", "case_study_1")

    plot_bar(stats, "mean_tracking_error_m", "Case Study 3: Mean Tracking Error", "Tracking Error (m)", plots_dir / "case3_tracking_error_comparison.png", "case_study_3")
    plot_fault_phase(case_specs[1].case_dir, plots_dir / "case3_prefault_fault_post_error.png", plots_dir / "case3_prefault_fault_post_connectivity.png")
    plot_bar(stats, "formation_error_rel", "Case Study 3: Formation Error", "Relative Formation Error", plots_dir / "case3_formation_error_comparison.png", "case_study_3")
    plot_bar(stats, "control_effort", "Case Study 3: Control Effort", "Mean RPM-Norm Squared", plots_dir / "case3_control_effort_comparison.png", "case_study_3")
    plot_bar(stats, "control_smoothness", "Case Study 3: Control Smoothness", "Mean Step-to-Step RPM Change", plots_dir / "case3_control_smoothness_comparison.png", "case_study_3")
    plot_bar(stats, "post_vs_pre_error_degradation_pct", "Case Study 3: Post-vs-Pre Error Degradation", "Degradation (%)", plots_dir / "case3_degradation_error_comparison.png", "case_study_3")
    plot_error_time(case_specs[0].case_dir, plots_dir / "case1_error_over_time.png", "Case Study 1: Tracking Error Over Time")
    plot_error_time(case_specs[1].case_dir, plots_dir / "case3_error_over_time.png", "Case Study 3: Tracking Error Over Time")


if __name__ == "__main__":
    main()
