from __future__ import annotations

import argparse
import copy
import json
import subprocess
from pathlib import Path
from typing import Any

import numpy as np
import pandas as pd
import yaml

from src.run_all import summarize, summarize_faults


def parse_args() -> argparse.Namespace:
    ap = argparse.ArgumentParser()
    ap.add_argument('--base-config', required=True)
    ap.add_argument('--seeds', nargs='+', type=int, required=True)
    ap.add_argument('--output-root', required=True)
    return ap.parse_args()


def load_yaml(path: Path) -> dict[str, Any]:
    with open(path, 'r') as f:
        return yaml.safe_load(f) or {}


def save_yaml(data: dict[str, Any], path: Path) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with open(path, 'w') as f:
        yaml.safe_dump(data, f, sort_keys=False)


def aggregate_method(method_dir: Path, seeds: list[int]) -> pd.DataFrame:
    dfs = []
    for s in seeds:
        p = method_dir / 'csv' / f'agentic_seed{s}.csv'
        if not p.exists():
            raise FileNotFoundError(p)
        df = pd.read_csv(p)
        df['seed'] = s
        dfs.append(df)
    return pd.concat(dfs, ignore_index=True)


def aggregate_pid(method_dir: Path, seeds: list[int]) -> pd.DataFrame:
    dfs = []
    for s in seeds:
        p = method_dir / 'csv' / f'pid_seed{s}.csv'
        if not p.exists():
            raise FileNotFoundError(p)
        df = pd.read_csv(p)
        df['seed'] = s
        dfs.append(df)
    return pd.concat(dfs, ignore_index=True)


def per_seed_metrics(method_dir: Path, controller_file_prefix: str, seeds: list[int], t0: float, label: str) -> pd.DataFrame:
    rows = []
    for s in seeds:
        p = method_dir / 'csv' / f'{controller_file_prefix}_seed{s}.csv'
        df = pd.read_csv(p)
        mean_err = float(df['mean_err_m'].mean())
        rmse = float(np.sqrt(np.mean(np.square(df['mean_err_m'].to_numpy(dtype=float)))))
        conn = float(df['connectivity_rate'].mean())
        form = float(df['formation_err_rel'].mean())
        eff = float(df['control_effort'].mean()) if 'control_effort' in df.columns else float('nan')
        smooth = float(df['control_smoothness'].mean(skipna=True)) if 'control_smoothness' in df.columns else float('nan')
        fault = summarize_faults(df, label, t0)
        rows.append({
            'method': label,
            'seed': s,
            'mean_error_m': mean_err,
            'rmse_m': rmse,
            'connectivity': conn,
            'formation_error_rel': form,
            'control_effort': eff,
            'control_smoothness': smooth,
            'pre_error_m': fault['mean_err_pre'],
            'fault_error_m': fault['mean_err_fault'],
            'post_error_m': fault['mean_err_post'],
            'peak_fault_error_m': fault['max_err_fault'],
            'connectivity_pre': fault['connectivity_pre'],
            'connectivity_fault': fault['connectivity_fault'],
            'connectivity_post': fault['connectivity_post'],
        })
    return pd.DataFrame(rows)


def summarize_seed_metrics(df: pd.DataFrame) -> pd.DataFrame:
    metric_cols = [c for c in df.columns if c not in {'method', 'seed'}]
    rows = []
    for method, g in df.groupby('method'):
        row = {'method': method}
        for col in metric_cols:
            vals = pd.to_numeric(g[col], errors='coerce').dropna()
            row[f'{col}_mean'] = float(vals.mean()) if not vals.empty else float('nan')
            row[f'{col}_std'] = float(vals.std(ddof=1)) if len(vals) > 1 else (0.0 if len(vals) == 1 else float('nan'))
        rows.append(row)
    return pd.DataFrame(rows)


def set_supervisor_off(cfg: dict[str, Any]) -> None:
    sup = cfg['controllers']['agentic']['supervisor']
    sup['conn_thresh'] = -1.0
    sup['err_thresh_m'] = 1e9
    sup['err_hard_m'] = 1e9
    sup['phase_boost_s'] = 0.0
    sup['phase_boost_rate_mps'] = 0.0
    sup['recovery_ref_shift_rate_mps'] = 0.0
    sup['ref_shift_rate_mps'] = 0.0
    sup['ref_shift_decay_mps'] = 0.0
    sup['formation_scale_min'] = 1.0
    sup['formation_scale_max'] = 1.0
    sup['smooth_alpha_active'] = 1.0
    sup['phase_boost_smooth_alpha'] = 1.0
    sup['comm_freeze_refshift'] = False


def main() -> None:
    args = parse_args()
    seeds = list(args.seeds)
    out_root = Path(args.output_root)
    out_root.mkdir(parents=True, exist_ok=True)
    configs_dir = out_root / 'configs'

    base = load_yaml(Path(args.base_config))
    t0 = float(base['faults']['events'][0]['t0_s'])

    # PID baseline
    pid_dir = out_root / 'pid_baseline'
    subprocess.check_call([
        'python', '-m', 'src.run_all',
        '--config', args.base_config,
        '--seeds', *[str(s) for s in seeds],
        '--output-root', str(pid_dir),
    ])

    variants: list[tuple[str, dict[str, Any]]] = []

    # Ref shift only: lookahead/ref-shift path enabled, comm bias off, supervisor off.
    cfg = copy.deepcopy(base)
    cfg['controllers']['agentic']['comm_bias_m'] = 0.0
    cfg['controllers']['agentic']['authority_connectivity_gate'] = -1.0
    cfg['controllers']['agentic']['authority_connectivity_scale'] = 1.0
    cfg['controllers']['agentic']['authority_untrusted_scale'] = 1.0
    set_supervisor_off(cfg)
    variants.append(('ref_shift_only', cfg))

    # Comm bias only: disable ref-shift path and supervisor, keep comm bias.
    cfg = copy.deepcopy(base)
    cfg['controllers']['agentic']['lookahead_s'] = 0.0
    cfg['controllers']['agentic']['max_ref_shift_m'] = 0.0
    cfg['controllers']['agentic']['comm_bias_m'] = 0.18
    cfg['controllers']['agentic']['authority_connectivity_gate'] = -1.0
    cfg['controllers']['agentic']['authority_connectivity_scale'] = 1.0
    cfg['controllers']['agentic']['authority_untrusted_scale'] = 1.0
    set_supervisor_off(cfg)
    variants.append(('comm_bias_only', cfg))

    # Formation scaling only: disable direct lookahead/comm bias, keep supervisor mode 3 scaling only.
    cfg = copy.deepcopy(base)
    cfg['controllers']['agentic']['lookahead_s'] = 0.0
    cfg['controllers']['agentic']['max_ref_shift_m'] = 0.0
    cfg['controllers']['agentic']['comm_bias_m'] = 0.0
    sup = cfg['controllers']['agentic']['supervisor']
    sup['conn_thresh'] = 0.7
    sup['err_thresh_m'] = 1e9
    sup['err_hard_m'] = 1e9
    sup['ref_shift_rate_mps'] = 0.0
    sup['ref_shift_decay_mps'] = 0.0
    sup['recovery_ref_shift_rate_mps'] = 0.0
    sup['phase_boost_s'] = 0.0
    sup['phase_boost_rate_mps'] = 0.0
    sup['smooth_alpha_active'] = 1.0
    sup['phase_boost_smooth_alpha'] = 1.0
    sup['formation_scale_min'] = 0.9
    sup['formation_scale_max'] = 1.0
    variants.append(('formation_scaling_only', cfg))

    # Full agentic core architecture: keep the three main supervisory mechanisms
    # while disabling the extra transient re-anchor boosts that dominated the
    # 5-seed ablation and obscured component attribution.
    cfg = copy.deepcopy(base)
    sup = cfg['controllers']['agentic']['supervisor']
    sup['phase_boost_s'] = 0.0
    sup['phase_boost_rate_mps'] = 0.0
    sup['phase_boost_smooth_alpha'] = 1.0
    sup['recovery_ref_shift_rate_mps'] = 0.0
    variants.append(('full_agentic', cfg))

    records = []

    pid_seed = per_seed_metrics(pid_dir, 'pid', seeds, t0, 'PID')
    records.append(pid_seed)

    for name, cfg in variants:
        cfg_path = configs_dir / f'{name}.yaml'
        save_yaml(cfg, cfg_path)
        method_dir = out_root / name
        for s in seeds:
            subprocess.check_call([
                'python', '-m', 'src.run_one',
                '--config', str(cfg_path),
                '--controller', 'agentic',
                '--seed', str(s),
                '--output-root', str(method_dir),
            ])
        df = aggregate_method(method_dir, seeds)
        summary = pd.DataFrame([summarize(df, name)])
        summary.to_csv(method_dir / 'summary.csv', index=False)
        fault_summary = pd.DataFrame([summarize_faults(df, name, t0)])
        fault_summary.to_csv(method_dir / 'summary_faults.csv', index=False)
        records.append(per_seed_metrics(method_dir, 'agentic', seeds, t0, name))

    all_seed = pd.concat(records, ignore_index=True)
    all_seed.to_csv(out_root / 'ablation_per_seed.csv', index=False)
    wide = summarize_seed_metrics(all_seed)
    wide.to_csv(out_root / 'ablation_summary.csv', index=False)

    # Compact table for paper
    keep = [
        'method',
        'mean_error_m_mean', 'rmse_m_mean', 'connectivity_mean',
        'post_error_m_mean', 'peak_fault_error_m_mean',
        'control_effort_mean', 'control_smoothness_mean',
    ]
    paper = wide[keep].copy()
    label_map = {
        'PID': 'PID',
        'ref_shift_only': 'PID + Ref Shift',
        'comm_bias_only': 'PID + Comm Bias',
        'formation_scaling_only': 'PID + Formation Scaling',
        'full_agentic': 'PID + Full Agentic',
    }
    paper['method'] = paper['method'].map(label_map).fillna(paper['method'])
    paper.to_csv(out_root / 'ablation_table_for_paper.csv', index=False)

    # Improvements over PID
    pid_row = paper[paper['method'] == 'PID'].iloc[0]
    imp_rows = []
    for _, row in paper.iterrows():
        if row['method'] == 'PID':
            continue
        imp_rows.append({
            'method': row['method'],
            'mean_error_improvement_pct': (pid_row['mean_error_m_mean'] - row['mean_error_m_mean']) / pid_row['mean_error_m_mean'] * 100.0,
            'rmse_improvement_pct': (pid_row['rmse_m_mean'] - row['rmse_m_mean']) / pid_row['rmse_m_mean'] * 100.0,
            'post_error_improvement_pct': (pid_row['post_error_m_mean'] - row['post_error_m_mean']) / pid_row['post_error_m_mean'] * 100.0,
            'peak_fault_improvement_pct': (pid_row['peak_fault_error_m_mean'] - row['peak_fault_error_m_mean']) / pid_row['peak_fault_error_m_mean'] * 100.0,
            'connectivity_improvement_pct': (row['connectivity_mean'] - pid_row['connectivity_mean']) / pid_row['connectivity_mean'] * 100.0,
        })
    pd.DataFrame(imp_rows).to_csv(out_root / 'ablation_improvements_vs_pid.csv', index=False)

    meta = {
        'base_config': args.base_config,
        'seeds': seeds,
        'variants': list(label_map.values()),
    }
    (out_root / 'ablation_metadata.json').write_text(json.dumps(meta, indent=2))


if __name__ == '__main__':
    main()
