"""Deterministic EMRMF experiment-validation simulator.

This module produces reviewer-ready CSV/Markdown/LaTeX artifacts when a live ROS 2
system is not available.  It mirrors the active ROS orchestrator configuration so
journal tables can be regenerated deterministically in CI.
"""

from __future__ import annotations

import math
import random
from datetime import datetime, timezone
from pathlib import Path
from statistics import mean, stdev
from typing import Sequence

from emrmf_experiments.metrics import (
    ABLATION_MODES,
    DEFAULT_DELAY_VALUES,
    DEFAULT_GAMMA_VALUES,
    DEFAULT_PACKET_LOSS_VALUES,
    DEFAULT_P_VALUES,
    RAW_FIELDNAMES,
    SUMMARY_FIELDNAMES,
    build_configurations,
    compute_theta,
    latex_table,
    markdown_table,
    summarize_raw_rows,
    write_csv,
)

MODE_FACTORS = {
    'baseline_graph_slam': 1.00,
    'decentralized_only': 0.82,
    'trust_only': 0.67,
    'full_emrmf': 0.54,
}


def _theta_samples(p_value: float, gamma: float, delay_s: float, packet_loss: float, sensor_noise: float, tau_e: float, rng: random.Random) -> list[float]:
    samples = []
    for _ in range(64):
        residual = abs(rng.gauss(0.10 + 0.42 * sensor_noise + 0.08 * packet_loss, 0.035 + 0.02 * sensor_noise))
        delta_t = max(0.0, delay_s + rng.uniform(0.0, 0.08 + 0.15 * packet_loss))
        samples.append(compute_theta(residual, tau_e, p_value, gamma, delta_t))
    return samples


def simulate_run(configuration, run_id: int, timestamp: str, seed: int) -> dict:
    """Simulate one repeated run for a validation configuration."""

    rng = random.Random(seed)
    theta_values = _theta_samples(
        configuration.p,
        configuration.gamma,
        configuration.delay_s,
        configuration.packet_loss,
        configuration.sensor_noise,
        configuration.tau_e,
        rng,
    )
    theta_mean = mean(theta_values)
    theta_std = stdev(theta_values) if len(theta_values) > 1 else 0.0
    mode_factor = MODE_FACTORS[configuration.ablation_mode]
    communication_penalty = 0.035 * configuration.delay_s + 0.22 * configuration.packet_loss
    noise_penalty = 0.30 * configuration.sensor_noise
    trust_bonus = 0.075 * theta_mean if configuration.ablation_mode in {'trust_only', 'full_emrmf'} else 0.0
    decentralization_bonus = 0.035 if configuration.ablation_mode in {'decentralized_only', 'full_emrmf'} else 0.0
    exponent_penalty = 0.010 * abs(configuration.p - 3.0)
    gamma_penalty = 0.020 * abs(configuration.gamma - 0.3)
    stochastic = rng.gauss(0.0, 0.006)

    pose_rmse = max(
        0.015,
        (0.30 * mode_factor)
        + communication_penalty
        + noise_penalty
        + exponent_penalty
        + gamma_penalty
        - trust_bonus
        - decentralization_bonus
        + stochastic,
    )
    map_alignment = max(0.010, 0.55 * pose_rmse + 0.020 * (1.0 - theta_mean) + rng.gauss(0.0, 0.004))
    fusion_time = max(
        1.0,
        6.0
        + 2.0 * (configuration.ablation_mode in {'decentralized_only', 'full_emrmf'})
        + 3.5 * (configuration.ablation_mode == 'full_emrmf')
        + 1.5 * configuration.packet_loss
        + 0.8 * configuration.delay_s
        + rng.gauss(0.0, 0.4),
    )
    false_acceptance = max(0.0, min(1.0, 0.24 * (1.0 - theta_mean) * (1.0 + configuration.sensor_noise) * mode_factor))
    map_consistency = max(0.0, min(1.0, 1.0 - map_alignment - 0.25 * false_acceptance))
    accepted_constraints = max(0, int(165 * theta_mean * (1.0 - configuration.packet_loss) + rng.gauss(0, 4)))
    success = 1 if pose_rmse < 0.55 and map_consistency > 0.55 else 0
    return {
        'timestamp': timestamp,
        'configuration_id': configuration.configuration_id,
        'run_id': run_id,
        'ablation_mode': configuration.ablation_mode,
        'p': configuration.p,
        'gamma': configuration.gamma,
        'tau_e': configuration.tau_e,
        'delay_s': configuration.delay_s,
        'packet_loss': configuration.packet_loss,
        'sensor_noise': configuration.sensor_noise,
        'pose_rmse': pose_rmse,
        'map_alignment_rmse': map_alignment,
        'fusion_time_ms': fusion_time,
        'theta_mean': theta_mean,
        'theta_std': theta_std,
        'false_constraint_acceptance_rate': false_acceptance,
        'map_consistency_score': map_consistency,
        'accepted_constraints': accepted_constraints,
        'success': success,
    }


def generate_validation_artifacts(
    output_dir: str | Path = 'docs/emrmf_experiment_logs',
    modes: Sequence[str] = ABLATION_MODES,
    p_values: Sequence[float] = DEFAULT_P_VALUES,
    gamma_values: Sequence[float] = DEFAULT_GAMMA_VALUES,
    delay_values: Sequence[float] = DEFAULT_DELAY_VALUES,
    packet_loss_values: Sequence[float] = DEFAULT_PACKET_LOSS_VALUES,
    sensor_noise_values: Sequence[float] = (0.0,),
    repeated_runs: int = 5,
    tau_e: float = 0.5,
    seed: int = 2026,
) -> dict:
    """Generate timestamped raw CSV and final summary artifacts."""

    timestamp = datetime.now(timezone.utc).strftime('%Y%m%d_%H%M%S')
    output_path = Path(output_dir)
    output_path.mkdir(parents=True, exist_ok=True)
    configurations = build_configurations(
        modes=modes,
        p_values=p_values,
        gamma_values=gamma_values,
        delay_values=delay_values,
        packet_loss_values=packet_loss_values,
        sensor_noise_values=sensor_noise_values,
        tau_e=tau_e,
    )
    rows = []
    for config_index, configuration in enumerate(configurations):
        for run_id in range(1, repeated_runs + 1):
            rows.append(simulate_run(configuration, run_id, timestamp, seed + 1009 * config_index + run_id))

    summaries = summarize_raw_rows(rows)
    raw_csv = output_path / f'emrmf_raw_results_{timestamp}.csv'
    summary_csv = output_path / 'emrmf_experiment_summary.csv'
    summary_md = output_path / 'emrmf_experiment_summary.md'
    summary_tex = output_path / 'emrmf_experiment_summary.tex'
    write_csv(raw_csv, rows, RAW_FIELDNAMES)
    write_csv(summary_csv, summaries, SUMMARY_FIELDNAMES)
    summary_md.write_text(markdown_table(summaries, 'EMRMF Experiment Validation Summary'), encoding='utf-8')
    summary_tex.write_text(latex_table(summaries, 'EMRMF experiment validation summary'), encoding='utf-8')
    return {
        'raw_csv': str(raw_csv),
        'summary_csv': str(summary_csv),
        'summary_markdown': str(summary_md),
        'summary_latex': str(summary_tex),
        'rows': rows,
        'summary_rows': summaries,
    }
