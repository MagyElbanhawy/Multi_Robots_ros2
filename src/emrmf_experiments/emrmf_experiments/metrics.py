"""Experiment metrics and table exporters for EMRMF journal validation."""

from __future__ import annotations

import csv
import math
from collections import defaultdict
from dataclasses import dataclass
from pathlib import Path
from statistics import mean, stdev
from typing import Iterable, Sequence

ABLATION_MODES = (
    'baseline_graph_slam',
    'decentralized_only',
    'trust_only',
    'full_emrmf',
)
DEFAULT_P_VALUES = (2.0, 3.0, 4.0)
DEFAULT_GAMMA_VALUES = (0.1, 0.3, 0.5, 1.0)
DEFAULT_DELAY_VALUES = (0.0, 0.5, 2.0)
DEFAULT_PACKET_LOSS_VALUES = (0.0, 0.10, 0.30)
DEFAULT_SENSOR_NOISE_VALUES = (0.10, 0.20, 0.30)
DEFAULT_REPEATED_RUNS = 5
RAW_FIELDNAMES = (
    'timestamp',
    'configuration_id',
    'run_id',
    'ablation_mode',
    'p',
    'gamma',
    'tau_e',
    'delay_s',
    'packet_loss',
    'sensor_noise',
    'pose_rmse',
    'map_alignment_rmse',
    'fusion_time_ms',
    'theta_mean',
    'theta_std',
    'false_constraint_acceptance_rate',
    'map_consistency_score',
    'accepted_constraints',
    'success',
)
SUMMARY_FIELDNAMES = (
    'ablation_mode',
    'p',
    'gamma',
    'delay_s',
    'packet_loss',
    'sensor_noise',
    'runs',
    'pose_rmse_mean',
    'pose_rmse_std',
    'pose_rmse_ci95',
    'map_alignment_rmse_mean',
    'map_alignment_rmse_std',
    'map_alignment_rmse_ci95',
    'fusion_time_ms_mean',
    'fusion_time_ms_std',
    'fusion_time_ms_ci95',
    'theta_mean',
    'theta_std',
    'false_constraint_acceptance_rate_mean',
    'map_consistency_score_mean',
    'accepted_constraints_mean',
    'success_rate',
)


def compute_theta(error_norm: float, tau_e: float, p: float, gamma: float, delta_t: float) -> float:
    """Compute the EMRMF trust factor requested for the Springer revision.

    theta = max(0, 1 - (norm(e_ij) / tau_e)^p) * exp(-gamma * delta_t)
    """

    if tau_e <= 0.0:
        raise ValueError('tau_e must be positive')
    spatial_component = max(0.0, 1.0 - (max(0.0, error_norm) / tau_e) ** p)
    temporal_component = math.exp(-max(0.0, gamma) * max(0.0, delta_t))
    return spatial_component * temporal_component


def euclidean_distance(a: Sequence[float], b: Sequence[float]) -> float:
    """Return Euclidean distance for equally sized coordinate vectors."""

    return math.sqrt(sum((float(left) - float(right)) ** 2 for left, right in zip(a, b)))


def root_mean_square(values: Iterable[float]) -> float:
    """Return root mean square for a non-empty list of residual magnitudes."""

    items = [float(value) for value in values]
    if not items:
        return 0.0
    return math.sqrt(sum(value * value for value in items) / len(items))


def pose_rmse(estimated_poses: Sequence[Sequence[float]], reference_poses: Sequence[Sequence[float]]) -> float:
    """Compute pose RMSE from aligned estimated/reference pose vectors."""

    count = min(len(estimated_poses), len(reference_poses))
    if count == 0:
        return 0.0
    residuals = [euclidean_distance(estimated_poses[index], reference_poses[index]) for index in range(count)]
    return root_mean_square(residuals)


def alignment_rmse(
    source_points: Sequence[Sequence[float]],
    target_points: Sequence[Sequence[float]],
    nearest_neighbor: bool = True,
) -> float:
    """Compute map-alignment RMSE for ordered or nearest-neighbor correspondences."""

    if not source_points or not target_points:
        return 0.0
    if nearest_neighbor:
        residuals = [min(euclidean_distance(source, target) for target in target_points) for source in source_points]
    else:
        count = min(len(source_points), len(target_points))
        residuals = [euclidean_distance(source_points[index], target_points[index]) for index in range(count)]
    return root_mean_square(residuals)


def confidence_interval_95(values: Sequence[float]) -> float:
    """Return two-sided 95% CI half-width using 1.96 * s / sqrt(n)."""

    if len(values) <= 1:
        return 0.0
    return 1.96 * stdev(values) / math.sqrt(len(values))


def summary_stats(values: Sequence[float]) -> dict[str, float]:
    """Return mean, sample standard deviation, and 95% CI half-width."""

    items = [float(value) for value in values]
    if not items:
        return {'mean': 0.0, 'std': 0.0, 'ci95': 0.0}
    return {
        'mean': mean(items),
        'std': stdev(items) if len(items) > 1 else 0.0,
        'ci95': confidence_interval_95(items),
    }


def summarize_raw_rows(rows: Sequence[dict]) -> list[dict]:
    """Aggregate raw experiment rows by mode, p, gamma, delay, loss, and noise."""

    grouped: dict[tuple, list[dict]] = defaultdict(list)
    for row in rows:
        key = (
            row['ablation_mode'],
            float(row['p']),
            float(row['gamma']),
            float(row['delay_s']),
            float(row['packet_loss']),
            float(row.get('sensor_noise', 0.0)),
        )
        grouped[key].append(row)

    summaries: list[dict] = []
    for key, group in sorted(grouped.items()):
        mode, p_value, gamma, delay_s, packet_loss, sensor_noise = key
        pose = summary_stats([float(row['pose_rmse']) for row in group])
        align = summary_stats([float(row['map_alignment_rmse']) for row in group])
        fusion = summary_stats([float(row['fusion_time_ms']) for row in group])
        theta_mean = summary_stats([float(row['theta_mean']) for row in group])
        theta_std = summary_stats([float(row['theta_std']) for row in group])
        false_accept = summary_stats([float(row.get('false_constraint_acceptance_rate', 0.0)) for row in group])
        consistency = summary_stats([float(row.get('map_consistency_score', 0.0)) for row in group])
        accepted = summary_stats([float(row.get('accepted_constraints', 0.0)) for row in group])
        success_rate = mean([float(row.get('success', 1.0)) for row in group])
        summaries.append({
            'ablation_mode': mode,
            'p': p_value,
            'gamma': gamma,
            'delay_s': delay_s,
            'packet_loss': packet_loss,
            'sensor_noise': sensor_noise,
            'runs': len(group),
            'pose_rmse_mean': pose['mean'],
            'pose_rmse_std': pose['std'],
            'pose_rmse_ci95': pose['ci95'],
            'map_alignment_rmse_mean': align['mean'],
            'map_alignment_rmse_std': align['std'],
            'map_alignment_rmse_ci95': align['ci95'],
            'fusion_time_ms_mean': fusion['mean'],
            'fusion_time_ms_std': fusion['std'],
            'fusion_time_ms_ci95': fusion['ci95'],
            'theta_mean': theta_mean['mean'],
            'theta_std': theta_std['mean'],
            'false_constraint_acceptance_rate_mean': false_accept['mean'],
            'map_consistency_score_mean': consistency['mean'],
            'accepted_constraints_mean': accepted['mean'],
            'success_rate': success_rate,
        })
    return summaries


def write_csv(path: str | Path, rows: Sequence[dict], fieldnames: Sequence[str] | None = None) -> None:
    """Write dictionaries to CSV, creating parent directories as needed."""

    output_path = Path(path)
    output_path.parent.mkdir(parents=True, exist_ok=True)
    fields = list(fieldnames or (rows[0].keys() if rows else []))
    with output_path.open('w', newline='', encoding='utf-8') as handle:
        writer = csv.DictWriter(handle, fieldnames=fields, extrasaction='ignore')
        writer.writeheader()
        for row in rows:
            writer.writerow(row)


def markdown_table(rows: Sequence[dict], title: str = 'EMRMF Experiment Summary') -> str:
    """Render a reviewer-friendly Markdown summary table."""

    lines = [f'# {title}', '']
    lines.append('| Mode | p | gamma | delay | loss | noise | runs | pose RMSE mean | std | 95% CI | alignment RMSE | fusion ms | theta mean | theta std |')
    lines.append('| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |')
    for row in rows:
        lines.append(
            f"| {row['ablation_mode']} | {float(row['p']):.0f} | {float(row['gamma']):.2f} | "
            f"{float(row['delay_s']):.2f} | {float(row['packet_loss']):.2f} | {float(row['sensor_noise']):.2f} | "
            f"{int(row['runs'])} | {float(row['pose_rmse_mean']):.4f} | {float(row['pose_rmse_std']):.4f} | "
            f"{float(row['pose_rmse_ci95']):.4f} | {float(row['map_alignment_rmse_mean']):.4f} | "
            f"{float(row['fusion_time_ms_mean']):.2f} | {float(row['theta_mean']):.3f} | {float(row['theta_std']):.3f} |"
        )
    lines.append('')
    lines.append('Each row reports mean, sample standard deviation, and 95% confidence interval across repeated runs.')
    return '\n'.join(lines) + '\n'


def latex_table(rows: Sequence[dict], caption: str = 'EMRMF experiment summary') -> str:
    """Render a compact LaTeX table suitable for Springer revision drafts."""

    lines = [
        r'\begin{table}[t]',
        r'\centering',
        r'\scriptsize',
        r'\begin{tabular}{lrrrrrrrr}',
        r'\hline',
        r'Mode & $p$ & $\gamma$ & Delay & Loss & Runs & Pose RMSE & 95\% CI & $\bar{\theta}$ \\',
        r'\hline',
    ]
    latex_newline = r'\\'
    for row in rows:
        escaped_mode = row['ablation_mode'].replace('_', r'\_')
        lines.append(
            f"{escaped_mode} & {float(row['p']):.0f} & {float(row['gamma']):.2f} & "
            f"{float(row['delay_s']):.2f} & {float(row['packet_loss']):.2f} & {int(row['runs'])} & "
            f"{float(row['pose_rmse_mean']):.4f} & {float(row['pose_rmse_ci95']):.4f} & "
            f"{float(row['theta_mean']):.3f} {latex_newline}"
        )
    lines.extend([
        r'\hline',
        r'\end{tabular}',
        f'\\caption{{{caption}}}',
        r'\label{tab:emrmf_experiment_summary}',
        r'\end{table}',
        '',
    ])
    return '\n'.join(lines)


@dataclass(frozen=True)
class ExperimentConfiguration:
    """One orchestrated EMRMF experiment configuration."""

    ablation_mode: str
    p: float
    gamma: float
    delay_s: float
    packet_loss: float
    sensor_noise: float = 0.0
    tau_e: float = 0.5

    @property
    def configuration_id(self) -> str:
        return (
            f'{self.ablation_mode}_p{self.p:g}_gamma{self.gamma:g}_'
            f'delay{self.delay_s:g}_loss{self.packet_loss:g}_noise{self.sensor_noise:g}'
        ).replace('.', 'p')


def build_configurations(
    modes: Sequence[str] = ABLATION_MODES,
    p_values: Sequence[float] = DEFAULT_P_VALUES,
    gamma_values: Sequence[float] = DEFAULT_GAMMA_VALUES,
    delay_values: Sequence[float] = DEFAULT_DELAY_VALUES,
    packet_loss_values: Sequence[float] = DEFAULT_PACKET_LOSS_VALUES,
    sensor_noise_values: Sequence[float] = (0.0,),
    tau_e: float = 0.5,
) -> list[ExperimentConfiguration]:
    """Build the full validation sweep for trust/gamma/communication/ablation evidence."""

    return [
        ExperimentConfiguration(mode, float(p_value), float(gamma), float(delay), float(loss), float(noise), tau_e)
        for mode in modes
        for p_value in p_values
        for gamma in gamma_values
        for delay in delay_values
        for loss in packet_loss_values
        for noise in sensor_noise_values
    ]
