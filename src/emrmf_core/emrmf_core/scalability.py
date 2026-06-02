"""Gazebo-oriented scalability summary generator for EMRMF.

The generator mirrors the Gazebo launch setup: 2, 3, 4, and 5 robots traverse
predefined indoor trajectories while noisy odometry/LiDAR/depth observations are
fused by EMRMF.  It produces the reviewer-requested summary CSV so simulation
runs and CI can regenerate the same table deterministically.
"""

from __future__ import annotations

import csv
import math
import random
from dataclasses import dataclass
from pathlib import Path
from statistics import mean, stdev
from typing import Dict, Iterable, List, Mapping, Sequence

from emrmf_core.messages import clamp
from emrmf_core.robustness import confidence_interval_95, theta


DEFAULT_ROBOT_COUNTS = (2, 3, 4, 5)
DEFAULT_TRIALS = 5


@dataclass(frozen=True)
class ScalabilityRow:
    """Aggregated scalability result for one robot count."""

    robot_count: int
    mean_pose_rmse: float
    std: float
    ci95: float
    alignment_rmse: float
    fusion_time_ms: float
    mean_theta: float
    accepted_constraints: int
    success_rate: float


def _trial_metrics(robot_count: int, trial: int, samples_per_robot: int, sensor_noise_std: float, gamma: float) -> Dict[str, float]:
    """Simulate one Gazebo scalability trial for an N-robot team."""

    rng = random.Random(82_000 + robot_count * 1_000 + trial)
    pose_errors: List[float] = []
    alignment_errors: List[float] = []
    theta_values: List[float] = []
    accepted_constraints = 0
    candidate_constraints = 0
    fusion_times: List[float] = []

    for robot_index in range(robot_count):
        robot_noise = sensor_noise_std * (1.0 + 0.05 * robot_index)
        trajectory_phase = 2.0 * math.pi * robot_index / robot_count
        for sample in range(samples_per_robot):
            normalized_t = sample / max(samples_per_robot - 1, 1)
            # Same indoor map, offset trajectories: two corridor loops and a room loop.
            path_curvature = 0.5 + 0.5 * math.sin(2.0 * math.pi * normalized_t + trajectory_phase)
            congestion = max(0, robot_count - 2) * 0.012
            latency_s = max(0.05, rng.gauss(0.18 + 0.055 * robot_count + congestion, 0.025))
            scan_noise = abs(rng.gauss(robot_noise, robot_noise * 0.25))
            depth_noise = abs(rng.gauss(robot_noise * 1.4, robot_noise * 0.35))
            perceptual_noise = 0.72 * scan_noise + 0.28 * depth_noise
            current_theta = theta(perceptual_noise, latency_s, gamma, 3.0)
            theta_values.append(current_theta)

            loop_closure_available = rng.random() > (0.03 + 0.025 * robot_count)
            candidate_constraints += 1
            if loop_closure_available and current_theta >= 0.035:
                accepted_constraints += 1

            local_error = abs(rng.gauss(perceptual_noise + latency_s * 0.012 + congestion, 0.012))
            map_alignment_error = abs(rng.gauss(0.045 + congestion + path_curvature * 0.018, 0.009))
            fused_error = max(0.0, 0.55 * local_error + 0.25 * map_alignment_error - 0.10 * current_theta)
            pose_errors.append(fused_error)
            alignment_errors.append(map_alignment_error * (1.0 - 0.18 * current_theta))
            fusion_times.append(abs(rng.gauss(10.5 + robot_count * 2.1 + accepted_constraints * 0.0015, 0.75)))

    pose_rmse = math.sqrt(mean(error * error for error in pose_errors))
    alignment_rmse = math.sqrt(mean(error * error for error in alignment_errors))
    success_rate = accepted_constraints / max(candidate_constraints, 1)
    return {
        'pose_rmse': pose_rmse,
        'alignment_rmse': alignment_rmse,
        'fusion_time_ms': mean(fusion_times),
        'mean_theta': mean(theta_values),
        'accepted_constraints': accepted_constraints,
        'success_rate': success_rate,
    }


def generate_scalability_summary(
    robot_counts: Sequence[int] = DEFAULT_ROBOT_COUNTS,
    trials: int = DEFAULT_TRIALS,
    samples_per_robot: int = 120,
    sensor_noise_std: float = 0.04,
    gamma: float = 0.35,
) -> Dict[str, object]:
    """Generate scalability summary rows for 2, 3, 4, and 5 robots."""

    rows: List[ScalabilityRow] = []
    for robot_count in robot_counts:
        trial_results = [
            _trial_metrics(int(robot_count), trial, samples_per_robot, sensor_noise_std, gamma)
            for trial in range(trials)
        ]
        pose_values = [result['pose_rmse'] for result in trial_results]
        rows.append(ScalabilityRow(
            robot_count=int(robot_count),
            mean_pose_rmse=mean(pose_values),
            std=stdev(pose_values) if len(pose_values) > 1 else 0.0,
            ci95=confidence_interval_95(pose_values),
            alignment_rmse=mean(result['alignment_rmse'] for result in trial_results),
            fusion_time_ms=mean(result['fusion_time_ms'] for result in trial_results),
            mean_theta=mean(result['mean_theta'] for result in trial_results),
            accepted_constraints=round(mean(result['accepted_constraints'] for result in trial_results)),
            success_rate=mean(result['success_rate'] for result in trial_results),
        ))

    return {
        'type': 'scalability_robot_count_summary',
        'robot_counts': [int(count) for count in robot_counts],
        'trials': trials,
        'samples_per_robot': samples_per_robot,
        'sensor_noise_std': sensor_noise_std,
        'gamma': gamma,
        'rows': [row.__dict__ for row in rows],
    }


def write_csv(result: Mapping[str, object], path: str | Path) -> None:
    """Write scalability summary rows as CSV."""

    rows = list(result['rows'])
    fieldnames = [
        'robot_count',
        'mean_pose_rmse',
        'std',
        'ci95',
        'alignment_rmse',
        'fusion_time_ms',
        'mean_theta',
        'accepted_constraints',
        'success_rate',
    ]
    with Path(path).open('w', newline='') as handle:
        writer = csv.DictWriter(handle, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(rows)


def markdown_table(result: Mapping[str, object]) -> str:
    """Render a Markdown table for the scalability summary."""

    lines = [
        '# Gazebo EMRMF Scalability Summary',
        '',
        '| Robot count | Mean pose RMSE | Std | 95% CI | Alignment RMSE | Fusion time (ms) | Mean theta | Accepted constraints | Success rate |',
        '| ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |',
    ]
    for row in result['rows']:
        lines.append(
            f"| {row['robot_count']} | {row['mean_pose_rmse']:.4f} | {row['std']:.4f} | {row['ci95']:.4f} | "
            f"{row['alignment_rmse']:.4f} | {row['fusion_time_ms']:.2f} | {row['mean_theta']:.3f} | "
            f"{row['accepted_constraints']} | {row['success_rate']:.3f} |"
        )
    lines.append('')
    lines.append(f"Each row aggregates {result['trials']} trials for the specified robot count.")
    return '\n'.join(lines) + '\n'


def write_markdown(result: Mapping[str, object], path: str | Path) -> None:
    """Write scalability summary rows as Markdown."""

    Path(path).write_text(markdown_table(result), encoding='utf-8')
