"""Deterministic robustness comparison utilities for trust-factor exponents.

The model intentionally separates three error sources that reviewers asked about:
packet loss, communication delay, and perceptual noise.  It is not a replacement
for physical experiments; it is a reproducible sensitivity harness that exposes
why a cubic trust curve can be preferable to a square or quartic curve.
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


DEFAULT_EXPONENTS = (2.0, 3.0, 4.0)
DEFAULT_PACKET_LOSS_LEVELS = (0.0, 0.1, 0.3)
DEFAULT_DELAY_LEVELS_S = (0.5, 2.0, 5.0)
DEFAULT_NOISE_LEVELS_STD = (0.03, 0.08, 0.16)


@dataclass(frozen=True)
class Scenario:
    """One robustness stress scenario."""

    stressor: str
    level: float
    packet_loss: float
    delay_s: float
    noise_std: float


@dataclass(frozen=True)
class RobustnessRow:
    """Aggregated metric row for one exponent in one scenario."""

    stressor: str
    level: float
    exponent: float
    runs: int
    mean_rmse_m: float
    ci95_rmse_m: float
    baseline_rmse_m: float
    improvement_vs_baseline_pct: float
    mean_theta: float
    mean_accepted_ratio: float
    rank: int


def confidence_interval_95(values: Iterable[float]) -> float:
    """Return the 95% confidence interval half-width for repeated runs."""

    samples = list(values)
    if len(samples) < 2:
        return 0.0
    return 1.96 * stdev(samples) / math.sqrt(len(samples))


def theta(noise: float, delay: float, gamma: float, exponent: float) -> float:
    """Trust factor used for exponent comparison."""

    reference_noise = 0.05
    reference_delay = 0.5
    sensor = math.pow(reference_noise / (reference_noise + max(noise, 0.0)), exponent)
    temporal = math.exp(-gamma * max(delay, 0.0)) * (reference_delay / (reference_delay + max(delay, 0.0)))
    return clamp(sensor * temporal, 0.02, 1.0)


def scenarios(
    packet_loss_levels: Sequence[float] = DEFAULT_PACKET_LOSS_LEVELS,
    delay_levels_s: Sequence[float] = DEFAULT_DELAY_LEVELS_S,
    noise_levels_std: Sequence[float] = DEFAULT_NOISE_LEVELS_STD,
) -> List[Scenario]:
    """Build isolated stress scenarios for packet loss, delay, and noise."""

    nominal_packet_loss = 0.0
    nominal_delay_s = 0.5
    nominal_noise_std = 0.05
    return [
        *[
            Scenario('packet_loss', float(level), float(level), nominal_delay_s, nominal_noise_std)
            for level in packet_loss_levels
        ],
        *[
            Scenario('delay', float(level), nominal_packet_loss, float(level), nominal_noise_std)
            for level in delay_levels_s
        ],
        *[
            Scenario('noise', float(level), nominal_packet_loss, nominal_delay_s, float(level))
            for level in noise_levels_std
        ],
    ]




def scenario_seed(scenario: Scenario) -> int:
    """Stable deterministic seed prefix for a stress scenario."""

    stressor_offsets = {'packet_loss': 100_000, 'delay': 200_000, 'noise': 300_000}
    return stressor_offsets.get(scenario.stressor, 400_000) + int(round(scenario.level * 10_000))

def simulate_run(scenario: Scenario, exponent: float, gamma: float, samples: int, seed: int) -> Dict[str, float]:
    """Simulate one repeated trial for a scenario/exponent pair.

    The estimator trades off map-update acceptance and rejection.  Lower exponents
    accept more data, which helps under nominal conditions but over-trusts delayed,
    noisy, or faulty packets.  Higher exponents reject more data, which protects
    against outliers but loses useful observations.  The optimal exponent is the
    one with the best average rank across the three stress families.
    """

    rng = random.Random(seed)
    fused_errors: List[float] = []
    baseline_errors: List[float] = []
    theta_values: List[float] = []
    accepted_ratios: List[float] = []

    for _ in range(samples):
        packet_dropped = rng.random() < scenario.packet_loss
        jittered_delay = max(0.0, rng.gauss(scenario.delay_s, max(0.01, scenario.delay_s * 0.12)))
        measured_noise = abs(rng.gauss(scenario.noise_std, max(0.005, scenario.noise_std * 0.25)))
        outlier_probability = clamp(0.04 + scenario.packet_loss * 0.35 + scenario.delay_s * 0.015 + scenario.noise_std)
        outlier_bias = rng.uniform(0.45, 1.2) if rng.random() < outlier_probability else 0.0

        measurement_error = abs(rng.gauss(measured_noise + jittered_delay * 0.035 + outlier_bias, 0.02))
        prediction_error = abs(rng.gauss(0.14 + scenario.packet_loss * 0.18 + jittered_delay * 0.015, 0.025))
        baseline_error = measurement_error if not packet_dropped else prediction_error + scenario.packet_loss * 0.25
        baseline_errors.append(baseline_error)

        current_theta = 0.0 if packet_dropped else theta(measured_noise, jittered_delay, gamma, exponent)
        theta_values.append(current_theta)
        accepted_ratio = clamp(current_theta / 0.35)
        accepted_ratios.append(accepted_ratio)

        # Weighted fusion error: rejecting everything falls back to prediction;
        # over-accepting corrupted observations adds a residual consistency penalty.
        overtrust_penalty = (current_theta ** 2) * (outlier_bias * 2.8 + measured_noise * 3.2 + jittered_delay * 0.08)
        underuse_penalty = math.pow(max(0.0, 0.055 - current_theta), 2.0) * 5.0
        fused_error = (
            current_theta * measurement_error
            + (1.0 - current_theta) * prediction_error
            + overtrust_penalty
            + underuse_penalty
        )
        fused_errors.append(fused_error)

    return {
        'rmse_m': mean(fused_errors),
        'baseline_rmse_m': mean(baseline_errors),
        'mean_theta': mean(theta_values),
        'mean_accepted_ratio': mean(accepted_ratios),
    }


def generate_robustness_comparison(
    runs: int = 5,
    samples_per_run: int = 200,
    gamma: float = 0.35,
    exponents: Sequence[float] = DEFAULT_EXPONENTS,
    packet_loss_levels: Sequence[float] = DEFAULT_PACKET_LOSS_LEVELS,
    delay_levels_s: Sequence[float] = DEFAULT_DELAY_LEVELS_S,
    noise_levels_std: Sequence[float] = DEFAULT_NOISE_LEVELS_STD,
) -> Dict[str, object]:
    """Generate p=2, p=3, p=4 robustness comparison under all stressors."""

    rows: List[RobustnessRow] = []
    for scenario in scenarios(packet_loss_levels, delay_levels_s, noise_levels_std):
        scenario_rows: List[RobustnessRow] = []
        for exponent in exponents:
            run_metrics = [
                simulate_run(scenario, float(exponent), gamma, samples_per_run, seed=scenario_seed(scenario) + run)
                for run in range(runs)
            ]
            rmse_values = [metric['rmse_m'] for metric in run_metrics]
            baseline_values = [metric['baseline_rmse_m'] for metric in run_metrics]
            baseline = mean(baseline_values)
            rmse = mean(rmse_values)
            improvement = 100.0 * (baseline - rmse) / baseline if baseline > 0.0 else 0.0
            scenario_rows.append(RobustnessRow(
                stressor=scenario.stressor,
                level=scenario.level,
                exponent=float(exponent),
                runs=runs,
                mean_rmse_m=rmse,
                ci95_rmse_m=confidence_interval_95(rmse_values),
                baseline_rmse_m=baseline,
                improvement_vs_baseline_pct=improvement,
                mean_theta=mean(metric['mean_theta'] for metric in run_metrics),
                mean_accepted_ratio=mean(metric['mean_accepted_ratio'] for metric in run_metrics),
                rank=0,
            ))
        ranked = sorted(scenario_rows, key=lambda row: row.mean_rmse_m)
        rank_by_exponent = {row.exponent: rank + 1 for rank, row in enumerate(ranked)}
        rows.extend([
            RobustnessRow(**{**row.__dict__, 'rank': rank_by_exponent[row.exponent]})
            for row in scenario_rows
        ])

    mean_rank_by_exponent = {
        float(exponent): mean(row.rank for row in rows if row.exponent == float(exponent))
        for exponent in exponents
    }
    mean_rmse_by_exponent = {
        float(exponent): mean(row.mean_rmse_m for row in rows if row.exponent == float(exponent))
        for exponent in exponents
    }
    winner = min(mean_rank_by_exponent, key=lambda exp: (mean_rank_by_exponent[exp], mean_rmse_by_exponent[exp]))
    return {
        'type': 'robustness_comparison',
        'exponents': [float(exponent) for exponent in exponents],
        'stressors': ['packet_loss', 'delay', 'noise'],
        'gamma': gamma,
        'runs': runs,
        'samples_per_run': samples_per_run,
        'rows': [row.__dict__ for row in rows],
        'mean_rank_by_exponent': mean_rank_by_exponent,
        'mean_rmse_by_exponent': mean_rmse_by_exponent,
        'recommended_exponent': winner,
        'recommendation': (
            f'p={winner:g} has the lowest average robustness rank across packet loss, delay, and noise; '
            'this supports retaining the cubic formulation when the winner is p=3.'
        ),
    }


def write_csv(result: Mapping[str, object], path: str | Path) -> None:
    """Write robustness rows as CSV."""

    rows = list(result['rows'])
    fieldnames = list(rows[0].keys()) if rows else []
    with Path(path).open('w', newline='') as handle:
        writer = csv.DictWriter(handle, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(rows)


def markdown_table(result: Mapping[str, object]) -> str:
    """Render a compact reviewer-facing Markdown table."""

    lines = [
        '# Robustness Comparison for Trust Exponent p',
        '',
        f"Recommended exponent: **p={result['recommended_exponent']:g}**.",
        '',
        '| Stressor | Level | p | RMSE (m) | 95% CI | Improvement vs baseline | Mean theta | Rank |',
        '| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: |',
    ]
    for row in result['rows']:
        lines.append(
            f"| {row['stressor']} | {row['level']:.3g} | {row['exponent']:.0f} | "
            f"{row['mean_rmse_m']:.4f} | {row['ci95_rmse_m']:.4f} | "
            f"{row['improvement_vs_baseline_pct']:.1f}% | {row['mean_theta']:.3f} | {row['rank']} |"
        )
    lines.extend([
        '',
        'Mean rank by exponent:',
        *[
            f"- p={exponent:g}: {rank:.2f}"
            for exponent, rank in sorted(result['mean_rank_by_exponent'].items())
        ],
        '',
        str(result['recommendation']),
    ])
    return '\n'.join(lines) + '\n'


def write_markdown(result: Mapping[str, object], path: str | Path) -> None:
    """Write a Markdown robustness report."""

    Path(path).write_text(markdown_table(result), encoding='utf-8')
