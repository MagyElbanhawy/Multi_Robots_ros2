"""Generate statistical validation tables for EMRMF ablation results."""

from __future__ import annotations

import argparse
import csv
import itertools
import math
from collections import defaultdict
from pathlib import Path
from statistics import mean, stdev


METHOD_LABELS = {
    'baseline_graph_slam': 'Baseline Graph SLAM',
    'decentralized_only': 'Decentralized Only',
    'trust_only': 'Trust Only',
    'full_emrmf': 'Full EMRMF',
}


def confidence_interval_95(values: list[float]) -> float:
    if len(values) <= 1:
        return 0.0
    return 1.96 * stdev(values) / math.sqrt(len(values))


def signed_rank_test(baseline: list[float], treatment: list[float]) -> dict[str, float]:
    """Exact two-sided Wilcoxon signed-rank test for paired samples.

    Returns the smaller rank sum W and an exact p-value by enumerating all sign
    assignments. This avoids requiring scipy for this small validation table.
    """

    differences = [b - t for b, t in zip(baseline, treatment) if abs(b - t) > 1.0e-12]
    absolute = sorted((abs(value), index, value) for index, value in enumerate(differences))
    ranks = [0.0] * len(differences)
    rank = 1
    for _, group in itertools.groupby(absolute, key=lambda item: item[0]):
        items = list(group)
        average_rank = sum(range(rank, rank + len(items))) / len(items)
        for _, index, _ in items:
            ranks[index] = average_rank
        rank += len(items)

    positive = sum(r for r, d in zip(ranks, differences) if d > 0)
    negative = sum(r for r, d in zip(ranks, differences) if d < 0)
    observed = min(positive, negative)
    total_rank = sum(ranks)

    if len(ranks) <= 25:
        distribution = [0.0]
        for value in ranks:
            distribution += [current + value for current in distribution]
        extreme = sum(1 for value in distribution if min(value, total_rank - value) <= observed + 1.0e-12)
        p_value = extreme / len(distribution)
    else:
        expected = len(ranks) * (len(ranks) + 1) / 4.0
        variance = len(ranks) * (len(ranks) + 1) * (2 * len(ranks) + 1) / 24.0
        z_score = (observed - expected + 0.5) / math.sqrt(variance)
        p_value = math.erfc(abs(z_score) / math.sqrt(2.0))
    return {'n_pairs': len(differences), 'w_statistic': observed, 'p_value': p_value}


def load_raw_rows(path: Path) -> list[dict[str, str]]:
    with path.open(newline='', encoding='utf-8') as handle:
        return list(csv.DictReader(handle))


def method_summary(path: Path) -> list[dict[str, float | str]]:
    with path.open(newline='', encoding='utf-8') as handle:
        summary_rows = list(csv.DictReader(handle))
    by_method = {row['method']: row for row in summary_rows}
    summaries = []
    for method in ['baseline_graph_slam', 'decentralized_only', 'trust_only', 'full_emrmf']:
        row = by_method[method]
        summaries.append({
            'method': METHOD_LABELS[method],
            'runs': int(row['runs']),
            'samples_per_run': int(row['samples_per_run']),
            'mean_rmse_m': float(row['mean_rmse_m']),
            'std_dev_m': float(row['std_rmse_m']),
            'ci95_m': float(row['ci95_rmse_m']),
            'improvement_vs_baseline_pct': float(row['improvement_vs_baseline_pct']),
        })
    return summaries


def paired_baseline_full(rows: list[dict[str, str]]) -> tuple[list[float], list[float]]:
    paired: dict[tuple[str, str, str, str, str, str], dict[str, float]] = defaultdict(dict)
    for row in rows:
        key = (
            row['p'],
            row['gamma'],
            row['delay_s'],
            row['packet_loss'],
            row['sensor_noise'],
            row['run_id'],
        )
        if row['ablation_mode'] in {'baseline_graph_slam', 'full_emrmf'}:
            paired[key][row['ablation_mode']] = float(row['pose_rmse'])

    baseline = []
    full = []
    for values in paired.values():
        if 'baseline_graph_slam' in values and 'full_emrmf' in values:
            baseline.append(values['baseline_graph_slam'])
            full.append(values['full_emrmf'])
    return baseline, full


def write_csv(rows: list[dict[str, float | str]], path: Path) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open('w', newline='', encoding='utf-8') as handle:
        writer = csv.DictWriter(handle, fieldnames=list(rows[0].keys()))
        writer.writeheader()
        writer.writerows(rows)


def write_markdown(rows: list[dict[str, float | str]], test: dict[str, float], path: Path) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    lines = [
        '# Statistical Validation',
        '',
        '| Method | Runs | Samples/run | Mean RMSE (m) | Std Dev (m) | 95% CI (m) | Improvement vs baseline |',
        '| --- | ---: | ---: | ---: | ---: | ---: | ---: |',
    ]
    for row in rows:
        lines.append(
            f"| {row['method']} | {int(row['runs'])} | {int(row['samples_per_run'])} | "
            f"{float(row['mean_rmse_m']):.3f} | {float(row['std_dev_m']):.3f} | "
            f"+/-{float(row['ci95_m']):.3f} | {float(row['improvement_vs_baseline_pct']):.1f}% |"
        )
    p_text = '< 0.001' if test['p_value'] < 0.001 else f"= {test['p_value']:.4f}"
    lines.extend([
        '',
        'Paired Wilcoxon signed-rank test comparing Baseline Graph SLAM and Full EMRMF across matched repeated configurations:',
        '',
        f"- Matched pairs: {int(test['n_pairs'])}",
        f"- W = {test['w_statistic']:.0f}",
        f"- p {p_text}",
    ])
    path.write_text('\n'.join(lines) + '\n', encoding='utf-8')


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--raw', default='docs/emrmf_experiment_logs/emrmf_raw_results_20260604_105425.csv', type=Path)
    parser.add_argument('--summary', default='docs/final_ablation_summary.csv', type=Path)
    parser.add_argument('--csv-out', default='docs/statistical_validation.csv', type=Path)
    parser.add_argument('--markdown-out', default='docs/statistical_validation.md', type=Path)
    args = parser.parse_args()

    rows = load_raw_rows(args.raw)
    summaries = method_summary(args.summary)
    baseline, full = paired_baseline_full(rows)
    test = signed_rank_test(baseline, full)
    write_csv(summaries, args.csv_out)
    write_markdown(summaries, test, args.markdown_out)


if __name__ == '__main__':
    main()
