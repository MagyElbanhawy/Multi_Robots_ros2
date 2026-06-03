#!/usr/bin/env python3
"""Generate reviewer-ready EMRMF experiment validation artifacts."""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'src' / 'emrmf_experiments'))

from emrmf_experiments.validation_simulator import generate_validation_artifacts  # noqa: E402


def parse_csv_floats(value: str) -> list[float]:
    return [float(item.strip()) for item in value.split(',') if item.strip()]


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output-dir', type=Path, default=Path('docs/emrmf_experiment_logs'))
    parser.add_argument('--p-values', default='2,3,4')
    parser.add_argument('--gamma-values', default='0.1,0.3,0.5,1.0')
    parser.add_argument('--delay-values', default='0.0,0.5,2.0')
    parser.add_argument('--packet-loss-values', default='0.0,0.10,0.30')
    parser.add_argument('--sensor-noise-values', default='0.0')
    parser.add_argument('--repeated-runs', type=int, default=5)
    parser.add_argument('--tau-e', type=float, default=0.5)
    parser.add_argument('--seed', type=int, default=2026)
    args = parser.parse_args()
    result = generate_validation_artifacts(
        output_dir=args.output_dir,
        p_values=parse_csv_floats(args.p_values),
        gamma_values=parse_csv_floats(args.gamma_values),
        delay_values=parse_csv_floats(args.delay_values),
        packet_loss_values=parse_csv_floats(args.packet_loss_values),
        sensor_noise_values=parse_csv_floats(args.sensor_noise_values),
        repeated_runs=args.repeated_runs,
        tau_e=args.tau_e,
        seed=args.seed,
    )
    print(f"Raw CSV: {result['raw_csv']}")
    print(f"Summary CSV: {result['summary_csv']}")
    print(f"Markdown: {result['summary_markdown']}")
    print(f"LaTeX: {result['summary_latex']}")


if __name__ == '__main__':
    main()
