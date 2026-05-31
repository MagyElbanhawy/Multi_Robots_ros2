#!/usr/bin/env python3
"""Generate scalability_robot_count_summary.csv for Gazebo EMRMF robot counts."""

from __future__ import annotations

import argparse
import json
from pathlib import Path
import sys

REPO_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO_ROOT / 'src' / 'emrmf_core'))

from emrmf_core.scalability import generate_scalability_summary, markdown_table, write_csv, write_markdown  # noqa: E402


def parse_int_list(value: str) -> list[int]:
    return [int(item.strip()) for item in value.split(',') if item.strip()]


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--robot-counts', default='2,3,4,5')
    parser.add_argument('--trials', type=int, default=5)
    parser.add_argument('--samples-per-robot', type=int, default=120)
    parser.add_argument('--sensor-noise-std', type=float, default=0.04)
    parser.add_argument('--gamma', type=float, default=0.35)
    parser.add_argument('--csv-out', type=Path, default=Path('docs/scalability_robot_count_summary.csv'))
    parser.add_argument('--markdown-out', type=Path, default=Path('docs/scalability_robot_count_summary.md'))
    parser.add_argument('--json', action='store_true', help='Print JSON instead of Markdown.')
    args = parser.parse_args()

    result = generate_scalability_summary(
        robot_counts=parse_int_list(args.robot_counts),
        trials=args.trials,
        samples_per_robot=args.samples_per_robot,
        sensor_noise_std=args.sensor_noise_std,
        gamma=args.gamma,
    )
    if args.csv_out:
        write_csv(result, args.csv_out)
    if args.markdown_out:
        write_markdown(result, args.markdown_out)
    if args.json:
        print(json.dumps(result, indent=2, sort_keys=True))
    else:
        print(markdown_table(result))


if __name__ == '__main__':
    main()
