#!/usr/bin/env python3
"""Generate reviewer-facing robustness tables for p=2, p=3, and p=4."""

from __future__ import annotations

import argparse
import json
from pathlib import Path
import sys

REPO_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO_ROOT / 'src' / 'emrmf_core'))

from emrmf_core.robustness import generate_robustness_comparison, markdown_table, write_csv, write_markdown  # noqa: E402


def parse_float_list(value: str) -> list[float]:
    return [float(item.strip()) for item in value.split(',') if item.strip()]


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--runs', type=int, default=5)
    parser.add_argument('--samples-per-run', type=int, default=200)
    parser.add_argument('--gamma', type=float, default=0.35)
    parser.add_argument('--exponents', default='2,3,4')
    parser.add_argument('--packet-loss', default='0.0,0.1,0.3')
    parser.add_argument('--delay', default='0.5,2.0,5.0')
    parser.add_argument('--noise', default='0.03,0.08,0.16')
    parser.add_argument('--markdown-out', type=Path)
    parser.add_argument('--csv-out', type=Path)
    parser.add_argument('--json', action='store_true', help='Print JSON instead of Markdown.')
    args = parser.parse_args()

    result = generate_robustness_comparison(
        runs=args.runs,
        samples_per_run=args.samples_per_run,
        gamma=args.gamma,
        exponents=parse_float_list(args.exponents),
        packet_loss_levels=parse_float_list(args.packet_loss),
        delay_levels_s=parse_float_list(args.delay),
        noise_levels_std=parse_float_list(args.noise),
    )
    if args.markdown_out:
        write_markdown(result, args.markdown_out)
    if args.csv_out:
        write_csv(result, args.csv_out)
    if args.json:
        print(json.dumps(result, indent=2, sort_keys=True))
    else:
        print(markdown_table(result))


if __name__ == '__main__':
    main()
