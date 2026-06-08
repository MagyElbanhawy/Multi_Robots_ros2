"""Create the trust-factor evolution figure from logs/trust_factor.csv."""

from __future__ import annotations

import argparse
from pathlib import Path

import matplotlib.pyplot as plt
import pandas as pd


def plot_trust_factor(csv_path: Path, output_path: Path) -> None:
    df = pd.read_csv(csv_path)
    time_column = 'time' if 'time' in df.columns else 'timestamp'

    output_path.parent.mkdir(parents=True, exist_ok=True)
    plt.figure(figsize=(8, 4))
    plt.plot(df[time_column], df['theta'], linewidth=2)
    plt.xlabel('Time (s)')
    plt.ylabel('Trust Factor θ')
    plt.title('Trust Factor Evolution During Multi-Robot Mapping')
    plt.grid(True)
    plt.tight_layout()
    plt.savefig(output_path, dpi=300)
    plt.close()


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--csv', default='logs/trust_factor.csv', type=Path)
    parser.add_argument('--output', default='docs/images/trust_factor_visualization.png', type=Path)
    args = parser.parse_args()
    plot_trust_factor(args.csv, args.output)


if __name__ == '__main__':
    main()
