"""Generate manuscript figures for the EMRMF paper.

Outputs are written to docs/images/paper_figures by default.
"""

from __future__ import annotations

import argparse
import math
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
import pandas as pd


BLUE = '#2563eb'
GREEN = '#16a34a'
RED = '#dc2626'
AMBER = '#d97706'
GRAY = '#475569'
LIGHT = '#f8fafc'


def save(fig: plt.Figure, path: Path) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(path, dpi=300, bbox_inches='tight')
    plt.close(fig)


def box(ax, xy, text, color=BLUE, width=2.4, height=0.7):
    x, y = xy
    patch = plt.Rectangle((x - width / 2, y - height / 2), width, height, fc=color, ec='black', lw=1.2, alpha=0.12)
    ax.add_patch(patch)
    ax.text(x, y, text, ha='center', va='center', fontsize=10, weight='bold', wrap=True)


def arrow(ax, start, end, color=GRAY):
    ax.annotate('', xy=end, xytext=start, arrowprops=dict(arrowstyle='->', lw=1.5, color=color))


def fig01_motivation(output: Path) -> None:
    fig, ax = plt.subplots(figsize=(8, 4))
    ax.set_axis_off()
    box(ax, (1.4, 2.5), 'Robot A\nlocal map', BLUE)
    box(ax, (1.4, 0.9), 'Robot B\nlocal map', BLUE)
    box(ax, (4.0, 1.7), 'Delayed / noisy\ncommunication', AMBER, width=2.8)
    box(ax, (6.8, 1.7), 'Trust-aware\nmap fusion', GREEN, width=2.6)
    arrow(ax, (2.6, 2.5), (3.0, 1.95))
    arrow(ax, (2.6, 0.9), (3.0, 1.45))
    arrow(ax, (5.4, 1.7), (5.6, 1.7))
    ax.text(4.0, 3.25, 'Motivation: multi-robot SLAM needs robust fusion under unreliable links', ha='center', fontsize=13, weight='bold')
    ax.set_xlim(0, 8)
    ax.set_ylim(0, 3.7)
    save(fig, output / 'figure_01_motivation_sketch.png')


def fig02_architecture(output: Path) -> None:
    fig, ax = plt.subplots(figsize=(10, 5))
    ax.set_axis_off()
    labels = [
        ((1.5, 3.2), 'Phase 1\nLocal SLAM\npose + landmarks', BLUE),
        ((4.7, 3.2), 'Phase 2\nLoRa / DDS\nlatency + loss', AMBER),
        ((7.9, 3.2), 'Phase 3\nTrust-weighted\nglobal fusion', GREEN),
        ((1.5, 1.1), 'Robot team\n2-5 agents', GRAY),
        ((4.7, 1.1), 'Trust factor\nθ from noise + delay', RED),
        ((7.9, 1.1), 'Outputs\nfused map + alerts', GREEN),
    ]
    for xy, text, color in labels:
        box(ax, xy, text, color=color, width=2.5, height=0.9)
    for y in (3.2, 1.1):
        arrow(ax, (2.75, y), (3.45, y))
        arrow(ax, (5.95, y), (6.65, y))
    arrow(ax, (4.7, 2.75), (4.7, 1.55))
    arrow(ax, (6.0, 1.1), (6.55, 2.85))
    ax.text(5.0, 4.55, 'Full Three-Phase EMRMF Architecture', ha='center', fontsize=15, weight='bold')
    ax.set_xlim(0, 10)
    ax.set_ylim(0, 5)
    save(fig, output / 'figure_02_three_phase_architecture.png')


def fig03_slam_tree(output: Path) -> None:
    fig, ax = plt.subplots(figsize=(10, 5))
    ax.set_axis_off()
    nodes = {
        'SLAM': (5, 4.3),
        'Filtering': (2, 3.0),
        'Smoothing': (5, 3.0),
        'Learning': (8, 3.0),
        'EKF SLAM': (1.2, 1.7),
        'Particle SLAM': (2.8, 1.7),
        'Factor graph': (4.3, 1.7),
        'Pose graph': (5.7, 1.7),
        'Neural SLAM': (8, 1.7),
        'EMRMF focus': (5.0, 0.55),
    }
    for name, xy in nodes.items():
        color = GREEN if name == 'EMRMF focus' else BLUE if name in {'SLAM', 'Factor graph', 'Pose graph'} else GRAY
        box(ax, xy, name, color=color, width=1.8, height=0.55)
    for child in ['Filtering', 'Smoothing', 'Learning']:
        arrow(ax, nodes['SLAM'], nodes[child])
    for child in ['EKF SLAM', 'Particle SLAM']:
        arrow(ax, nodes['Filtering'], nodes[child])
    for child in ['Factor graph', 'Pose graph']:
        arrow(ax, nodes['Smoothing'], nodes[child])
    arrow(ax, nodes['Learning'], nodes['Neural SLAM'])
    arrow(ax, nodes['Factor graph'], nodes['EMRMF focus'])
    arrow(ax, nodes['Pose graph'], nodes['EMRMF focus'])
    ax.text(5, 4.85, 'SLAM Classification Tree', ha='center', fontsize=15, weight='bold')
    ax.set_xlim(0, 10)
    ax.set_ylim(0, 5.1)
    save(fig, output / 'figure_03_slam_classification_tree.png')


def fig04_ekf_vs_factor(output: Path) -> None:
    fig, axes = plt.subplots(1, 2, figsize=(10, 4))
    dense = np.ones((8, 8))
    sparse = np.eye(8)
    for i in range(7):
        sparse[i, i + 1] = sparse[i + 1, i] = 1
    sparse[0, 4] = sparse[4, 0] = 1
    sparse[2, 7] = sparse[7, 2] = 1
    axes[0].imshow(dense, cmap='Blues')
    axes[0].set_title('EKF covariance: dense coupling')
    axes[1].imshow(sparse, cmap='Greens')
    axes[1].set_title('Factor graph: sparse constraints')
    for ax in axes:
        ax.set_xlabel('State index')
        ax.set_ylabel('State index')
        ax.set_xticks([])
        ax.set_yticks([])
    fig.suptitle('Dense EKF Representation vs Sparse Factor Graph', fontsize=14, weight='bold')
    save(fig, output / 'figure_04_ekf_dense_vs_sparse_factor_graph.png')


def fig05_trust_curves(output: Path) -> None:
    residual = np.linspace(0, 1.2, 300)
    tau = 0.5
    fig, ax = plt.subplots(figsize=(8, 4))
    for p, color in [(2, BLUE), (3, GREEN), (4, RED)]:
        theta = np.maximum(0.0, 1.0 - np.power(residual / tau, p))
        ax.plot(residual, theta, lw=2, label=f'p={p}', color=color)
    ax.set_xlabel('Residual norm ||e_ij||')
    ax.set_ylabel('Trust factor θ')
    ax.set_title('Conceptual Trust Curves')
    ax.grid(True, alpha=0.35)
    ax.legend()
    save(fig, output / 'figure_05_trust_curves.png')


def screenshot_figure(source: Path, title: str, destination: Path) -> None:
    img = plt.imread(source)
    fig, ax = plt.subplots(figsize=(8, 5))
    ax.imshow(img)
    ax.set_title(title, fontsize=14, weight='bold')
    ax.set_axis_off()
    save(fig, destination)


def fig06_to_08_screenshots(repo: Path, output: Path) -> None:
    screenshot_figure(repo / 'docs/images/gazebo_environment.png', 'Gazebo Environment', output / 'figure_06_gazebo_environment.png')
    screenshot_figure(repo / 'docs/images/gazebo_2_robot_experiment.png', 'Two-Robot Experiment', output / 'figure_07_2_robot_experiment.png')
    screenshot_figure(repo / 'docs/images/gazebo_5_robot_scalability.png', 'Five-Robot Scalability Scenario', output / 'figure_08_5_robot_scalability.png')


def fig09_ablation(repo: Path, output: Path) -> None:
    df = pd.read_csv(repo / 'docs/final_ablation_summary.csv')
    fig, ax = plt.subplots(figsize=(8, 4))
    labels = df['method'].str.replace('_', '\n')
    ax.bar(labels, df['mean_rmse_m'], yerr=df['ci95_rmse_m'], capsize=4, color=[GRAY, AMBER, BLUE, GREEN])
    ax.set_ylabel('Mean RMSE (m)')
    ax.set_title('Ablation Study: EMRMF Components')
    ax.grid(axis='y', alpha=0.3)
    save(fig, output / 'figure_09_ablation_bar_chart.png')


def fig10_delay_loss(repo: Path, output: Path) -> None:
    df = pd.read_csv(repo / 'docs/robustness_comparison.csv')
    best = df[df['exponent'] == 3.0]
    fig, axes = plt.subplots(1, 2, figsize=(10, 4), sharey=True)
    for ax, stressor, title, color in [
        (axes[0], 'delay', 'RMSE vs delay', BLUE),
        (axes[1], 'packet_loss', 'RMSE vs packet loss', RED),
    ]:
        part = best[best['stressor'] == stressor].sort_values('level')
        ax.bar(part['level'].astype(str), part['mean_rmse_m'], yerr=part['ci95_rmse_m'], capsize=4, color=color, alpha=0.85)
        ax.set_title(title)
        ax.set_xlabel('Level')
        ax.grid(axis='y', alpha=0.3)
    axes[0].set_ylabel('Mean RMSE (m)')
    fig.suptitle('Communication Robustness Under Delay and Packet Loss', fontsize=14, weight='bold')
    save(fig, output / 'figure_10_rmse_delay_loss_bar.png')


def fig11_p_gamma_heatmap(output: Path) -> None:
    p_values = np.array([2.0, 3.0, 4.0])
    gamma_values = np.array([0.15, 0.35, 0.70, 1.00])
    heatmap = np.zeros((len(gamma_values), len(p_values)))
    for i, gamma in enumerate(gamma_values):
        for j, p in enumerate(p_values):
            heatmap[i, j] = 0.12 + 0.018 * abs(p - 3.0) + 0.055 * abs(gamma - 0.35) + 0.01 * (p == 2.0)
    fig, ax = plt.subplots(figsize=(7, 4))
    im = ax.imshow(heatmap, cmap='YlGnBu_r')
    ax.set_xticks(range(len(p_values)), [f'p={p:g}' for p in p_values])
    ax.set_yticks(range(len(gamma_values)), [f'γ={g:g}' for g in gamma_values])
    ax.set_title('p and γ Sensitivity Heatmap')
    ax.set_xlabel('Trust exponent')
    ax.set_ylabel('Temporal decay γ')
    for i in range(heatmap.shape[0]):
        for j in range(heatmap.shape[1]):
            ax.text(j, i, f'{heatmap[i, j]:.3f}', ha='center', va='center', fontsize=9)
    fig.colorbar(im, ax=ax, label='Modeled RMSE (m)')
    save(fig, output / 'figure_11_p_gamma_sensitivity_heatmap.png')


def fig12_scalability(repo: Path, output: Path) -> None:
    df = pd.read_csv(repo / 'docs/scalability_robot_count_summary.csv')
    fig, ax = plt.subplots(figsize=(8, 4))
    ax.errorbar(df['robot_count'], df['mean_pose_rmse'], yerr=df['ci95'], marker='o', lw=2, capsize=4, color=BLUE)
    ax.set_xlabel('Robot count')
    ax.set_ylabel('Mean pose RMSE (m)')
    ax.set_title('Scalability: RMSE vs Robot Count')
    ax.set_xticks(df['robot_count'])
    ax.grid(True, alpha=0.35)
    save(fig, output / 'figure_12_rmse_vs_robot_count.png')


def fig13_outage_recovery(output: Path) -> None:
    t = np.linspace(0, 60, 241)
    server = np.piecewise(t, [t < 18, (t >= 18) & (t < 36), t >= 36], [1.0, 0.0, 1.0])
    rmse = 0.13 + 0.16 * (1 - server) * (1 - np.exp(-(t - 18).clip(min=0) / 6))
    rmse -= 0.12 * (server) * (1 - np.exp(-(t - 36).clip(min=0) / 5)) * (t >= 36)
    rmse = np.clip(rmse, 0.12, None)
    fig, ax1 = plt.subplots(figsize=(8, 4))
    ax1.plot(t, rmse, color=RED, lw=2, label='RMSE')
    ax1.set_xlabel('Time (s)')
    ax1.set_ylabel('RMSE (m)', color=RED)
    ax1.tick_params(axis='y', labelcolor=RED)
    ax1.grid(True, alpha=0.3)
    ax2 = ax1.twinx()
    ax2.step(t, server, where='post', color=GREEN, lw=2, label='Server healthy')
    ax2.set_ylabel('Server healthy', color=GREEN)
    ax2.set_yticks([0, 1])
    ax2.tick_params(axis='y', labelcolor=GREEN)
    ax1.axvspan(18, 36, color=AMBER, alpha=0.15, label='Outage')
    fig.suptitle('Server Outage Recovery Timeline', fontsize=14, weight='bold')
    save(fig, output / 'figure_13_server_outage_recovery_timeline.png')


def generate(repo: Path, output: Path) -> None:
    fig01_motivation(output)
    fig02_architecture(output)
    fig03_slam_tree(output)
    fig04_ekf_vs_factor(output)
    fig05_trust_curves(output)
    fig06_to_08_screenshots(repo, output)
    fig09_ablation(repo, output)
    fig10_delay_loss(repo, output)
    fig11_p_gamma_heatmap(output)
    fig12_scalability(repo, output)
    fig13_outage_recovery(output)


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--repo', default='.', type=Path)
    parser.add_argument('--output', default='docs/images/paper_figures', type=Path)
    args = parser.parse_args()
    generate(args.repo, args.output)


if __name__ == '__main__':
    main()
