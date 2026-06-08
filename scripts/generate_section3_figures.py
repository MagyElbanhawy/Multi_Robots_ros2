"""Generate EMRMF Section 3 conceptual figures.

Figures:
- Fig. 6: trust-weighted pose graph visualization
- Fig. 7: potential-field task allocation visualization
"""

from __future__ import annotations

import argparse
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
from matplotlib.collections import LineCollection
from matplotlib.colors import LinearSegmentedColormap, Normalize


TRUST_CMAP = LinearSegmentedColormap.from_list('trust', ['#dc2626', '#facc15', '#16a34a'])


def save_all(fig: plt.Figure, output_dir: Path, stem: str) -> None:
    output_dir.mkdir(parents=True, exist_ok=True)
    fig.savefig(output_dir / f'{stem}.png', dpi=300, bbox_inches='tight')
    fig.savefig(output_dir / f'{stem}.pdf', bbox_inches='tight')
    plt.close(fig)


def generate_pose_graph(output_dir: Path) -> None:
    rng = np.random.default_rng(7)
    robot_colors = {
        'robot_1': '#2563eb',
        'robot_2': '#d97706',
        'robot_3': '#7c3aed',
    }
    trajectories = {
        'robot_1': np.array([[0.0, 0.0], [0.9, 0.35], [1.8, 0.65], [2.7, 0.9], [3.6, 1.2], [4.5, 1.5]]),
        'robot_2': np.array([[0.2, 2.6], [1.0, 2.25], [1.8, 2.0], [2.7, 1.85], [3.7, 1.75], [4.6, 1.65]]),
        'robot_3': np.array([[0.4, -1.8], [1.0, -1.25], [1.6, -0.65], [2.35, -0.15], [3.2, 0.25], [4.0, 0.6]]),
    }
    trajectories = {name: points + rng.normal(0.0, 0.035, points.shape) for name, points in trajectories.items()}

    edges: list[tuple[np.ndarray, np.ndarray, float]] = []
    for points in trajectories.values():
        for start, end in zip(points[:-1], points[1:]):
            edges.append((start, end, 0.90))

    loop_edges = [
        ('robot_1', 2, 'robot_2', 2, 0.82),
        ('robot_1', 4, 'robot_2', 4, 0.91),
        ('robot_1', 1, 'robot_3', 2, 0.64),
        ('robot_1', 3, 'robot_3', 4, 0.76),
        ('robot_2', 1, 'robot_3', 3, 0.28),
        ('robot_2', 5, 'robot_3', 5, 0.42),
    ]
    for a_robot, a_idx, b_robot, b_idx, theta in loop_edges:
        edges.append((trajectories[a_robot][a_idx], trajectories[b_robot][b_idx], theta))

    fig, ax = plt.subplots(figsize=(8, 5))
    segments = [(start, end) for start, end, _ in edges]
    theta_values = np.array([theta for _, _, theta in edges])
    line_widths = 0.8 + 4.0 * theta_values
    collection = LineCollection(
        segments,
        cmap=TRUST_CMAP,
        norm=Normalize(vmin=0.0, vmax=1.0),
        linewidths=line_widths,
        alpha=0.88,
    )
    collection.set_array(theta_values)
    ax.add_collection(collection)

    for robot_id, points in trajectories.items():
        ax.scatter(
            points[:, 0],
            points[:, 1],
            s=90,
            color=robot_colors[robot_id],
            edgecolor='black',
            linewidth=0.8,
            label=robot_id,
            zorder=3,
        )
        for index, point in enumerate(points):
            ax.text(point[0], point[1] + 0.12, str(index + 1), ha='center', fontsize=8)

    cbar = fig.colorbar(collection, ax=ax, pad=0.02)
    cbar.set_label('Trust factor theta')
    ax.set_title('Trust-Weighted Multi-Robot Pose Graph', fontsize=14, weight='bold')
    ax.set_xlabel('x position (m)')
    ax.set_ylabel('y position (m)')
    ax.grid(True, alpha=0.25)
    ax.legend(loc='upper left', frameon=True)
    ax.set_aspect('equal', adjustable='box')
    ax.set_xlim(-0.4, 5.1)
    ax.set_ylim(-2.3, 3.1)
    save_all(fig, output_dir, 'figure_06_pose_graph_trust_visualization')


def _repulsive_force(robot: np.ndarray, obstacle: np.ndarray, radius: float, gain: float = 0.55) -> np.ndarray:
    delta = robot - obstacle
    distance = max(float(np.linalg.norm(delta)), 1.0e-6)
    influence = radius + 1.4
    if distance >= influence:
        return np.zeros(2)
    direction = delta / distance
    strength = gain * (1.0 / distance - 1.0 / influence) / (distance * distance)
    return strength * direction


def generate_potential_field(output_dir: Path) -> None:
    robots = np.array([
        [-2.6, -1.4],
        [-1.7, 1.3],
        [0.8, -1.2],
        [2.0, 1.0],
    ])
    goals = np.array([
        [2.8, 1.8],
        [2.4, -1.9],
        [-2.8, 1.9],
        [-0.4, 2.3],
    ])
    obstacles = [
        (np.array([-0.8, -0.1]), 0.55),
        (np.array([0.9, 0.5]), 0.50),
        (np.array([1.7, -0.9]), 0.42),
    ]

    attractive = 0.34 * (goals - robots)
    repulsive = np.zeros_like(robots)
    for index, robot in enumerate(robots):
        for obstacle, radius in obstacles:
            repulsive[index] += _repulsive_force(robot, obstacle, radius)
    resultant = attractive + repulsive

    fig, ax = plt.subplots(figsize=(8, 5))
    for obstacle, radius in obstacles:
        circle = plt.Circle(obstacle, radius, fc='#9ca3af', ec='#4b5563', alpha=0.55)
        ax.add_patch(circle)
    ax.scatter(robots[:, 0], robots[:, 1], s=95, color='#2563eb', edgecolor='black', label='Robots', zorder=4)
    ax.scatter(goals[:, 0], goals[:, 1], s=180, color='#16a34a', marker='*', edgecolor='black', label='Goals', zorder=4)

    ax.quiver(robots[:, 0], robots[:, 1], attractive[:, 0], attractive[:, 1], angles='xy', scale_units='xy', scale=1, color='#16a34a', width=0.007, label='F_att')
    ax.quiver(robots[:, 0], robots[:, 1], repulsive[:, 0], repulsive[:, 1], angles='xy', scale_units='xy', scale=1, color='#eab308', width=0.007, label='F_rep')
    ax.quiver(robots[:, 0], robots[:, 1], resultant[:, 0], resultant[:, 1], angles='xy', scale_units='xy', scale=1, color='#dc2626', width=0.009, label='F_res')

    for index, robot in enumerate(robots, start=1):
        ax.text(robot[0], robot[1] - 0.28, f'R{index}', ha='center', fontsize=9, weight='bold')
    for index, goal in enumerate(goals, start=1):
        ax.text(goal[0], goal[1] + 0.25, f'G{index}', ha='center', fontsize=9, weight='bold')

    ax.set_title('Potential-Field Task Allocation', fontsize=14, weight='bold')
    ax.set_xlabel('x position (m)')
    ax.set_ylabel('y position (m)')
    ax.set_xlim(-3.5, 3.5)
    ax.set_ylim(-2.7, 2.8)
    ax.set_aspect('equal', adjustable='box')
    ax.grid(True, alpha=0.25)
    ax.legend(loc='center left', bbox_to_anchor=(1.02, 0.5), frameon=True)
    save_all(fig, output_dir, 'figure_07_potential_field_task_allocation')


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', default='docs/images/section3_figures', type=Path)
    args = parser.parse_args()
    generate_pose_graph(args.output)
    generate_potential_field(args.output)


if __name__ == '__main__':
    main()
