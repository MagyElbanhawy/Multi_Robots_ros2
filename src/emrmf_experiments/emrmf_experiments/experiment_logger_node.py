"""Live ROS 2 EMRMF experiment logger for reviewer-ready validation CSVs."""

from __future__ import annotations

import csv
import json
import math
from datetime import datetime, timezone
from pathlib import Path
from statistics import mean, stdev
from typing import Sequence

import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Bool, Float64, Float64MultiArray

from emrmf_experiments.metrics import (
    RAW_FIELDNAMES,
    SUMMARY_FIELDNAMES,
    alignment_rmse,
    compute_theta,
    euclidean_distance,
    latex_table,
    markdown_table,
    pose_rmse,
    summarize_raw_rows,
    write_csv,
)

DEFAULT_ESTIMATED_TOPICS = [
    {'robot_id': 'robot1', 'topic': '/robot1/odom', 'type': 'Odometry'},
    {'robot_id': 'robot2', 'topic': '/robot2/odom', 'type': 'Odometry'},
]
DEFAULT_GROUND_TRUTH_TOPICS = [
    {'robot_id': 'robot1', 'topic': '/robot1/ground_truth', 'type': 'Odometry'},
    {'robot_id': 'robot2', 'topic': '/robot2/ground_truth', 'type': 'Odometry'},
]


def pose_from_odometry(message: Odometry) -> tuple[float, float, float]:
    """Extract x, y, z pose coordinates from nav_msgs/Odometry."""

    position = message.pose.pose.position
    return (float(position.x), float(position.y), float(position.z))


def pose_from_stamped(message: PoseStamped) -> tuple[float, float, float]:
    """Extract x, y, z pose coordinates from geometry_msgs/PoseStamped."""

    position = message.pose.position
    return (float(position.x), float(position.y), float(position.z))


def stamp_to_seconds(message) -> float:
    """Return ROS message header stamp as seconds when available."""

    stamp = message.header.stamp
    return float(stamp.sec) + float(stamp.nanosec) / 1.0e9


class ExperimentLoggerNode(Node):
    """Collect live EMRMF metrics and write raw plus summary artifacts."""

    def __init__(self) -> None:
        super().__init__('emrmf_experiment_logger')
        self.declare_parameter('output_dir', 'docs/emrmf_experiment_logs')
        self.declare_parameter('ablation_mode', 'full_emrmf')
        self.declare_parameter('p', 3.0)
        self.declare_parameter('gamma', 0.3)
        self.declare_parameter('tau_e', 0.5)
        self.declare_parameter('delay_s', 0.0)
        self.declare_parameter('packet_loss', 0.0)
        self.declare_parameter('sensor_noise', 0.0)
        self.declare_parameter('run_id', 1)
        self.declare_parameter('configuration_id', 'manual_live')
        self.declare_parameter('estimated_pose_topics_json', json.dumps(DEFAULT_ESTIMATED_TOPICS))
        self.declare_parameter('ground_truth_topics_json', json.dumps(DEFAULT_GROUND_TRUTH_TOPICS))
        self.declare_parameter('reference_trajectory_csv', '')
        self.declare_parameter('source_matched_points_topic', '/map_fusion/source_matched_points')
        self.declare_parameter('target_matched_points_topic', '/map_fusion/target_matched_points')
        self.declare_parameter('source_transform_xyz_yaw', [0.0, 0.0, 0.0, 0.0])
        self.declare_parameter('constraint_error_topic', '/map_fusion/constraint_error_norm')
        self.declare_parameter('observed_relative_transform_topic', '/map_fusion/observed_relative_transform')
        self.declare_parameter('predicted_relative_transform_topic', '/map_fusion/predicted_relative_transform')
        self.declare_parameter('fusion_time_topic', '/map_fusion/fusion_time_ms')
        self.declare_parameter('summary_period_s', 5.0)
        self.started_at = datetime.now(timezone.utc).strftime('%Y%m%d_%H%M%S')
        self.output_dir = Path(str(self.get_parameter('output_dir').value))
        self.output_dir.mkdir(parents=True, exist_ok=True)
        self.raw_csv = self.output_dir / f'emrmf_live_raw_{self.started_at}.csv'
        self.summary_csv = self.output_dir / 'emrmf_live_summary.csv'
        self.summary_md = self.output_dir / 'emrmf_live_summary.md'
        self.summary_tex = self.output_dir / 'emrmf_live_summary.tex'
        self.estimated_history: dict[str, list[tuple[float, tuple[float, float, float]]]] = {}
        self.ground_truth_history: dict[str, list[tuple[float, tuple[float, float, float]]]] = {}
        self.reference_history = self._load_reference_trajectory(str(self.get_parameter('reference_trajectory_csv').value))
        self.source_points: list[tuple[float, float, float]] = []
        self.target_points: list[tuple[float, float, float]] = []
        self.constraint_errors: list[float] = []
        self.latest_observed_relative_transform: list[float] | None = None
        self.latest_predicted_relative_transform: list[float] | None = None
        self.theta_values: list[float] = []
        self.fusion_times_ms: list[float] = []
        self.raw_rows: list[dict] = []
        self.experiment_done = False
        self._create_pose_subscriptions()
        self.create_subscription(PointCloud2, str(self.get_parameter('source_matched_points_topic').value), self._source_points_callback, 10)
        self.create_subscription(PointCloud2, str(self.get_parameter('target_matched_points_topic').value), self._target_points_callback, 10)
        self.create_subscription(Float64, str(self.get_parameter('constraint_error_topic').value), self._constraint_error_callback, 10)
        self.create_subscription(Float64MultiArray, str(self.get_parameter('constraint_error_topic').value) + '_array', self._constraint_error_array_callback, 10)
        self.create_subscription(Float64MultiArray, str(self.get_parameter('observed_relative_transform_topic').value), self._observed_relative_transform_callback, 10)
        self.create_subscription(Float64MultiArray, str(self.get_parameter('predicted_relative_transform_topic').value), self._predicted_relative_transform_callback, 10)
        self.create_subscription(Float64, str(self.get_parameter('fusion_time_topic').value), self._fusion_time_callback, 10)
        self.create_subscription(Bool, '/experiment_done', self._experiment_done_callback, 10)
        self.create_timer(float(self.get_parameter('summary_period_s').value), self.write_current_results)
        self.get_logger().info(f'Writing EMRMF live experiment logs to {self.raw_csv}')

    def _create_pose_subscriptions(self) -> None:
        estimated_topics = json.loads(str(self.get_parameter('estimated_pose_topics_json').value))
        ground_truth_topics = json.loads(str(self.get_parameter('ground_truth_topics_json').value))
        for item in estimated_topics:
            self._subscribe_pose(item, self.estimated_history)
        for item in ground_truth_topics:
            self._subscribe_pose(item, self.ground_truth_history)

    def _subscribe_pose(self, item: dict, storage: dict) -> None:
        robot_id = item['robot_id']
        storage.setdefault(robot_id, [])
        message_type = item.get('type', 'Odometry')
        if message_type == 'PoseStamped':
            self.create_subscription(PoseStamped, item['topic'], lambda msg, rid=robot_id: self._record_pose_stamped(storage, rid, msg), 10)
        else:
            self.create_subscription(Odometry, item['topic'], lambda msg, rid=robot_id: self._record_odometry(storage, rid, msg), 10)

    def _record_odometry(self, storage: dict, robot_id: str, message: Odometry) -> None:
        storage.setdefault(robot_id, []).append((stamp_to_seconds(message), pose_from_odometry(message)))

    def _record_pose_stamped(self, storage: dict, robot_id: str, message: PoseStamped) -> None:
        storage.setdefault(robot_id, []).append((stamp_to_seconds(message), pose_from_stamped(message)))

    def _load_reference_trajectory(self, csv_path: str) -> dict[str, list[tuple[float, tuple[float, float, float]]]]:
        if not csv_path:
            return {}
        reference: dict[str, list[tuple[float, tuple[float, float, float]]]] = {}
        with Path(csv_path).open(newline='', encoding='utf-8') as handle:
            for row in csv.DictReader(handle):
                robot_id = row.get('robot_id', 'robot1')
                reference.setdefault(robot_id, []).append((
                    float(row.get('stamp', row.get('time', 0.0))),
                    (float(row['x']), float(row['y']), float(row.get('z', 0.0))),
                ))
        return reference

    def _source_points_callback(self, message: PointCloud2) -> None:
        self.source_points = self._pointcloud_to_points(message)

    def _target_points_callback(self, message: PointCloud2) -> None:
        self.target_points = self._pointcloud_to_points(message)

    def _pointcloud_to_points(self, message: PointCloud2) -> list[tuple[float, float, float]]:
        return [(float(x), float(y), float(z)) for x, y, z in point_cloud2.read_points(message, field_names=('x', 'y', 'z'), skip_nans=True)]

    def _constraint_error_callback(self, message: Float64) -> None:
        self._record_constraint_error(float(message.data), self.get_clock().now().nanoseconds / 1.0e9)

    def _constraint_error_array_callback(self, message: Float64MultiArray) -> None:
        now_s = self.get_clock().now().nanoseconds / 1.0e9
        for value in message.data:
            self._record_constraint_error(float(value), now_s)


    def _observed_relative_transform_callback(self, message: Float64MultiArray) -> None:
        self.latest_observed_relative_transform = [float(value) for value in message.data]
        self._compute_relative_transform_error()

    def _predicted_relative_transform_callback(self, message: Float64MultiArray) -> None:
        self.latest_predicted_relative_transform = [float(value) for value in message.data]
        self._compute_relative_transform_error()

    def _compute_relative_transform_error(self) -> None:
        if self.latest_observed_relative_transform is None or self.latest_predicted_relative_transform is None:
            return
        count = min(len(self.latest_observed_relative_transform), len(self.latest_predicted_relative_transform))
        if count == 0:
            return
        error_norm = euclidean_distance(
            self.latest_observed_relative_transform[:count],
            self.latest_predicted_relative_transform[:count],
        )
        self._record_constraint_error(error_norm, self.get_clock().now().nanoseconds / 1.0e9)

    def _record_constraint_error(self, error_norm: float, current_time_s: float) -> None:
        self.constraint_errors.append(error_norm)
        p_value = float(self.get_parameter('p').value)
        gamma = float(self.get_parameter('gamma').value)
        tau_e = float(self.get_parameter('tau_e').value)
        delay_s = float(self.get_parameter('delay_s').value)
        delta_t = max(0.0, delay_s)
        self.theta_values.append(compute_theta(error_norm, tau_e, p_value, gamma, delta_t))

    def _fusion_time_callback(self, message: Float64) -> None:
        self.fusion_times_ms.append(float(message.data))

    def _experiment_done_callback(self, message: Bool) -> None:
        self.experiment_done = bool(message.data)
        if self.experiment_done:
            self.write_current_results()

    def _aligned_pose_lists(self) -> tuple[list[Sequence[float]], list[Sequence[float]]]:
        estimated = []
        reference = []
        truth_source = self.ground_truth_history if self.ground_truth_history else self.reference_history
        for robot_id, estimates in self.estimated_history.items():
            truths = truth_source.get(robot_id, [])
            count = min(len(estimates), len(truths))
            estimated.extend([pose for _, pose in estimates[:count]])
            reference.extend([pose for _, pose in truths[:count]])
        return estimated, reference


    def _transform_source_points(self, points: Sequence[Sequence[float]]) -> list[tuple[float, float, float]]:
        transform = [float(value) for value in self.get_parameter('source_transform_xyz_yaw').value]
        dx, dy, dz, yaw = transform
        cos_yaw = math.cos(yaw)
        sin_yaw = math.sin(yaw)
        transformed = []
        for point in points:
            x_value = float(point[0])
            y_value = float(point[1])
            z_value = float(point[2]) if len(point) > 2 else 0.0
            transformed.append((
                cos_yaw * x_value - sin_yaw * y_value + dx,
                sin_yaw * x_value + cos_yaw * y_value + dy,
                z_value + dz,
            ))
        return transformed

    def write_current_results(self) -> None:
        estimated, reference = self._aligned_pose_lists()
        pose_error = pose_rmse(estimated, reference)
        transformed_source_points = self._transform_source_points(self.source_points)
        map_error = alignment_rmse(transformed_source_points, self.target_points, nearest_neighbor=True)
        fusion_time = mean(self.fusion_times_ms) if self.fusion_times_ms else 0.0
        theta_mean = mean(self.theta_values) if self.theta_values else 0.0
        theta_std = stdev(self.theta_values) if len(self.theta_values) > 1 else 0.0
        false_acceptance = self._false_constraint_acceptance_rate()
        consistency = max(0.0, min(1.0, 1.0 - map_error - false_acceptance))
        row = {
            'timestamp': self.started_at,
            'configuration_id': str(self.get_parameter('configuration_id').value),
            'run_id': int(self.get_parameter('run_id').value),
            'ablation_mode': str(self.get_parameter('ablation_mode').value),
            'p': float(self.get_parameter('p').value),
            'gamma': float(self.get_parameter('gamma').value),
            'tau_e': float(self.get_parameter('tau_e').value),
            'delay_s': float(self.get_parameter('delay_s').value),
            'packet_loss': float(self.get_parameter('packet_loss').value),
            'sensor_noise': float(self.get_parameter('sensor_noise').value),
            'pose_rmse': pose_error,
            'map_alignment_rmse': map_error,
            'fusion_time_ms': fusion_time,
            'theta_mean': theta_mean,
            'theta_std': theta_std,
            'false_constraint_acceptance_rate': false_acceptance,
            'map_consistency_score': consistency,
            'accepted_constraints': sum(1 for value in self.theta_values if value > 0.05),
            'success': 1 if pose_error < 0.5 and consistency > 0.5 else 0,
        }
        self.raw_rows.append(row)
        write_csv(self.raw_csv, self.raw_rows, RAW_FIELDNAMES)
        summary_rows = summarize_raw_rows(self.raw_rows)
        write_csv(self.summary_csv, summary_rows, SUMMARY_FIELDNAMES)
        self.summary_md.write_text(markdown_table(summary_rows, 'EMRMF Live Experiment Summary'), encoding='utf-8')
        self.summary_tex.write_text(latex_table(summary_rows, 'EMRMF live experiment summary'), encoding='utf-8')

    def _false_constraint_acceptance_rate(self) -> float:
        if not self.constraint_errors:
            return 0.0
        tau_e = float(self.get_parameter('tau_e').value)
        accepted = sum(1 for value in self.constraint_errors if value <= tau_e)
        false_accept = sum(1 for value in self.constraint_errors if value > tau_e and compute_theta(value, tau_e, float(self.get_parameter('p').value), float(self.get_parameter('gamma').value), float(self.get_parameter('delay_s').value)) > 0.05)
        return false_accept / max(1, accepted + false_accept)


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = ExperimentLoggerNode()
    try:
        rclpy.spin(node)
    finally:
        node.write_current_results()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
