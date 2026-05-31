"""Experiment logger that writes scalability_robot_count_summary.csv."""

from __future__ import annotations

from pathlib import Path

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from emrmf_core.messages import encode, now_seconds
from emrmf_core.qos import telemetry_qos
from emrmf_core.scalability import generate_scalability_summary, write_csv, write_markdown


class ScalabilityExperimentLoggerNode(Node):
    """Generate and publish the EMRMF scalability summary artifact."""

    def __init__(self) -> None:
        super().__init__('scalability_experiment_logger_node')
        self.declare_parameter('robot_counts', [2, 3, 4, 5])
        self.declare_parameter('trials', 5)
        self.declare_parameter('samples_per_robot', 120)
        self.declare_parameter('sensor_noise_std', 0.04)
        self.declare_parameter('gamma', 0.35)
        self.declare_parameter('csv_path', 'docs/scalability_robot_count_summary.csv')
        self.declare_parameter('markdown_path', 'docs/scalability_robot_count_summary.md')
        self.publisher = self.create_publisher(String, 'experiments/scalability_summary', telemetry_qos())
        self.timer = self.create_timer(1.0, self.generate_once)
        self.completed = False

    def generate_once(self) -> None:
        if self.completed:
            return
        self.completed = True
        result = generate_scalability_summary(
            robot_counts=[int(value) for value in self.get_parameter('robot_counts').value],
            trials=int(self.get_parameter('trials').value),
            samples_per_robot=int(self.get_parameter('samples_per_robot').value),
            sensor_noise_std=float(self.get_parameter('sensor_noise_std').value),
            gamma=float(self.get_parameter('gamma').value),
        )
        csv_path = Path(str(self.get_parameter('csv_path').value))
        markdown_path = Path(str(self.get_parameter('markdown_path').value))
        csv_path.parent.mkdir(parents=True, exist_ok=True)
        markdown_path.parent.mkdir(parents=True, exist_ok=True)
        write_csv(result, csv_path)
        write_markdown(result, markdown_path)
        result['stamp'] = now_seconds()
        result['csv_path'] = str(csv_path)
        result['markdown_path'] = str(markdown_path)
        msg = String()
        msg.data = encode(result)
        self.publisher.publish(msg)
        self.get_logger().info(f"Wrote scalability summary to {csv_path}")


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = ScalabilityExperimentLoggerNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
