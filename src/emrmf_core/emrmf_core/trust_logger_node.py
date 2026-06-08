"""CSV logger for trust-factor diagnostics."""

from __future__ import annotations

import csv
from pathlib import Path
from typing import Any

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from emrmf_core.messages import decode, now_seconds
from emrmf_core.qos import telemetry_qos


FIELDNAMES = ['timestamp', 'time', 'theta', 'delay', 'packet_loss', 'robot_id']


class TrustLoggerNode(Node):
    """Record trust-factor evolution to a reviewer-friendly CSV file."""

    def __init__(self) -> None:
        super().__init__('trust_logger_node')
        self.declare_parameter('csv_path', 'logs/trust_factor.csv')
        self.declare_parameter('input_topic', 'trust/weighted_observations')

        self.csv_path = Path(str(self.get_parameter('csv_path').value))
        self.csv_path.parent.mkdir(parents=True, exist_ok=True)
        self.start_time: float | None = None

        self.file_handle = self.csv_path.open('a', newline='', encoding='utf-8')
        self.writer = csv.DictWriter(self.file_handle, fieldnames=FIELDNAMES)
        if self.csv_path.stat().st_size == 0:
            self.writer.writeheader()
            self.file_handle.flush()

        input_topic = str(self.get_parameter('input_topic').value)
        self.subscription = self.create_subscription(String, input_topic, self.record, telemetry_qos())
        self.get_logger().info(f'Recording trust factors from {input_topic} to {self.csv_path}')

    def record(self, msg: String) -> None:
        payload = decode(msg.data)
        if not payload or 'theta' not in payload:
            return

        timestamp = float(payload.get('trust_stamp', now_seconds()))
        if self.start_time is None:
            self.start_time = timestamp

        self.writer.writerow({
            'timestamp': timestamp,
            'time': timestamp - self.start_time,
            'theta': _float_value(payload, 'theta', 0.0),
            'delay': _float_value(payload, 'network_delay_s', _float_value(payload, 'latency_hint_s', 0.0)),
            'packet_loss': _float_value(payload, 'packet_loss_config', 0.0),
            'robot_id': str(payload.get('robot_id', 'unknown')),
        })
        self.file_handle.flush()

    def destroy_node(self) -> bool:
        if not self.file_handle.closed:
            self.file_handle.close()
        return super().destroy_node()


def _float_value(payload: dict[str, Any], key: str, default: float) -> float:
    try:
        return float(payload.get(key, default))
    except (TypeError, ValueError):
        return default


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = TrustLoggerNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
