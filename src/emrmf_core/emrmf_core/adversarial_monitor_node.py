"""Residual-based monitor for faulty or adversarial robots."""

from __future__ import annotations

from statistics import mean
from typing import Dict, List

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from emrmf_core.messages import decode, encode, now_seconds, pose_distance
from emrmf_core.qos import telemetry_qos


class AdversarialMonitorNode(Node):
    """Flag robots whose poses are inconsistent with trusted peers over time."""

    def __init__(self) -> None:
        super().__init__('adversarial_monitor_node')
        self.declare_parameter('residual_threshold_m', 1.5)
        self.declare_parameter('min_samples', 5)
        self.declare_parameter('low_trust_threshold', 0.35)
        self.history: Dict[str, List[float]] = {}
        self.last_pose_by_robot: Dict[str, List[float]] = {}
        self.subscription = self.create_subscription(String, 'trust/weighted_observations', self.inspect, telemetry_qos())
        self.publisher = self.create_publisher(String, 'trust/adversarial_alerts', telemetry_qos())

    def inspect(self, msg: String) -> None:
        payload = decode(msg.data)
        if not payload:
            return
        robot_id = str(payload.get('robot_id', 'unknown'))
        pose = payload.get('pose', [0.0, 0.0, 0.0])
        peer_poses = [p for rid, p in self.last_pose_by_robot.items() if rid != robot_id]
        self.last_pose_by_robot[robot_id] = pose
        if not peer_poses:
            return
        centroid = [sum(p[0] for p in peer_poses) / len(peer_poses), sum(p[1] for p in peer_poses) / len(peer_poses), 0.0]
        residual = pose_distance(pose, centroid)
        robot_history = self.history.setdefault(robot_id, [])
        robot_history.append(residual)
        del robot_history[:-20]
        suspicious = (
            len(robot_history) >= int(self.get_parameter('min_samples').value)
            and mean(robot_history) > float(self.get_parameter('residual_threshold_m').value)
            and float(payload.get('theta', 1.0)) < float(self.get_parameter('low_trust_threshold').value)
        )
        if suspicious:
            alert = String()
            alert.data = encode({
                'type': 'adversarial_or_faulty_robot_alert',
                'stamp': now_seconds(),
                'robot_id': robot_id,
                'mean_residual_m': mean(robot_history),
                'theta': float(payload.get('theta', 0.0)),
                'recommended_action': 'down-weight, isolate from global fusion, and request peer verification',
            })
            self.publisher.publish(alert)


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = AdversarialMonitorNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
