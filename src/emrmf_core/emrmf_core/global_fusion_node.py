"""Central/hybrid global map fusion node with timeout-aware fault tolerance."""

from __future__ import annotations

from collections import deque
from typing import Deque, Dict

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from emrmf_core.messages import decode, encode, merge_landmarks, now_seconds, pose_distance, weighted_pose_average
from emrmf_core.qos import telemetry_qos


class GlobalFusionNode(Node):
    """Fuse trusted observations while publishing server-health diagnostics."""

    def __init__(self) -> None:
        super().__init__('global_fusion_node')
        self.declare_parameter('fusion_window', 25)
        self.declare_parameter('server_timeout_s', 3.0)
        self.declare_parameter('min_theta_for_fusion', 0.05)
        self.declare_parameter('publish_hz', 2.0)

        self.observations: Deque[Dict] = deque(maxlen=int(self.get_parameter('fusion_window').value))
        self.last_robot_stamp: Dict[str, float] = {}
        self.subscription = self.create_subscription(String, 'trust/weighted_observations', self.record, telemetry_qos())
        self.publisher = self.create_publisher(String, 'global/fused_map', telemetry_qos())
        self.timer = self.create_timer(1.0 / max(float(self.get_parameter('publish_hz').value), 0.1), self.publish_fusion)

    def record(self, msg: String) -> None:
        payload = decode(msg.data)
        if not payload:
            return
        if float(payload.get('theta', 0.0)) < float(self.get_parameter('min_theta_for_fusion').value):
            return
        self.observations.append(payload)
        self.last_robot_stamp[str(payload.get('robot_id', 'unknown'))] = now_seconds()

    def publish_fusion(self) -> None:
        if not self.observations:
            return
        current = now_seconds()
        timeout = float(self.get_parameter('server_timeout_s').value)
        active = [rid for rid, stamp in self.last_robot_stamp.items() if current - stamp <= timeout]
        stale = [rid for rid, stamp in self.last_robot_stamp.items() if current - stamp > timeout]
        items = list(self.observations)
        fused_pose = weighted_pose_average(items)
        fused_landmarks = merge_landmarks(items)
        rmse_values = []
        for item in items:
            if 'true_pose' in item:
                rmse_values.append(pose_distance(item['pose'], item['true_pose']))
        mean_local_rmse = sum(rmse_values) / len(rmse_values) if rmse_values else None
        payload = {
            'type': 'global_fused_map',
            'stamp': current,
            'architecture': 'hybrid_server_with_local_fallback',
            'active_robots': active,
            'stale_robots': stale,
            'server_healthy': len(active) > 0,
            'fallback_policy': 'robots keep decentralized/local maps if server topic times out',
            'pose': fused_pose,
            'landmarks': fused_landmarks,
            'observation_count': len(items),
            'mean_local_rmse_m': mean_local_rmse,
        }
        msg = String()
        msg.data = encode(payload)
        self.publisher.publish(msg)


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = GlobalFusionNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
