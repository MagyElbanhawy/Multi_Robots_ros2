"""Peer-style decentralized map cache for server outage comparisons."""

from __future__ import annotations

from typing import Dict

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from emrmf_core.messages import decode, encode, merge_landmarks, now_seconds, weighted_pose_average
from emrmf_core.qos import telemetry_qos


class DecentralizedMapNode(Node):
    """Maintain a robot-local fusion result using the most recent peer packets."""

    def __init__(self) -> None:
        super().__init__('decentralized_map_node')
        self.declare_parameter('robot_id', 'robot_1')
        self.declare_parameter('peer_timeout_s', 5.0)
        self.declare_parameter('publish_hz', 1.0)
        self.robot_id = str(self.get_parameter('robot_id').value)
        self.latest_by_robot: Dict[str, Dict] = {}
        self.subscription = self.create_subscription(String, 'trust/weighted_observations', self.record_peer, telemetry_qos())
        self.publisher = self.create_publisher(String, 'decentralized/fused_map', telemetry_qos())
        self.timer = self.create_timer(1.0 / max(float(self.get_parameter('publish_hz').value), 0.1), self.publish_local_map)

    def record_peer(self, msg: String) -> None:
        payload = decode(msg.data)
        if not payload:
            return
        payload['received_by'] = self.robot_id
        payload['peer_received_stamp'] = now_seconds()
        self.latest_by_robot[str(payload.get('robot_id', 'unknown'))] = payload

    def publish_local_map(self) -> None:
        current = now_seconds()
        timeout = float(self.get_parameter('peer_timeout_s').value)
        fresh = [item for item in self.latest_by_robot.values() if current - item.get('peer_received_stamp', current) <= timeout]
        if not fresh:
            return
        payload = {
            'type': 'decentralized_fused_map',
            'robot_id': self.robot_id,
            'stamp': current,
            'pose': weighted_pose_average(fresh),
            'landmarks': merge_landmarks(fresh),
            'peer_count': len(fresh),
            'architecture': 'fully_decentralized_peer_cache',
        }
        msg = String()
        msg.data = encode(payload)
        self.publisher.publish(msg)


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = DecentralizedMapNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
