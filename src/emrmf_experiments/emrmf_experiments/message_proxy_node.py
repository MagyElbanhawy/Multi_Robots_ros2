"""Portable ROS 2 delay/packet-loss proxy for EMRMF communication topics."""

from __future__ import annotations

import heapq
import json
import random
from dataclasses import dataclass, field
from typing import Any

import rclpy
from rclpy.node import Node
from rosidl_runtime_py.utilities import get_message


DEFAULT_TOPIC_PAIRS = [
    {'input': '/robot1/local_map', 'output': '/proxy/robot1/local_map', 'type': 'std_msgs/msg/String'},
    {'input': '/robot2/local_map', 'output': '/proxy/robot2/local_map', 'type': 'std_msgs/msg/String'},
    {'input': '/robot1/pose_update', 'output': '/proxy/robot1/pose_update', 'type': 'nav_msgs/msg/Odometry'},
    {'input': '/robot2/pose_update', 'output': '/proxy/robot2/pose_update', 'type': 'nav_msgs/msg/Odometry'},
    {
        'input': '/map_fusion/inter_robot_constraints',
        'output': '/proxy/map_fusion/inter_robot_constraints',
        'type': 'std_msgs/msg/String',
    },
]


@dataclass(order=True)
class BufferedMessage:
    """One delayed message in the proxy priority queue."""

    release_time: float
    sequence: int
    publisher_key: str = field(compare=False)
    message: Any = field(compare=False)


class CommunicationProxyNode(Node):
    """Subscribe, delay/drop, and republish configured ROS 2 topics."""

    def __init__(self) -> None:
        super().__init__('emrmf_communication_proxy')
        self.declare_parameter('delay_s', 0.0)
        self.declare_parameter('packet_loss', 0.0)
        self.declare_parameter('random_seed', 2026)
        self.declare_parameter('topic_pairs_json', json.dumps(DEFAULT_TOPIC_PAIRS))
        self.random = random.Random(int(self.get_parameter('random_seed').value))
        self.publishers_by_key = {}
        self.queue: list[BufferedMessage] = []
        self.sequence = 0
        self.accepted_count = 0
        self.dropped_count = 0
        topic_pairs = json.loads(str(self.get_parameter('topic_pairs_json').value))
        for pair in topic_pairs:
            self._add_topic_pair(pair)
        self.create_timer(0.01, self._flush_ready_messages)
        self.get_logger().info(f'Configured {len(topic_pairs)} proxied communication topics')

    def _add_topic_pair(self, pair: dict) -> None:
        msg_type = get_message(pair['type'])
        publisher = self.create_publisher(msg_type, pair['output'], 10)
        self.publishers_by_key[pair['output']] = publisher

        def callback(message, output_topic=pair['output']):
            self._buffer_or_drop(output_topic, message)

        self.create_subscription(msg_type, pair['input'], callback, 10)
        self.get_logger().info(f"Proxying {pair['input']} -> {pair['output']} as {pair['type']}")

    def _buffer_or_drop(self, output_topic: str, message) -> None:
        packet_loss = min(1.0, max(0.0, float(self.get_parameter('packet_loss').value)))
        if self.random.random() < packet_loss:
            self.dropped_count += 1
            return
        delay_s = max(0.0, float(self.get_parameter('delay_s').value))
        release_time = self.get_clock().now().nanoseconds / 1.0e9 + delay_s
        self.sequence += 1
        heapq.heappush(self.queue, BufferedMessage(release_time, self.sequence, output_topic, message))
        self.accepted_count += 1

    def _flush_ready_messages(self) -> None:
        now_s = self.get_clock().now().nanoseconds / 1.0e9
        while self.queue and self.queue[0].release_time <= now_s:
            item = heapq.heappop(self.queue)
            self.publishers_by_key[item.publisher_key].publish(item.message)


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = CommunicationProxyNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
