"""LoRa-like network emulator with delay, packet loss, and byte-rate reporting."""

from __future__ import annotations

import random
from collections import deque
from typing import Deque, Tuple

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from emrmf_core.messages import decode, encode, now_seconds
from emrmf_core.qos import telemetry_qos


class LoraNetworkSimulator(Node):
    """Relay local SLAM observations through a configurable constrained link."""

    def __init__(self) -> None:
        super().__init__('lora_network_simulator')
        self.declare_parameter('delay_s', 0.5)
        self.declare_parameter('jitter_s', 0.05)
        self.declare_parameter('packet_loss', 0.0)
        self.declare_parameter('packet_interval_s', 0.2)
        self.declare_parameter('transmission_distance_m', 250.0)
        self.declare_parameter('random_seed', 7)

        self.rng = random.Random(int(self.get_parameter('random_seed').value))
        self.queue: Deque[Tuple[float, String]] = deque()
        self.bytes_forwarded = 0
        self.last_report = now_seconds()
        self.subscription = self.create_subscription(String, 'local_slam/observations', self.enqueue, telemetry_qos())
        self.publisher = self.create_publisher(String, 'lora/observations', telemetry_qos())
        self.timer = self.create_timer(0.02, self.flush_ready)

    def enqueue(self, msg: String) -> None:
        packet_loss = float(self.get_parameter('packet_loss').value)
        if self.rng.random() < packet_loss:
            return
        delay_s = float(self.get_parameter('delay_s').value)
        jitter_s = float(self.get_parameter('jitter_s').value)
        available_at = now_seconds() + max(0.0, self.rng.gauss(delay_s, jitter_s))
        self.queue.append((available_at, msg))

    def flush_ready(self) -> None:
        interval = float(self.get_parameter('packet_interval_s').value)
        current = now_seconds()
        if current - self.last_report < max(interval, 0.01):
            return
        self.last_report = current
        while self.queue and self.queue[0][0] <= current:
            _, msg = self.queue.popleft()
            payload = decode(msg.data)
            payload['network_stamp'] = current
            payload['network_delay_s'] = current - float(payload.get('stamp', current))
            payload['packet_loss_config'] = float(self.get_parameter('packet_loss').value)
            payload['transmission_distance_m'] = float(self.get_parameter('transmission_distance_m').value)
            outgoing = String()
            outgoing.data = encode(payload)
            self.bytes_forwarded += len(outgoing.data.encode('utf-8'))
            self.publisher.publish(outgoing)


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = LoraNetworkSimulator()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
