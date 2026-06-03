"""Synthetic local SLAM producer for repeatable multi-robot experiments."""

from __future__ import annotations

import math
import random
from typing import List

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from emrmf_core.messages import encode, now_seconds
from emrmf_core.qos import telemetry_qos


class RobotSlamNode(Node):
    """Publish local pose/landmark observations with configurable noise and faults."""

    def __init__(self) -> None:
        super().__init__('robot_slam_node')
        self.declare_parameter('robot_id', 'robot_1')
        self.declare_parameter('frame_id', 'map')
        self.declare_parameter('publish_hz', 5.0)
        self.declare_parameter('sensor_noise_std', 0.05)
        self.declare_parameter('latency_hint_s', 0.0)
        self.declare_parameter('fault_bias_m', 0.0)
        self.declare_parameter('landmark_count', 8)
        self.declare_parameter('trajectory_radius_m', 6.0)

        self.robot_id = self.get_parameter('robot_id').value
        self.frame_id = self.get_parameter('frame_id').value
        publish_hz = float(self.get_parameter('publish_hz').value)
        self.publisher = self.create_publisher(String, 'local_slam/observations', telemetry_qos())
        self.step = 0
        self.rng = random.Random(hash(self.robot_id) & 0xFFFFFFFF)
        self.timer = self.create_timer(1.0 / max(publish_hz, 0.1), self.publish_observation)

    def publish_observation(self) -> None:
        noise_std = float(self.get_parameter('sensor_noise_std').value)
        latency_hint_s = float(self.get_parameter('latency_hint_s').value)
        fault_bias_m = float(self.get_parameter('fault_bias_m').value)
        radius = float(self.get_parameter('trajectory_radius_m').value)
        landmark_count = int(self.get_parameter('landmark_count').value)

        t = self.step * 0.1
        robot_offset = (abs(hash(self.robot_id)) % 360) * math.pi / 180.0
        true_pose = [radius * math.cos(t + robot_offset), radius * math.sin(t + robot_offset), t]
        measured_pose = [
            true_pose[0] + self.rng.gauss(fault_bias_m, noise_std),
            true_pose[1] + self.rng.gauss(fault_bias_m, noise_std),
            true_pose[2] + self.rng.gauss(0.0, noise_std * 0.2),
        ]
        landmarks: List[dict] = []
        for idx in range(landmark_count):
            angle = (2.0 * math.pi * idx / max(landmark_count, 1)) + robot_offset * 0.1
            landmarks.append({
                'id': f'L{idx:02d}',
                'x': 10.0 * math.cos(angle) + self.rng.gauss(fault_bias_m, noise_std),
                'y': 10.0 * math.sin(angle) + self.rng.gauss(fault_bias_m, noise_std),
            })

        payload = {
            'type': 'local_slam_observation',
            'robot_id': self.robot_id,
            'frame_id': self.frame_id,
            'seq': self.step,
            'stamp': now_seconds(),
            'latency_hint_s': latency_hint_s,
            'sensor_noise_std': noise_std,
            'fault_bias_m': fault_bias_m,
            'pose': measured_pose,
            'true_pose': true_pose,
            'landmarks': landmarks,
        }
        msg = String()
        msg.data = encode(payload)
        self.publisher.publish(msg)
        self.step += 1


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = RobotSlamNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
