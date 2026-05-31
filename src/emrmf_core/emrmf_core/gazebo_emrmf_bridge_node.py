"""Bridge Gazebo odometry/LiDAR/depth topics into EMRMF local observations."""

from __future__ import annotations

import math
from typing import Optional

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import Image, LaserScan
from std_msgs.msg import String

from emrmf_core.messages import encode, now_seconds
from emrmf_core.qos import telemetry_qos


class GazeboEmrmfBridgeNode(Node):
    """Convert Gazebo sensor streams into the existing EMRMF observation topic."""

    def __init__(self) -> None:
        super().__init__('gazebo_emrmf_bridge_node')
        self.declare_parameter('robot_id', 'robot_1')
        self.declare_parameter('sensor_noise_std', 0.04)
        self.declare_parameter('publish_hz', 5.0)
        self.robot_id = str(self.get_parameter('robot_id').value)
        self.latest_odom: Optional[Odometry] = None
        self.latest_scan: Optional[LaserScan] = None
        self.latest_depth: Optional[Image] = None
        self.create_subscription(Odometry, 'odom', self.record_odom, telemetry_qos())
        self.create_subscription(LaserScan, 'scan', self.record_scan, telemetry_qos())
        self.create_subscription(Image, 'depth_camera/image_raw', self.record_depth, telemetry_qos())
        self.publisher = self.create_publisher(String, 'local_slam/observations', telemetry_qos())
        self.timer = self.create_timer(1.0 / float(self.get_parameter('publish_hz').value), self.publish_observation)
        self.seq = 0

    def record_odom(self, msg: Odometry) -> None:
        self.latest_odom = msg

    def record_scan(self, msg: LaserScan) -> None:
        self.latest_scan = msg

    def record_depth(self, msg: Image) -> None:
        self.latest_depth = msg

    def publish_observation(self) -> None:
        if self.latest_odom is None:
            return
        pose_msg = self.latest_odom.pose.pose
        yaw = quaternion_to_yaw(pose_msg.orientation.x, pose_msg.orientation.y, pose_msg.orientation.z, pose_msg.orientation.w)
        ranges = list(self.latest_scan.ranges) if self.latest_scan else []
        finite_ranges = [value for value in ranges if math.isfinite(value)]
        scan_mean_range = sum(finite_ranges) / len(finite_ranges) if finite_ranges else 0.0
        depth_available = self.latest_depth is not None
        payload = {
            'type': 'gazebo_local_slam_observation',
            'robot_id': self.robot_id,
            'seq': self.seq,
            'stamp': now_seconds(),
            'sensor_noise_std': float(self.get_parameter('sensor_noise_std').value),
            'pose': [pose_msg.position.x, pose_msg.position.y, yaw],
            'true_pose': [pose_msg.position.x, pose_msg.position.y, yaw],
            'scan_beams': len(ranges),
            'scan_mean_range_m': scan_mean_range,
            'depth_available': depth_available,
            'landmarks': synthetic_landmarks_from_scan(self.robot_id, finite_ranges[:24]),
        }
        msg = String()
        msg.data = encode(payload)
        self.publisher.publish(msg)
        self.seq += 1


def quaternion_to_yaw(x: float, y: float, z: float, w: float) -> float:
    """Convert quaternion components to yaw."""

    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


def synthetic_landmarks_from_scan(robot_id: str, ranges: list[float]) -> list[dict]:
    """Create compact scan-derived pseudo landmarks for the EMRMF fusion topic."""

    if not ranges:
        return []
    stride = max(1, len(ranges) // 6)
    landmarks = []
    for index, distance in enumerate(ranges[::stride][:6]):
        angle = -math.pi + index * (2.0 * math.pi / 6.0)
        landmarks.append({
            'id': f'{robot_id}_scan_{index}',
            'x': distance * math.cos(angle),
            'y': distance * math.sin(angle),
        })
    return landmarks


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = GazeboEmrmfBridgeNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
