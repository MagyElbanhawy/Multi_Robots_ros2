"""Predefined indoor trajectory commander for Gazebo scalability runs."""

from __future__ import annotations

import math

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node

from emrmf_core.qos import telemetry_qos


class GazeboTrajectoryNode(Node):
    """Publish deterministic cmd_vel commands for one robot namespace."""

    def __init__(self) -> None:
        super().__init__('gazebo_trajectory_node')
        self.declare_parameter('robot_id', 'robot_1')
        self.declare_parameter('trajectory_id', 0)
        self.declare_parameter('linear_speed_mps', 0.28)
        self.declare_parameter('angular_scale', 0.75)
        self.declare_parameter('publish_hz', 20.0)
        self.robot_id = str(self.get_parameter('robot_id').value)
        self.trajectory_id = int(self.get_parameter('trajectory_id').value)
        self.start_time = self.get_clock().now()
        self.publisher = self.create_publisher(Twist, 'cmd_vel', telemetry_qos())
        self.timer = self.create_timer(1.0 / float(self.get_parameter('publish_hz').value), self.publish_command)

    def publish_command(self) -> None:
        elapsed = (self.get_clock().now() - self.start_time).nanoseconds / 1.0e9
        base_speed = float(self.get_parameter('linear_speed_mps').value)
        angular_scale = float(self.get_parameter('angular_scale').value)
        phase = self.trajectory_id * math.pi / 3.0
        cmd = Twist()
        cmd.linear.x = base_speed * (0.85 + 0.15 * math.sin(0.08 * elapsed + phase))
        # Smooth figure-eight/corridor-like trajectories in the same indoor map.
        cmd.angular.z = angular_scale * (0.45 * math.sin(0.18 * elapsed + phase) + 0.18 * math.sin(0.47 * elapsed))
        self.publisher.publish(cmd)


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = GazeboTrajectoryNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
