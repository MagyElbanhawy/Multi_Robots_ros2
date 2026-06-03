"""Launch offline EMRMF ablation/sensitivity experiment summary node."""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='emrmf_core',
            executable='experiment_runner_node',
            name='experiment_runner_node',
            output='screen',
        )
    ])
