"""Launch EMRMF communication proxy and live experiment logger."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    params_file = PathJoinSubstitution([
        FindPackageShare('emrmf_experiments'),
        'config',
        'emrmf_experiment_params.yaml',
    ])
    return LaunchDescription([
        DeclareLaunchArgument('params_file', default_value=params_file),
        Node(
            package='emrmf_experiments',
            executable='communication_proxy_node',
            name='emrmf_communication_proxy',
            parameters=[LaunchConfiguration('params_file')],
            output='screen',
        ),
        Node(
            package='emrmf_experiments',
            executable='experiment_logger_node',
            name='emrmf_experiment_logger',
            parameters=[LaunchConfiguration('params_file')],
            output='screen',
        ),
    ])
