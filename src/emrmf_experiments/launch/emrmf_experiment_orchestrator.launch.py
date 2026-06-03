"""Launch the active EMRMF experiment orchestrator."""

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
            executable='experiment_orchestrator_node',
            name='emrmf_experiment_orchestrator',
            parameters=[LaunchConfiguration('params_file')],
            output='screen',
        ),
    ])
