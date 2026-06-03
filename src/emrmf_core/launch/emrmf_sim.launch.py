"""Launch EMRMF simulation nodes for hybrid and decentralized comparisons."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    params = PathJoinSubstitution([FindPackageShare('emrmf_core'), 'config', 'emrmf_params.yaml'])
    robot_count = LaunchConfiguration('robot_count')
    delay_s = LaunchConfiguration('delay_s')
    packet_loss = LaunchConfiguration('packet_loss')
    gamma = LaunchConfiguration('gamma')

    declared_arguments = [
        DeclareLaunchArgument('robot_count', default_value='3', description='Number of synthetic robot SLAM producers.'),
        DeclareLaunchArgument('delay_s', default_value='0.5', description='Injected LoRa delay in seconds.'),
        DeclareLaunchArgument('packet_loss', default_value='0.0', description='Injected LoRa packet loss probability.'),
        DeclareLaunchArgument('gamma', default_value='0.35', description='Temporal decay constant for theta.'),
    ]

    # Launch substitutions cannot directly drive Python ranges, so the default file starts three robots.
    robot_nodes = [
        Node(
            package='emrmf_core',
            executable='robot_slam_node',
            name=f'robot_{idx}_slam',
            namespace=f'robot_{idx}',
            parameters=[params, {'robot_id': f'robot_{idx}', 'sensor_noise_std': 0.05 + idx * 0.02}],
            remappings=[('local_slam/observations', '/local_slam/observations')],
            output='screen',
        )
        for idx in range(1, 4)
    ]

    shared_nodes = [
        Node(
            package='emrmf_core',
            executable='lora_network_simulator',
            name='lora_network_simulator',
            parameters=[params, {'delay_s': delay_s, 'packet_loss': packet_loss}],
            output='screen',
        ),
        Node(
            package='emrmf_core',
            executable='trust_factor_node',
            name='trust_factor_node',
            parameters=[params, {'gamma': gamma}],
            output='screen',
        ),
        Node(
            package='emrmf_core',
            executable='global_fusion_node',
            name='global_fusion_node',
            parameters=[params],
            output='screen',
        ),
        Node(
            package='emrmf_core',
            executable='decentralized_map_node',
            name='robot_1_decentralized_map',
            namespace='robot_1',
            parameters=[params, {'robot_id': 'robot_1'}],
            remappings=[('trust/weighted_observations', '/trust/weighted_observations')],
            output='screen',
        ),
        Node(
            package='emrmf_core',
            executable='adversarial_monitor_node',
            name='adversarial_monitor_node',
            parameters=[params],
            output='screen',
        ),
    ]

    return LaunchDescription(declared_arguments + robot_nodes + shared_nodes)
