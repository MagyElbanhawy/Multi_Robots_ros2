"""Gazebo scalability launch for EMRMF with 2-5 mobile robots."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def _spawn_robots(context, *args, **kwargs):
    robot_count = int(LaunchConfiguration('robot_count').perform(context))
    sensor_noise_std = LaunchConfiguration('sensor_noise_std')
    enable_depth_camera = LaunchConfiguration('enable_depth_camera')
    xacro_path = PathJoinSubstitution([FindPackageShare('emrmf_core'), 'urdf', 'emrmf_mobile_robot.urdf.xacro'])
    initial_poses = [(-6.2, -4.0, 0.0), (-6.2, 4.0, 0.0), (6.2, -4.0, 3.14), (6.2, 4.0, 3.14), (0.0, -4.1, 1.57)]
    actions = []
    for index in range(robot_count):
        robot_name = f'robot_{index + 1}'
        robot_description = ParameterValue(
            Command([
                'xacro ',
                xacro_path,
                ' robot_name:=', robot_name,
                ' sensor_noise_std:=', sensor_noise_std,
                ' enable_depth_camera:=', enable_depth_camera,
            ]),
            value_type=str,
        )
        x, y, yaw = initial_poses[index]
        actions.extend([
            Node(
                package='robot_state_publisher',
                executable='robot_state_publisher',
                namespace=robot_name,
                name='robot_state_publisher',
                parameters=[{'robot_description': robot_description, 'frame_prefix': f'{robot_name}/'}],
                output='screen',
            ),
            Node(
                package='gazebo_ros',
                executable='spawn_entity.py',
                arguments=[
                    '-topic', f'/{robot_name}/robot_description',
                    '-entity', robot_name,
                    '-robot_namespace', robot_name,
                    '-x', str(x), '-y', str(y), '-z', '0.08', '-Y', str(yaw),
                ],
                output='screen',
            ),
            Node(
                package='emrmf_core',
                executable='gazebo_trajectory_node',
                namespace=robot_name,
                name='trajectory_commander',
                parameters=[{'robot_id': robot_name, 'trajectory_id': index}],
                output='screen',
            ),
            Node(
                package='emrmf_core',
                executable='gazebo_emrmf_bridge_node',
                namespace=robot_name,
                name='emrmf_local_slam_bridge',
                parameters=[{'robot_id': robot_name, 'sensor_noise_std': sensor_noise_std}],
                remappings=[('local_slam/observations', '/local_slam/observations')],
                output='screen',
            ),
        ])
    return actions


def generate_launch_description():
    world = PathJoinSubstitution([FindPackageShare('emrmf_core'), 'worlds', 'emrmf_indoor.world'])
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([PathJoinSubstitution([FindPackageShare('gazebo_ros'), 'launch', 'gazebo.launch.py'])]),
        launch_arguments={'world': world, 'verbose': 'false'}.items(),
    )
    return LaunchDescription([
        DeclareLaunchArgument('robot_count', default_value='2', description='Robot team size: 2, 3, 4, or 5.'),
        DeclareLaunchArgument('sensor_noise_std', default_value='0.04', description='Gaussian noise for odometry, LiDAR, and depth camera.'),
        DeclareLaunchArgument('enable_depth_camera', default_value='true', description='Enable optional depth camera sensor.'),
        DeclareLaunchArgument('trials', default_value='5', description='Trials per robot count for the scalability logger.'),
        ExecuteProcess(cmd=['bash', '-lc', 'echo Starting EMRMF Gazebo scalability launch'], output='screen'),
        gazebo,
        OpaqueFunction(function=_spawn_robots),
        Node(package='emrmf_core', executable='lora_network_simulator', name='lora_network_simulator', output='screen'),
        Node(package='emrmf_core', executable='trust_factor_node', name='trust_factor_node', output='screen'),
        Node(package='emrmf_core', executable='global_fusion_node', name='global_fusion_node', output='screen'),
        Node(package='emrmf_core', executable='scalability_experiment_logger_node', name='emrmf_scalability_logger', parameters=[{'trials': LaunchConfiguration('trials'), 'sensor_noise_std': LaunchConfiguration('sensor_noise_std')}], output='screen'),
        Node(package='emrmf_core', executable='experiment_runner_node', name='emrmf_experiment_logger', output='screen'),
    ])
