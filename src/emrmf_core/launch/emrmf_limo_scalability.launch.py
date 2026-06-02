"""Gazebo Classic LIMO scalability launch for EMRMF reviewer screenshots."""

from __future__ import annotations

from pathlib import Path

from ament_index_python.packages import PackageNotFoundError, get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    OpaqueFunction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare

VALID_ROBOT_COUNTS = {0, 2, 3, 4, 5}
LIMO_SPAWN_POSES = [
    (-5.7, 1.65, 0.0),    # limo_1 near room 1
    (5.7, 1.65, 3.14),    # limo_2 near room 2
    (0.0, -0.75, 1.57),   # limo_3 near corridor
    (5.6, -4.45, 3.14),   # limo_4 near meeting area
    (-5.8, -4.55, 0.0),   # limo_5 near office/lab area
]


def _find_limo_description() -> tuple[Path, bool]:
    """Return a LIMO xacro path and whether it is the EMRMF placeholder.

    The Construct/browser images used for reviewer screenshots may not include
    AgileX's ``limo_description`` package.  When it is available, prefer its
    URDF/Xacro; otherwise use the lightweight Gazebo-compatible placeholder
    installed by ``emrmf_core``.
    """

    candidate_names = (
        'urdf/limo_four_diff.xacro',
        'urdf/limo_ackerman.xacro',
        'urdf/limo_diff.xacro',
        'urdf/limo.xacro',
        'urdf/limo.urdf.xacro',
    )
    try:
        limo_share = Path(get_package_share_directory('limo_description'))
        for relative_path in candidate_names:
            candidate = limo_share / relative_path
            if candidate.exists():
                return candidate, False
    except PackageNotFoundError:
        pass

    fallback = (
        Path(get_package_share_directory('emrmf_core'))
        / 'models'
        / 'limo_placeholder'
        / 'limo_placeholder.urdf.xacro'
    )
    return fallback, True


def _robot_description_command(
    xacro_path: Path,
    robot_name: str,
    robot_namespace: str,
    sensor_noise_std,
    is_placeholder: bool,
):
    command = ['xacro ', str(xacro_path)]
    if is_placeholder:
        command.extend([
            ' robot_name:=', robot_name,
            ' robot_namespace:=', robot_namespace,
            ' sensor_noise_std:=', sensor_noise_std,
        ])
    return ParameterValue(Command(command), value_type=str)


def _spawn_limo_robots(context, *args, **kwargs):
    robot_count = int(LaunchConfiguration('robot_count').perform(context))
    if robot_count not in VALID_ROBOT_COUNTS:
        raise ValueError('robot_count must be one of 0, 2, 3, 4, or 5')

    sensor_noise_std = LaunchConfiguration('sensor_noise_std')
    xacro_path, is_placeholder = _find_limo_description()
    actions = []
    for index in range(robot_count):
        robot_name = f'limo_{index + 1}'
        robot_namespace = f'/{robot_name}'
        x, y, yaw = LIMO_SPAWN_POSES[index]
        robot_description = _robot_description_command(
            xacro_path,
            robot_name,
            robot_namespace,
            sensor_noise_std,
            is_placeholder,
        )
        actions.extend([
            Node(
                package='robot_state_publisher',
                executable='robot_state_publisher',
                namespace=robot_name,
                name='robot_state_publisher',
                parameters=[{
                    'robot_description': robot_description,
                    'frame_prefix': f'{robot_name}/',
                }],
                remappings=[('/tf', 'tf'), ('/tf_static', 'tf_static')],
                output='screen',
            ),
            Node(
                package='gazebo_ros',
                executable='spawn_entity.py',
                arguments=[
                    '-topic', f'/{robot_name}/robot_description',
                    '-entity', robot_name,
                    '-robot_namespace', robot_namespace,
                    '-x', str(x), '-y', str(y), '-z', '0.16', '-Y', str(yaw),
                ],
                output='screen',
            ),
            Node(
                package='emrmf_core',
                executable='gazebo_trajectory_node',
                namespace=robot_name,
                name='trajectory_commander',
                parameters=[{
                    'robot_id': robot_name,
                    'trajectory_id': index,
                    'linear_speed_mps': 0.20,
                }],
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
    world = PathJoinSubstitution([
        FindPackageShare('emrmf_core'),
        'worlds',
        'emrmf_office_indoor.world',
    ])
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([FindPackageShare('gazebo_ros'), 'launch', 'gazebo.launch.py']),
        ]),
        launch_arguments={'world': world, 'verbose': 'false'}.items(),
    )
    return LaunchDescription([
        DeclareLaunchArgument('robot_count', default_value='2', description='LIMO count: 0, 2, 3, 4, or 5.'),
        DeclareLaunchArgument('sensor_noise_std', default_value='0.02', description='Gaussian noise for LIMO odometry and LiDAR.'),
        DeclareLaunchArgument('trials', default_value='5', description='Trials per robot count for the scalability logger.'),
        ExecuteProcess(cmd=['bash', '-lc', 'echo Starting EMRMF LIMO Gazebo scalability launch'], output='screen'),
        gazebo,
        OpaqueFunction(function=_spawn_limo_robots),
        Node(
            package='emrmf_core',
            executable='lora_network_simulator',
            name='lora_network_simulator',
            output='screen',
        ),
        Node(
            package='emrmf_core',
            executable='trust_factor_node',
            name='trust_factor_node',
            output='screen',
        ),
        Node(
            package='emrmf_core',
            executable='global_fusion_node',
            name='global_fusion_node',
            output='screen',
        ),
        Node(
            package='emrmf_core',
            executable='scalability_experiment_logger_node',
            name='emrmf_scalability_logger',
            parameters=[{
                'trials': LaunchConfiguration('trials'),
                'sensor_noise_std': LaunchConfiguration('sensor_noise_std'),
            }],
            output='screen',
        ),
        Node(
            package='emrmf_core',
            executable='experiment_runner_node',
            name='emrmf_experiment_logger',
            output='screen',
        ),
    ])
