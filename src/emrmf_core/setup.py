from glob import glob
from setuptools import find_packages, setup

package_name = 'emrmf_core'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/worlds', glob('worlds/*.world')),
        ('share/' + package_name + '/urdf', glob('urdf/*.xacro')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='EMRMF Research Team',
    maintainer_email='research@example.com',
    description='Trust-aware hybrid/decentralized map fusion nodes for ROS 2.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'robot_slam_node = emrmf_core.robot_slam_node:main',
            'lora_network_simulator = emrmf_core.lora_network_simulator:main',
            'trust_factor_node = emrmf_core.trust_factor_node:main',
            'decentralized_map_node = emrmf_core.decentralized_map_node:main',
            'global_fusion_node = emrmf_core.global_fusion_node:main',
            'adversarial_monitor_node = emrmf_core.adversarial_monitor_node:main',
            'experiment_runner_node = emrmf_core.experiment_runner_node:main',
            'gazebo_trajectory_node = emrmf_core.gazebo_trajectory_node:main',
            'gazebo_emrmf_bridge_node = emrmf_core.gazebo_emrmf_bridge_node:main',
            'scalability_experiment_logger_node = emrmf_core.scalability_experiment_logger_node:main',
        ],
    },
)
