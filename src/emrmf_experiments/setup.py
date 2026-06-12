from glob import glob
from setuptools import find_packages, setup

package_name = 'emrmf_experiments'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='The Author',
    maintainer_email='author@example.invalid',
    description='Experiment validation package for EMRMF multi-robot SLAM.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'communication_proxy_node = emrmf_experiments.message_proxy_node:main',
            'experiment_logger_node = emrmf_experiments.experiment_logger_node:main',
            'experiment_orchestrator_node = emrmf_experiments.experiment_orchestrator_node:main',
        ],
    },
)
