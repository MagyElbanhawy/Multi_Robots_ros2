"""Active EMRMF experiment orchestrator using subprocess-launched ROS 2 runs."""

from __future__ import annotations

import subprocess
import time
from typing import Sequence

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool

from emrmf_experiments.metrics import ABLATION_MODES, build_configurations


class ExperimentOrchestratorNode(Node):
    """Run repeated EMRMF validation configurations automatically."""

    def __init__(self) -> None:
        super().__init__('emrmf_experiment_orchestrator')
        self.declare_parameter('launch_package', 'emrmf_core')
        self.declare_parameter('launch_file', 'emrmf_sim.launch.py')
        self.declare_parameter('run_duration_s', 120.0)
        self.declare_parameter('use_experiment_done_topic', True)
        self.declare_parameter('repeated_runs', 5)
        self.declare_parameter('tau_e', 0.5)
        self.declare_parameter('p_values', [2.0, 3.0, 4.0])
        self.declare_parameter('gamma_values', [0.1, 0.3, 0.5, 1.0])
        self.declare_parameter('delay_values', [0.0, 0.5, 2.0])
        self.declare_parameter('packet_loss_values', [0.0, 0.10, 0.30])
        self.declare_parameter('sensor_noise_values', [0.0])
        self.declare_parameter('ablation_modes', list(ABLATION_MODES))
        self.experiment_done = False
        self.create_subscription(Bool, '/experiment_done', self._done_callback, 10)
        self.create_timer(1.0, self._run_all_once)
        self.started = False

    def _done_callback(self, message: Bool) -> None:
        self.experiment_done = bool(message.data)

    def _run_all_once(self) -> None:
        if self.started:
            return
        self.started = True
        configurations = build_configurations(
            modes=[str(value) for value in self.get_parameter('ablation_modes').value],
            p_values=[float(value) for value in self.get_parameter('p_values').value],
            gamma_values=[float(value) for value in self.get_parameter('gamma_values').value],
            delay_values=[float(value) for value in self.get_parameter('delay_values').value],
            packet_loss_values=[float(value) for value in self.get_parameter('packet_loss_values').value],
            sensor_noise_values=[float(value) for value in self.get_parameter('sensor_noise_values').value],
            tau_e=float(self.get_parameter('tau_e').value),
        )
        repeated_runs = int(self.get_parameter('repeated_runs').value)
        for configuration in configurations:
            for run_id in range(1, repeated_runs + 1):
                self._run_configuration(configuration, run_id)
        self.get_logger().info('Completed all EMRMF experiment configurations')

    def _run_configuration(self, configuration, run_id: int) -> None:
        self.experiment_done = False
        command = self._command_for_configuration(configuration, run_id)
        self.get_logger().info('Starting experiment: ' + ' '.join(command))
        process = subprocess.Popen(command)
        start_time = time.monotonic()
        run_duration = float(self.get_parameter('run_duration_s').value)
        use_done = bool(self.get_parameter('use_experiment_done_topic').value)
        while process.poll() is None:
            rclpy.spin_once(self, timeout_sec=0.1)
            elapsed = time.monotonic() - start_time
            if elapsed >= run_duration or (use_done and self.experiment_done):
                self._stop_process(process)
                break
        self.get_logger().info(f'Finished {configuration.configuration_id} run {run_id}')

    def _command_for_configuration(self, configuration, run_id: int) -> list[str]:
        return [
            'ros2',
            'launch',
            str(self.get_parameter('launch_package').value),
            str(self.get_parameter('launch_file').value),
            f'p:={configuration.p:g}',
            f'gamma:={configuration.gamma:g}',
            f'tau_e:={configuration.tau_e:g}',
            f'delay_s:={configuration.delay_s:g}',
            f'packet_loss:={configuration.packet_loss:g}',
            f'sensor_noise:={configuration.sensor_noise:g}',
            f'ablation_mode:={configuration.ablation_mode}',
            f'configuration_id:={configuration.configuration_id}',
            f'run_id:={run_id}',
        ]

    def _stop_process(self, process: subprocess.Popen) -> None:
        process.terminate()
        try:
            process.wait(timeout=10.0)
        except subprocess.TimeoutExpired:
            process.kill()
            process.wait(timeout=5.0)


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = ExperimentOrchestratorNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
