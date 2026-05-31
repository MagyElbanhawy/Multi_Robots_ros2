"""Offline ablation and sensitivity runner for reviewer-facing EMRMF evidence."""

from __future__ import annotations

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from emrmf_core.messages import encode, now_seconds
from emrmf_core.qos import telemetry_qos
from emrmf_core.ablation import generate_ablation_summary
from emrmf_core.robustness import generate_robustness_comparison
from emrmf_core.scalability import generate_scalability_summary


class ExperimentRunnerNode(Node):
    """Generate repeatable ablation and robustness metrics."""

    def __init__(self) -> None:
        super().__init__('experiment_runner_node')
        self.declare_parameter('runs', 5)
        self.declare_parameter('samples_per_run', 200)
        self.declare_parameter('gamma', 0.35)
        self.declare_parameter('exponents', [2.0, 3.0, 4.0])
        self.declare_parameter('packet_loss_rates', [0.0, 0.1, 0.3])
        self.declare_parameter('delay_levels_s', [0.5, 2.0, 5.0])
        self.declare_parameter('noise_levels_std', [0.03, 0.08, 0.16])
        self.declare_parameter('robot_counts', [2, 3, 4, 5])
        self.publisher = self.create_publisher(String, 'experiments/summary', telemetry_qos())
        self.timer = self.create_timer(1.0, self.run_once)
        self.completed = False

    def run_once(self) -> None:
        if self.completed:
            return
        self.completed = True
        robustness = generate_robustness_comparison(
            runs=int(self.get_parameter('runs').value),
            samples_per_run=int(self.get_parameter('samples_per_run').value),
            gamma=float(self.get_parameter('gamma').value),
            exponents=[float(value) for value in self.get_parameter('exponents').value],
            packet_loss_levels=[float(value) for value in self.get_parameter('packet_loss_rates').value],
            delay_levels_s=[float(value) for value in self.get_parameter('delay_levels_s').value],
            noise_levels_std=[float(value) for value in self.get_parameter('noise_levels_std').value],
        )
        ablation = generate_ablation_summary(
            runs=int(self.get_parameter('runs').value),
            samples_per_run=int(self.get_parameter('samples_per_run').value),
            gamma=float(self.get_parameter('gamma').value),
            packet_loss_levels=[float(value) for value in self.get_parameter('packet_loss_rates').value],
            delay_levels_s=[float(value) for value in self.get_parameter('delay_levels_s').value],
            noise_levels_std=[float(value) for value in self.get_parameter('noise_levels_std').value],
        )
        scalability = generate_scalability_summary(
            robot_counts=[int(value) for value in self.get_parameter('robot_counts').value],
            trials=int(self.get_parameter('runs').value),
            samples_per_robot=int(self.get_parameter('samples_per_run').value),
            sensor_noise_std=float(self.get_parameter('noise_levels_std').value[0]),
            gamma=float(self.get_parameter('gamma').value),
        )
        result = {
            'type': 'experiment_summary',
            'stamp': now_seconds(),
            'robustness_comparison': robustness,
            'final_ablation_summary': ablation,
            'scalability_robot_count_summary': scalability,
        }
        msg = String()
        msg.data = encode(result)
        self.publisher.publish(msg)
        self.get_logger().info(msg.data)


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = ExperimentRunnerNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
