"""Trust factor computation for EMRMF sensor/network weighting."""

from __future__ import annotations

import math
from typing import Dict

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from emrmf_core.messages import clamp, decode, encode, now_seconds
from emrmf_core.qos import telemetry_qos


class TrustFactorNode(Node):
    """Compute theta from sensor reliability and communication aging."""

    def __init__(self) -> None:
        super().__init__('trust_factor_node')
        self.declare_parameter('reference_noise_std', 0.05)
        self.declare_parameter('reference_latency_s', 0.5)
        self.declare_parameter('cubic_exponent', 3.0)
        self.declare_parameter('gamma', 0.35)
        self.declare_parameter('low_trust_threshold', 0.35)
        self.declare_parameter('min_theta', 0.02)
        self.declare_parameter('publish_diagnostics', True)

        self.last_theta: Dict[str, float] = {}
        self.subscription = self.create_subscription(String, 'lora/observations', self.score_observation, telemetry_qos())
        self.publisher = self.create_publisher(String, 'trust/weighted_observations', telemetry_qos())
        self.diagnostics = self.create_publisher(String, 'trust/diagnostics', telemetry_qos())

    def compute_theta(self, noise_std: float, latency_s: float) -> tuple[float, Dict[str, float]]:
        reference_noise = max(float(self.get_parameter('reference_noise_std').value), 1.0e-6)
        reference_latency = max(float(self.get_parameter('reference_latency_s').value), 1.0e-6)
        exponent = max(float(self.get_parameter('cubic_exponent').value), 0.1)
        gamma = max(float(self.get_parameter('gamma').value), 0.0)
        min_theta = clamp(float(self.get_parameter('min_theta').value), 0.0, 1.0)

        sensor_ratio = reference_noise / (reference_noise + max(noise_std, 0.0))
        latency_ratio = reference_latency / (reference_latency + max(latency_s, 0.0))
        sensor_component = math.pow(clamp(sensor_ratio), exponent)
        temporal_component = math.exp(-gamma * max(latency_s, 0.0)) * latency_ratio
        theta = clamp(max(min_theta, sensor_component * temporal_component))
        return theta, {
            'sensor_ratio': sensor_ratio,
            'latency_ratio': latency_ratio,
            'sensor_component': sensor_component,
            'temporal_component': temporal_component,
            'gamma': gamma,
            'exponent': exponent,
        }

    def score_observation(self, msg: String) -> None:
        payload = decode(msg.data)
        if not payload:
            return
        latency = float(payload.get('network_delay_s', payload.get('latency_hint_s', 0.0)))
        noise = float(payload.get('sensor_noise_std', 0.05))
        theta, components = self.compute_theta(noise, latency)
        low_trust_threshold = float(self.get_parameter('low_trust_threshold').value)
        robot_id = str(payload.get('robot_id', 'unknown'))
        self.last_theta[robot_id] = theta
        payload['theta'] = theta
        payload['trust_components'] = components
        payload['low_trust'] = theta < low_trust_threshold
        payload['trust_stamp'] = now_seconds()

        outgoing = String()
        outgoing.data = encode(payload)
        self.publisher.publish(outgoing)

        if bool(self.get_parameter('publish_diagnostics').value):
            diagnostic = String()
            diagnostic.data = encode({
                'type': 'trust_diagnostic',
                'robot_id': robot_id,
                'theta': theta,
                'low_trust': payload['low_trust'],
                **components,
            })
            self.diagnostics.publish(diagnostic)


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = TrustFactorNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
