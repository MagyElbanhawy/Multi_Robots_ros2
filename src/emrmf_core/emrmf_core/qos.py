"""ROS 2 QoS profiles used by EMRMF topics."""

from rclpy.duration import Duration
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy


def telemetry_qos(depth: int = 10, deadline_ms: int = 500) -> QoSProfile:
    """Reliable volatile telemetry profile with bounded history and deadline."""

    return QoSProfile(
        history=HistoryPolicy.KEEP_LAST,
        depth=depth,
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.VOLATILE,
        deadline=Duration(seconds=0, nanoseconds=deadline_ms * 1_000_000),
    )
