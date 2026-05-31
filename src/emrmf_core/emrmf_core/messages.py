"""JSON message helpers shared by EMRMF ROS 2 nodes."""

from __future__ import annotations

import json
import math
import time
from typing import Any, Dict, Iterable, List, Mapping, MutableMapping, Tuple

Pose2D = Tuple[float, float, float]


def now_seconds() -> float:
    """Return a wall-clock timestamp for simulation metadata."""

    return time.time()


def encode(payload: Mapping[str, Any]) -> str:
    """Encode payload as compact deterministic JSON for std_msgs/String."""

    return json.dumps(payload, sort_keys=True, separators=(",", ":"))


def decode(data: str) -> Dict[str, Any]:
    """Decode a JSON String payload, returning an empty dict on malformed input."""

    try:
        value = json.loads(data)
    except json.JSONDecodeError:
        return {}
    return value if isinstance(value, dict) else {}


def clamp(value: float, lower: float = 0.0, upper: float = 1.0) -> float:
    """Clamp a numeric value into an inclusive interval."""

    return max(lower, min(upper, value))


def pose_distance(a: Iterable[float], b: Iterable[float]) -> float:
    """Euclidean translational distance between two x/y/yaw poses."""

    ax, ay, *_ = list(a)
    bx, by, *_ = list(b)
    return math.hypot(ax - bx, ay - by)


def weighted_pose_average(items: Iterable[Mapping[str, Any]]) -> Pose2D:
    """Average pose estimates using trust weights; falls back to uniform weights."""

    sx = sy = sc = ss = total = 0.0
    for item in items:
        pose = item.get("pose", [0.0, 0.0, 0.0])
        weight = float(item.get("theta", item.get("weight", 1.0)))
        if weight <= 0.0:
            continue
        sx += weight * float(pose[0])
        sy += weight * float(pose[1])
        sc += weight * math.cos(float(pose[2]))
        ss += weight * math.sin(float(pose[2]))
        total += weight
    if total <= 0.0:
        return (0.0, 0.0, 0.0)
    return (sx / total, sy / total, math.atan2(ss / total, sc / total))


def merge_landmarks(observations: Iterable[Mapping[str, Any]]) -> List[Dict[str, float]]:
    """Fuse landmark points by id using trust-weighted averaging."""

    buckets: MutableMapping[str, Dict[str, float]] = {}
    for observation in observations:
        theta = float(observation.get("theta", observation.get("weight", 1.0)))
        for landmark in observation.get("landmarks", []):
            key = str(landmark.get("id", "unknown"))
            bucket = buckets.setdefault(key, {"x": 0.0, "y": 0.0, "w": 0.0})
            bucket["x"] += theta * float(landmark.get("x", 0.0))
            bucket["y"] += theta * float(landmark.get("y", 0.0))
            bucket["w"] += theta
    fused: List[Dict[str, float]] = []
    for key, bucket in sorted(buckets.items()):
        weight = bucket["w"] or 1.0
        fused.append({"id": key, "x": bucket["x"] / weight, "y": bucket["y"] / weight})
    return fused
