"""Pure-Python control and validation primitives.

This module intentionally has no ROS dependency, so the safety logic can be unit-tested
without a running simulator.
"""

from __future__ import annotations

from collections import deque
from dataclasses import dataclass
import math
from typing import Deque, Optional, Tuple


@dataclass(frozen=True)
class Point3:
    x: float
    y: float
    z: float

    def distance_xy(self, other: "Point3") -> float:
        return math.hypot(self.x - other.x, self.y - other.y)

    def distance_3d(self, other: "Point3") -> float:
        return math.sqrt(
            (self.x - other.x) ** 2
            + (self.y - other.y) ** 2
            + (self.z - other.z) ** 2
        )


@dataclass(frozen=True)
class PositionDecision:
    accepted: bool
    point: Optional[Point3]
    jump_m: float
    reason: str


class PositionValidator:
    """Rejects non-finite samples and one-step position jumps."""

    def __init__(self, max_jump_m: float) -> None:
        if max_jump_m <= 0.0:
            raise ValueError("max_jump_m must be positive")
        self.max_jump_m = float(max_jump_m)
        self.last_valid: Optional[Point3] = None

    def update(self, point: Point3) -> PositionDecision:
        if not all(math.isfinite(value) for value in (point.x, point.y, point.z)):
            return PositionDecision(False, self.last_valid, math.inf, "non_finite")

        if self.last_valid is None:
            self.last_valid = point
            return PositionDecision(True, point, 0.0, "first_sample")

        jump_m = point.distance_3d(self.last_valid)
        if jump_m > self.max_jump_m:
            return PositionDecision(False, self.last_valid, jump_m, "jump")

        self.last_valid = point
        return PositionDecision(True, point, jump_m, "accepted")


class FollowerPlanner:
    """Computes a fixed-distance waypoint behind the car's direction of travel."""

    def __init__(
        self,
        offset_m: float,
        altitude_m: float,
        min_heading_speed_mps: float,
    ) -> None:
        if offset_m <= 0.0:
            raise ValueError("offset_m must be positive")
        if altitude_m <= 0.0:
            raise ValueError("altitude_m must be positive")
        if min_heading_speed_mps < 0.0:
            raise ValueError("min_heading_speed_mps cannot be negative")
        self.offset_m = float(offset_m)
        self.altitude_m = float(altitude_m)
        self.min_heading_speed_mps = float(min_heading_speed_mps)
        self._last_heading: Tuple[float, float] = (1.0, 0.0)

    def waypoint(
        self,
        current: Point3,
        previous: Optional[Point3],
        dt_s: Optional[float],
    ) -> Point3:
        heading_x, heading_y = self._last_heading

        if previous is not None and dt_s is not None and dt_s > 0.0:
            vx = (current.x - previous.x) / dt_s
            vy = (current.y - previous.y) / dt_s
            speed = math.hypot(vx, vy)
            if speed >= self.min_heading_speed_mps:
                heading_x = vx / speed
                heading_y = vy / speed
                self._last_heading = (heading_x, heading_y)

        return Point3(
            x=current.x - self.offset_m * heading_x,
            y=current.y - self.offset_m * heading_y,
            z=self.altitude_m,
        )


class StaleDetector:
    """Uses monotonic time to detect missing updates."""

    def __init__(self, timeout_s: float) -> None:
        if timeout_s <= 0.0:
            raise ValueError("timeout_s must be positive")
        self.timeout_s = float(timeout_s)
        self.last_update_monotonic: Optional[float] = None

    def mark(self, now_monotonic: float) -> None:
        self.last_update_monotonic = float(now_monotonic)

    def is_stale(self, now_monotonic: float) -> bool:
        if self.last_update_monotonic is None:
            return True
        return (float(now_monotonic) - self.last_update_monotonic) > self.timeout_s


class ArmRetryPolicy:
    """Tracks an initial arming command followed by a configured number of retries."""

    def __init__(self, retry_count: int, retry_interval_s: float) -> None:
        if retry_count < 0:
            raise ValueError("retry_count cannot be negative")
        if retry_interval_s <= 0.0:
            raise ValueError("retry_interval_s must be positive")
        self.retry_count = int(retry_count)
        self.retry_interval_s = float(retry_interval_s)
        self.commands_sent = 0
        self.last_command_monotonic: Optional[float] = None

    @property
    def max_commands(self) -> int:
        return 1 + self.retry_count

    def should_send(self, now_monotonic: float) -> bool:
        if self.commands_sent >= self.max_commands:
            return False
        if self.last_command_monotonic is None:
            return True
        return (float(now_monotonic) - self.last_command_monotonic) >= self.retry_interval_s

    def record_command(self, now_monotonic: float) -> int:
        if self.commands_sent >= self.max_commands:
            raise RuntimeError("arming command budget exhausted")
        self.commands_sent += 1
        self.last_command_monotonic = float(now_monotonic)
        return self.commands_sent

    def exhausted_after_wait(self, now_monotonic: float) -> bool:
        if self.commands_sent < self.max_commands:
            return False
        if self.last_command_monotonic is None:
            return False
        return (float(now_monotonic) - self.last_command_monotonic) >= self.retry_interval_s


class RtfEstimator:
    """Sliding-window Gazebo real-time-factor estimator from /clock samples."""

    def __init__(self, window_s: float) -> None:
        if window_s <= 0.0:
            raise ValueError("window_s must be positive")
        self.window_s = float(window_s)
        self._samples: Deque[Tuple[float, float]] = deque()
        self._latest_sim_s: Optional[float] = None

    def add(self, wall_monotonic_s: float, sim_s: float) -> None:
        wall = float(wall_monotonic_s)
        sim = float(sim_s)
        self._latest_sim_s = sim
        self._samples.append((wall, sim))
        self._trim(wall)

    def estimate(self, wall_monotonic_s: float) -> Optional[float]:
        if self._latest_sim_s is None or not self._samples:
            return None
        wall = float(wall_monotonic_s)
        self._trim(wall)
        first_wall, first_sim = self._samples[0]
        wall_dt = wall - first_wall
        if wall_dt <= 1e-6:
            return None
        sim_dt = self._latest_sim_s - first_sim
        return max(0.0, sim_dt / wall_dt)

    def _trim(self, wall_now: float) -> None:
        cutoff = wall_now - self.window_s
        while len(self._samples) > 1 and self._samples[1][0] <= cutoff:
            self._samples.popleft()


def enu_to_ned(point: Point3) -> Point3:
    """ROS ENU (east, north, up) to PX4 NED (north, east, down)."""
    return Point3(x=point.y, y=point.x, z=-point.z)


def ned_to_enu(point: Point3) -> Point3:
    """PX4 NED (north, east, down) to ROS ENU (east, north, up)."""
    return Point3(x=point.y, y=point.x, z=-point.z)
