from __future__ import annotations

from bisect import bisect_left
from dataclasses import dataclass
import math

from wpimath.geometry import Pose2d, Rotation2d, Translation2d
from wpimath.kinematics import ChassisSpeeds


class _InterpolatingLookupTable:
    """Linear interpolation over distance-indexed lookup points."""

    def __init__(self) -> None:
        self._keys: list[float] = []
        self._values: list[float] = []

    def __len__(self) -> int:
        return len(self._keys)

    def put(self, key: float, value: float) -> None:
        index = bisect_left(self._keys, key)
        if index < len(self._keys) and self._keys[index] == key:
            self._values[index] = value
            return

        self._keys.insert(index, key)
        self._values.insert(index, value)

    def get(self, key: float) -> float | None:
        if not self._keys:
            return None

        index = bisect_left(self._keys, key)
        if index < len(self._keys) and self._keys[index] == key:
            return self._values[index]
        if index == 0:
            return self._values[0]
        if index >= len(self._keys):
            return self._values[-1]

        low_key = self._keys[index - 1]
        high_key = self._keys[index]
        low_value = self._values[index - 1]
        high_value = self._values[index]

        if high_key == low_key:
            return low_value

        blend = (key - low_key) / (high_key - low_key)
        return low_value + (high_value - low_value) * blend

    def clear(self) -> None:
        self._keys.clear()
        self._values.clear()


@dataclass
class Config:
    launcher_offset_x: float = 0.20
    launcher_offset_y: float = 0.0

    min_scoring_distance: float = 0.5
    max_scoring_distance: float = 5.0

    max_iterations: int = 25
    convergence_tolerance: float = 0.001
    tof_min: float = 0.05
    tof_max: float = 5.0

    min_sotm_speed: float = 0.1
    max_sotm_speed: float = 3.0

    phase_delay_ms: float = 30.0
    mech_latency_ms: float = 20.0

    sotm_drag_coeff: float = 0.47

    w_convergence: float = 1.0
    w_velocity_stability: float = 0.8
    w_vision_confidence: float = 1.2
    w_heading_accuracy: float = 1.5
    w_distance_in_range: float = 0.5
    heading_max_error_rad: float = math.radians(15.0)
    heading_speed_scalar: float = 1.0
    heading_reference_distance: float = 2.5

    max_tilt_deg: float = 5.0


@dataclass(frozen=True, slots=True)
class ShotInputs:
    robot_pose: Pose2d
    field_velocity: ChassisSpeeds
    robot_velocity: ChassisSpeeds
    hub_center: Translation2d
    hub_forward: Translation2d
    vision_confidence: float
    pitch_deg: float = 0.0
    roll_deg: float = 0.0


@dataclass(frozen=True, slots=True)
class LaunchParameters:
    rpm: float
    time_of_flight_sec: float
    drive_angle: Rotation2d
    drive_angular_velocity_rad_per_sec: float
    is_valid: bool
    confidence: float
    solved_distance_m: float
    iterations_used: int
    warm_start_used: bool


LaunchParameters.INVALID = LaunchParameters(
    rpm=0.0,
    time_of_flight_sec=0.0,
    drive_angle=Rotation2d(),
    drive_angular_velocity_rad_per_sec=0.0,
    is_valid=False,
    confidence=0.0,
    solved_distance_m=0.0,
    iterations_used=0,
    warm_start_used=False,
)


__all__ = ["Config", "LaunchParameters", "ShotInputs", "_InterpolatingLookupTable"]
