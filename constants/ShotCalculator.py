import math
from typing import Final

from wpimath import angleModulus
from wpimath.geometry import Rotation2d, Transform2d, Translation2d, Twist2d

from constants.physics import calc_shot_profile
from constants.shot_calculator_support import (
    Config,
    LaunchParameters,
    ShotInputs,
    _InterpolatingLookupTable,
)

# Temporary empirical seed table for the hybrid shot model.
# Each tuple is: (distance_m, flywheel_rps, time_of_flight_sec).
# These values are absolute shot targets, and load_lut_entry() converts them
# into residual corrections against the physics baseline.
_DEFAULT_CALIBRATION_ENTRY_LABELS: Final[tuple[str, str, str]] = (
    "distance_m",
    "flywheel_rps",
    "time_of_flight_sec",
)
_DEFAULT_CALIBRATION_ENTRIES: Final[tuple[tuple[float, float, float], ...]] = (
    (0.500000000000, 19.075039701033, 0.214595846814),
    (0.821428571429, 30.179703655724, 0.222828939040),
    (1.142857142857, 40.313893530590, 0.232088681475),
    (1.464285714286, 49.412599542271, 0.242607868321),
    (1.785714285714, 57.398264920801, 0.254700599605),
    (2.107142857143, 64.176622193444, 0.268802859211),
    (2.428571428571, 69.630456063661, 0.285540950219),
    (2.750000000000, 73.609828825519, 0.305853633901),
    (3.071428571429, 75.915872239012, 0.331226131558),
    (3.392857142857, 76.271882389781, 0.364181489479),
    (3.714285714286, 74.266475704167, 0.409448469101),
    (4.035714285714, 69.225061400513, 0.477280643204),
    (4.357142857143, 59.847811470644, 0.596032971015),
    (4.678571428571, 42.650029386438, 0.898071032519),
    (5.000000000000, 23.696753157268, 1.727419900308),
)


class ShotCalculator:
    """Shoot-on-the-move solver ported from the original Java implementation."""

    LOOP_PERIOD_SEC: Final[float] = 0.02
    DERIVATIVE_STEP_METERS: Final[float] = 0.01

    def __init__(self, config: Config | None = None) -> None:
        self.config = config if config is not None else Config()

        # Residual tables store empirical correction on top of the physics
        # baseline, indexed by shooting distance in meters.
        self._rpm_residual_map = _InterpolatingLookupTable()
        self._tof_residual_map = _InterpolatingLookupTable()

        self._previous_tof = -1.0
        self._previous_speed = 0.0

        self._prev_robot_vx = 0.0
        self._prev_robot_vy = 0.0
        self._prev_robot_omega = 0.0

        for distance_m, flywheel_rps, time_of_flight_sec in _DEFAULT_CALIBRATION_ENTRIES:
            self.load_lut_entry(distance_m, flywheel_rps, time_of_flight_sec)

    @staticmethod
    def clamp(value: float, low: float, high: float) -> float:
        return max(low, min(high, value))

    def _launcher_transform(self) -> Transform2d:
        return Transform2d(
            Translation2d(self.config.launcher_offset_x, self.config.launcher_offset_y),
            Rotation2d(),
        )

    def _lookup_required(self, table: _InterpolatingLookupTable, distance: float, label: str) -> float:
        value = table.get(distance)
        if value is None:
            raise ValueError(f"{label} lookup table is empty")
        return value

    def _physics_rpm(self, distance: float) -> float:
        rpm, _ = calc_shot_profile(distance)
        return rpm

    def _physics_tof(self, distance: float) -> float:
        _, tof = calc_shot_profile(distance)
        return tof

    def load_lut_entry(self, distance_m: float, rpm: float, tof: float) -> None:
        # The lookup table is defined as absolute empirical shot targets:
        # distance_m -> (flywheel_rps, time_of_flight_sec).
        # Internally we store those values as residuals so the calculator uses
        # physics baseline + interpolated empirical correction.
        physics_rpm = self._physics_rpm(distance_m)
        physics_tof = self._physics_tof(distance_m)

        self._rpm_residual_map.put(distance_m, rpm - physics_rpm)
        self._tof_residual_map.put(distance_m, tof - physics_tof)

    def _effective_rpm(self, distance: float) -> float:
        return self._physics_rpm(distance) + self._lookup_required(
            self._rpm_residual_map,
            distance,
            "RPM residual",
        )

    def _effective_tof(self, distance: float) -> float:
        return self._physics_tof(distance) + self._lookup_required(
            self._tof_residual_map,
            distance,
            "TOF residual",
        )

    def _drag_compensated_tof(self, tof: float) -> float:
        if self.config.sotm_drag_coeff < 1e-6:
            return tof

        return (1.0 - math.exp(-self.config.sotm_drag_coeff * tof)) / self.config.sotm_drag_coeff

    def _tof_map_derivative(self, distance: float) -> float:
        high = self._effective_tof(distance + self.DERIVATIVE_STEP_METERS)
        low = self._effective_tof(distance - self.DERIVATIVE_STEP_METERS)
        return (high - low) / (2.0 * self.DERIVATIVE_STEP_METERS)

    def calculate(self, inputs: ShotInputs | None) -> LaunchParameters:
        if (
            inputs is None
            or inputs.robot_pose is None
            or inputs.field_velocity is None
            or inputs.robot_velocity is None
            or len(self._rpm_residual_map) == 0
            or len(self._tof_residual_map) == 0
        ):
            return LaunchParameters.INVALID

        raw_pose = inputs.robot_pose
        if not math.isfinite(raw_pose.x) or not math.isfinite(raw_pose.y):
            return LaunchParameters.INVALID

        dt = self.config.phase_delay_ms / 1000.0
        robot_velocity = inputs.robot_velocity
        field_velocity = inputs.field_velocity

        ax = (robot_velocity.vx - self._prev_robot_vx) / self.LOOP_PERIOD_SEC
        ay = (robot_velocity.vy - self._prev_robot_vy) / self.LOOP_PERIOD_SEC
        angular_accel = (robot_velocity.omega - self._prev_robot_omega) / self.LOOP_PERIOD_SEC

        compensated_pose = raw_pose.exp(
            Twist2d(
                robot_velocity.vx * dt + 0.5 * ax * dt * dt,
                robot_velocity.vy * dt + 0.5 * ay * dt * dt,
                robot_velocity.omega * dt + 0.5 * angular_accel * dt * dt,
            )
        )

        self._prev_robot_vx = robot_velocity.vx
        self._prev_robot_vy = robot_velocity.vy
        self._prev_robot_omega = robot_velocity.omega

        robot_translation = compensated_pose.translation()
        heading = compensated_pose.rotation()
        to_hub_from_robot = inputs.hub_center - robot_translation

        if (
            to_hub_from_robot.x * inputs.hub_forward.x
            + to_hub_from_robot.y * inputs.hub_forward.y
        ) < 0.0:
            return LaunchParameters.INVALID

        if (
            abs(inputs.pitch_deg) > self.config.max_tilt_deg
            or abs(inputs.roll_deg) > self.config.max_tilt_deg
        ):
            return LaunchParameters.INVALID

        launcher_pose = compensated_pose.transformBy(self._launcher_transform())
        launcher_translation = launcher_pose.translation()
        launcher_offset_field = launcher_translation - robot_translation

        launcher_velocity = Translation2d(
            field_velocity.vx - launcher_offset_field.y * field_velocity.omega,
            field_velocity.vy + launcher_offset_field.x * field_velocity.omega,
        )

        displacement_to_hub = inputs.hub_center - launcher_translation
        distance = displacement_to_hub.norm()
        if (
            distance < self.config.min_scoring_distance
            or distance > self.config.max_scoring_distance
        ):
            return LaunchParameters.INVALID

        robot_speed = launcher_velocity.norm()
        if robot_speed > self.config.max_sotm_speed:
            return LaunchParameters.INVALID

        velocity_filtered = robot_speed < self.config.min_sotm_speed
        projected_distance = distance
        iterations_used = 0
        warm_start_used = False

        try:
            if velocity_filtered:
                solved_tof = self._effective_tof(distance)
            else:
                solved_tof = (
                    self._previous_tof
                    if self._previous_tof > 0.0
                    else self._effective_tof(distance)
                )
                warm_start_used = self._previous_tof > 0.0

                for iteration in range(self.config.max_iterations):
                    previous_tof = solved_tof

                    if self.config.sotm_drag_coeff < 1e-6:
                        drag_exp = 1.0
                        drift_tof = solved_tof
                    else:
                        drag_exp = math.exp(-self.config.sotm_drag_coeff * solved_tof)
                        drift_tof = (1.0 - drag_exp) / self.config.sotm_drag_coeff

                    projected_dx = displacement_to_hub.x - launcher_velocity.x * drift_tof
                    projected_dy = displacement_to_hub.y - launcher_velocity.y * drift_tof
                    projected_distance = math.hypot(projected_dx, projected_dy)

                    if projected_distance < 0.01:
                        solved_tof = self._effective_tof(distance)
                        projected_distance = distance
                        iterations_used = self.config.max_iterations + 1
                        break

                    lookup_tof = self._effective_tof(projected_distance)
                    distance_rate = -drag_exp * (
                        projected_dx * launcher_velocity.x
                        + projected_dy * launcher_velocity.y
                    ) / projected_distance
                    tof_rate = self._tof_map_derivative(projected_distance)
                    residual = lookup_tof - solved_tof
                    residual_rate = tof_rate * distance_rate - 1.0

                    if abs(residual_rate) > 0.01:
                        solved_tof -= residual / residual_rate
                    else:
                        solved_tof = lookup_tof

                    solved_tof = self.clamp(solved_tof, self.config.tof_min, self.config.tof_max)
                    iterations_used = iteration + 1

                    if abs(solved_tof - previous_tof) < self.config.convergence_tolerance:
                        break

                if not math.isfinite(solved_tof) or solved_tof < 0.0 or solved_tof > self.config.tof_max:
                    solved_tof = self._effective_tof(distance)
                    projected_distance = distance
                    iterations_used = self.config.max_iterations + 1
        except ValueError:
            return LaunchParameters.INVALID

        self._previous_tof = solved_tof

        effective_time_of_flight = solved_tof + self.config.mech_latency_ms / 1000.0
        rpm = self._effective_rpm(projected_distance)

        if velocity_filtered:
            compensated_target = inputs.hub_center
        else:
            compensated_target = inputs.hub_center - launcher_velocity * self._drag_compensated_tof(solved_tof)

        aim_vector = compensated_target - robot_translation
        drive_angle = Rotation2d(aim_vector.x, aim_vector.y)
        heading_error_rad = angleModulus(drive_angle.radians() - heading.radians())

        drive_angular_velocity = 0.0
        if not velocity_filtered and distance > 0.1:
            tangential_velocity = (
                displacement_to_hub.x * launcher_velocity.y
                - displacement_to_hub.y * launcher_velocity.x
            ) / distance
            drive_angular_velocity = tangential_velocity / distance

        if velocity_filtered:
            solver_quality = 1.0
        elif iterations_used > self.config.max_iterations:
            solver_quality = 0.0
        elif iterations_used <= 3:
            solver_quality = 1.0
        else:
            interpolation = (iterations_used - 3) / (self.config.max_iterations - 3)
            solver_quality = 1.0 + (0.1 - 1.0) * self.clamp(interpolation, 0.0, 1.0)

        confidence = self.compute_confidence(
            solver_quality=solver_quality,
            current_speed=robot_speed,
            heading_error_rad=heading_error_rad,
            distance=distance,
            vision_confidence=inputs.vision_confidence,
        )

        self._previous_speed = robot_speed

        return LaunchParameters(
            rpm=rpm,
            time_of_flight_sec=effective_time_of_flight,
            drive_angle=drive_angle,
            drive_angular_velocity_rad_per_sec=drive_angular_velocity,
            is_valid=True,
            confidence=confidence,
            solved_distance_m=distance,
            iterations_used=iterations_used,
            warm_start_used=warm_start_used,
        )

    def compute_confidence(
        self,
        solver_quality: float,
        current_speed: float,
        heading_error_rad: float,
        distance: float,
        vision_confidence: float,
    ) -> float:
        speed_delta = abs(current_speed - self._previous_speed)
        velocity_stability = self.clamp(1.0 - speed_delta / 0.5, 0.0, 1.0)
        vision_quality = self.clamp(vision_confidence, 0.0, 1.0)

        distance_scale = self.clamp(
            self.config.heading_reference_distance / distance if distance > 1e-9 else 2.0,
            0.5,
            2.0,
        )
        speed_scale = 1.0 / (1.0 + self.config.heading_speed_scalar * current_speed)
        scaled_max_error = self.config.heading_max_error_rad * distance_scale * speed_scale
        if scaled_max_error <= 1e-9:
            heading_accuracy = 1.0 if abs(heading_error_rad) <= 1e-9 else 0.0
        else:
            heading_accuracy = self.clamp(
                1.0 - abs(heading_error_rad) / scaled_max_error,
                0.0,
                1.0,
            )

        range_span = self.config.max_scoring_distance - self.config.min_scoring_distance
        if range_span <= 1e-9:
            distance_in_range = 1.0
        else:
            range_fraction = (distance - self.config.min_scoring_distance) / range_span
            distance_in_range = self.clamp(
                1.0 - 2.0 * abs(range_fraction - 0.5),
                0.0,
                1.0,
            )

        components = (
            solver_quality,
            velocity_stability,
            vision_quality,
            heading_accuracy,
            distance_in_range,
        )
        weights = (
            self.config.w_convergence,
            self.config.w_velocity_stability,
            self.config.w_vision_confidence,
            self.config.w_heading_accuracy,
            self.config.w_distance_in_range,
        )

        weight_sum = 0.0
        weighted_log_sum = 0.0
        for component, weight in zip(components, weights, strict=True):
            if component <= 0.0:
                return 0.0
            weighted_log_sum += weight * math.log(component)
            weight_sum += weight

        if weight_sum <= 0.0:
            return 0.0

        return self.clamp(math.exp(weighted_log_sum / weight_sum) * 100.0, 0.0, 100.0)

    def reset_warm_start(self) -> None:
        self._previous_tof = -1.0
        self._previous_speed = 0.0
        self._prev_robot_vx = 0.0
        self._prev_robot_vy = 0.0
        self._prev_robot_omega = 0.0
