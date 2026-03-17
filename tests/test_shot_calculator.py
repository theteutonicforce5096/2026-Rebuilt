import math
from pathlib import Path
import sys

from wpimath.geometry import Pose2d, Rotation2d, Translation2d
from wpimath.kinematics import ChassisSpeeds

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from constants.ShotCalculator import Config, LaunchParameters, ShotCalculator, ShotInputs


def _build_calculator() -> ShotCalculator:
    config = Config(
        min_scoring_distance=0.0,
        max_scoring_distance=10.0,
        phase_delay_ms=0.0,
        mech_latency_ms=0.0,
        sotm_drag_coeff=0.0,
    )
    calculator = ShotCalculator(config)
    calculator.load_lut_entry(1.0, 1000.0, 0.25)
    calculator.load_lut_entry(3.0, 3000.0, 0.75)
    calculator.load_lut_entry(5.0, 5000.0, 1.25)
    return calculator


def test_stationary_shot_uses_rotated_launcher_transform() -> None:
    calculator = _build_calculator()
    calculator.config.launcher_offset_x = 0.5

    result = calculator.calculate(
        ShotInputs(
            robot_pose=Pose2d(1.0, 2.0, Rotation2d.fromDegrees(90.0)),
            field_velocity=ChassisSpeeds(0.0, 0.0, 0.0),
            robot_velocity=ChassisSpeeds(0.0, 0.0, 0.0),
            hub_center=Translation2d(1.0, 4.5),
            hub_forward=Translation2d(0.0, 1.0),
            vision_confidence=1.0,
        )
    )

    assert result.is_valid
    assert math.isclose(result.solved_distance_m, 2.0, abs_tol=1e-9)
    assert math.isclose(result.time_of_flight_sec, 0.5, abs_tol=1e-9)
    assert math.isclose(result.rpm, 2000.0, abs_tol=1e-9)
    assert math.isclose(result.drive_angle.degrees(), 90.0, abs_tol=1e-9)
    assert result.iterations_used == 0
    assert not result.warm_start_used


def test_moving_shot_leads_target_and_reuses_warm_start() -> None:
    calculator = _build_calculator()

    inputs = ShotInputs(
        robot_pose=Pose2d(0.0, 0.0, Rotation2d()),
        field_velocity=ChassisSpeeds(0.0, 1.0, 0.0),
        robot_velocity=ChassisSpeeds(0.0, 1.0, 0.0),
        hub_center=Translation2d(4.0, 0.0),
        hub_forward=Translation2d(1.0, 0.0),
        vision_confidence=1.0,
    )

    first = calculator.calculate(inputs)
    second = calculator.calculate(inputs)

    assert first.is_valid
    assert first.drive_angle.degrees() < 0.0
    assert first.iterations_used > 0

    assert second.is_valid
    assert second.warm_start_used
    assert second.drive_angle.degrees() < 0.0
    assert math.isclose(second.drive_angle.radians(), first.drive_angle.radians(), abs_tol=1e-6)


def test_calculate_rejects_behind_hub_and_tilted_shots() -> None:
    calculator = _build_calculator()

    behind_hub = calculator.calculate(
        ShotInputs(
            robot_pose=Pose2d(5.0, 0.0, Rotation2d()),
            field_velocity=ChassisSpeeds(0.0, 0.0, 0.0),
            robot_velocity=ChassisSpeeds(0.0, 0.0, 0.0),
            hub_center=Translation2d(4.0, 0.0),
            hub_forward=Translation2d(1.0, 0.0),
            vision_confidence=1.0,
        )
    )

    tilted = calculator.calculate(
        ShotInputs(
            robot_pose=Pose2d(0.0, 0.0, Rotation2d()),
            field_velocity=ChassisSpeeds(0.0, 0.0, 0.0),
            robot_velocity=ChassisSpeeds(0.0, 0.0, 0.0),
            hub_center=Translation2d(2.0, 0.0),
            hub_forward=Translation2d(1.0, 0.0),
            vision_confidence=1.0,
            pitch_deg=calculator.config.max_tilt_deg + 0.1,
        )
    )

    assert not behind_hub.is_valid
    assert behind_hub == LaunchParameters.INVALID
    assert not tilted.is_valid
    assert tilted == LaunchParameters.INVALID


def test_public_api_surface_is_minimal() -> None:
    calculator = _build_calculator()
    config = Config()
    result = LaunchParameters.INVALID

    assert hasattr(calculator, "load_lut_entry")
    assert hasattr(calculator, "calculate")
    assert hasattr(calculator, "compute_confidence")
    assert hasattr(config, "launcher_offset_x")
    assert hasattr(config, "phase_delay_ms")
    assert hasattr(result, "is_valid")
    assert hasattr(result, "confidence")
    assert hasattr(result, "rpm")
    assert hasattr(result, "drive_angle")

    assert not hasattr(calculator, "effective_rpm")
    assert not hasattr(calculator, "effective_tof")
    assert not hasattr(calculator, "drag_compensated_tof")
    assert not hasattr(calculator, "tof_map_derivative")
    assert not hasattr(calculator, "add_rpm_correction")
    assert not hasattr(calculator, "add_tof_correction")
    assert not hasattr(calculator, "clear_corrections")
    assert not hasattr(calculator, "adjust_offset")
    assert not hasattr(calculator, "reset_offset")
    assert not hasattr(calculator, "get_offset")
    assert not hasattr(calculator, "get_time_of_flight")
    assert not hasattr(calculator, "get_base_rpm")
    assert not hasattr(calculator, "reset_warm_start")
    assert not hasattr(calculator, "get_rpm_map")
    assert not hasattr(calculator, "get_tof_map")


def test_regression_outputs_match_pre_cleanup_baseline() -> None:
    calculator = _build_calculator()

    scenarios = (
        (
            ShotInputs(
                robot_pose=Pose2d(1.0, 2.0, Rotation2d.fromDegrees(90.0)),
                field_velocity=ChassisSpeeds(0.0, 0.0, 0.0),
                robot_velocity=ChassisSpeeds(0.0, 0.0, 0.0),
                hub_center=Translation2d(1.0, 4.5),
                hub_forward=Translation2d(0.0, 1.0),
                vision_confidence=1.0,
            ),
            {
                "rpm": 2300.0,
                "time_of_flight_sec": 0.575,
                "drive_angle": 1.5707963267948966,
                "drive_angular_velocity_rad_per_sec": 0.0,
                "is_valid": True,
                "confidence": 92.5285556898915,
                "solved_distance_m": 2.3,
                "iterations_used": 0,
                "warm_start_used": False,
            },
        ),
        (
            ShotInputs(
                robot_pose=Pose2d(0.0, 0.0, Rotation2d()),
                field_velocity=ChassisSpeeds(0.0, 1.0, 0.0),
                robot_velocity=ChassisSpeeds(0.0, 1.0, 0.0),
                hub_center=Translation2d(4.0, 0.0),
                hub_forward=Translation2d(1.0, 0.0),
                vision_confidence=1.0,
            ),
            {
                "rpm": 3924.622896162366,
                "time_of_flight_sec": 0.9811557810391857,
                "drive_angle": -0.240539838010358,
                "drive_angular_velocity_rad_per_sec": 0.2631578947368421,
                "is_valid": True,
                "confidence": 0.0,
                "solved_distance_m": 3.8,
                "iterations_used": 3,
                "warm_start_used": True,
            },
        ),
        (
            ShotInputs(
                robot_pose=Pose2d(0.5, -0.2, Rotation2d.fromDegrees(20.0)),
                field_velocity=ChassisSpeeds(0.8, -0.3, 0.4),
                robot_velocity=ChassisSpeeds(0.8, -0.3, 0.4),
                hub_center=Translation2d(4.5, 1.2),
                hub_forward=Translation2d(1.0, 0.0),
                vision_confidence=0.82,
                pitch_deg=1.5,
                roll_deg=-0.5,
            ),
            {
                "rpm": 3490.137612480127,
                "time_of_flight_sec": 0.8725177497156167,
                "drive_angle": 0.4474591214459063,
                "drive_angular_velocity_rad_per_sec": -0.11566340943015384,
                "is_valid": True,
                "confidence": 0.0,
                "solved_distance_m": 4.037940134088193,
                "iterations_used": 2,
                "warm_start_used": True,
            },
        ),
    )

    for inputs, expected in scenarios:
        result = calculator.calculate(inputs)
        assert math.isclose(result.rpm, expected["rpm"], rel_tol=1e-12, abs_tol=1e-12)
        assert math.isclose(
            result.time_of_flight_sec,
            expected["time_of_flight_sec"],
            rel_tol=1e-12,
            abs_tol=1e-12,
        )
        assert math.isclose(
            result.drive_angle.radians(),
            expected["drive_angle"],
            rel_tol=1e-12,
            abs_tol=1e-12,
        )
        assert math.isclose(
            result.drive_angular_velocity_rad_per_sec,
            expected["drive_angular_velocity_rad_per_sec"],
            rel_tol=1e-12,
            abs_tol=1e-12,
        )
        assert result.is_valid is expected["is_valid"]
        assert math.isclose(
            result.confidence,
            expected["confidence"],
            rel_tol=1e-12,
            abs_tol=1e-12,
        )
        assert math.isclose(
            result.solved_distance_m,
            expected["solved_distance_m"],
            rel_tol=1e-12,
            abs_tol=1e-12,
        )
        assert result.iterations_used == expected["iterations_used"]
        assert result.warm_start_used is expected["warm_start_used"]
