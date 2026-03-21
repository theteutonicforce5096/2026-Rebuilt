from pathlib import Path
import sys

from wpimath.geometry import Rotation2d

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from constants.shot_calculator_support import LaunchParameters
from subsystems.swerve_drivetrain import SwerveDrivetrain


def test_resolve_shot_alignment_angle_uses_drive_angle_for_valid_shot() -> None:
    current_heading = Rotation2d.fromDegrees(15.0)
    target_angle = Rotation2d.fromDegrees(65.0)
    launch_parameters = LaunchParameters(
        flywheel_rps=50.0,
        time_of_flight_sec=0.25,
        drive_angle=target_angle,
        drive_angular_velocity_rad_per_sec=0.0,
        is_valid=True,
        confidence=90.0,
        solved_distance_m=2.0,
        iterations_used=1,
        warm_start_used=False,
    )

    resolved_angle = SwerveDrivetrain._resolve_shot_alignment_angle(
        launch_parameters,
        current_heading,
    )

    assert resolved_angle == target_angle


def test_resolve_shot_alignment_angle_holds_heading_for_invalid_shot() -> None:
    current_heading = Rotation2d.fromDegrees(30.0)
    launch_parameters = LaunchParameters(
        flywheel_rps=0.0,
        time_of_flight_sec=0.0,
        drive_angle=Rotation2d.fromDegrees(80.0),
        drive_angular_velocity_rad_per_sec=0.0,
        is_valid=False,
        confidence=0.0,
        solved_distance_m=0.0,
        iterations_used=0,
        warm_start_used=False,
    )

    resolved_angle = SwerveDrivetrain._resolve_shot_alignment_angle(
        launch_parameters,
        current_heading,
    )

    assert resolved_angle == current_heading
