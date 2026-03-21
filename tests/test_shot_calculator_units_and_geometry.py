import math
from pathlib import Path
import sys

from wpilib import DriverStation
from wpimath.geometry import Pose2d, Rotation2d, Transform2d, Translation2d
from wpimath.kinematics import ChassisSpeeds
from wpimath.units import inchesToMeters

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from constants.ShotCalculator import ShotCalculator
from constants.physics import (
    calc_distance_to_hub,
    calc_shooter_to_hub_distance,
    calc_x_dis,
    get_hub_center,
)
from constants.shot_calculator_support import LaunchParameters, ShotInputs


def test_launch_parameters_use_flywheel_rps() -> None:
    calculator = ShotCalculator()
    result = calculator.calculate(
        ShotInputs(
            robot_pose=Pose2d(0.0, 0.0, Rotation2d()),
            field_velocity=ChassisSpeeds(),
            robot_velocity=ChassisSpeeds(),
            hub_center=Translation2d(2.0, 0.0),
            hub_forward=Translation2d(1.0, 0.0),
            vision_confidence=1.0,
        )
    )

    assert LaunchParameters.INVALID.flywheel_rps == 0.0
    assert result.is_valid
    assert result.flywheel_rps > 0.0
    assert not hasattr(result, "rpm")


def test_load_lut_entry_interpolates_rps_and_tof() -> None:
    calculator = ShotCalculator()
    calculator._rps_residual_map.clear()
    calculator._tof_residual_map.clear()

    low_distance = 1.0
    high_distance = 2.0
    midpoint_distance = 1.5

    calculator.load_lut_entry(
        low_distance,
        calculator._physics_rps(low_distance) + 2.0,
        calculator._physics_tof(low_distance) + 0.1,
    )
    calculator.load_lut_entry(
        high_distance,
        calculator._physics_rps(high_distance) + 4.0,
        calculator._physics_tof(high_distance) + 0.3,
    )

    assert math.isclose(
        calculator._effective_rps(midpoint_distance),
        calculator._physics_rps(midpoint_distance) + 3.0,
        rel_tol=1e-9,
        abs_tol=1e-9,
    )
    assert math.isclose(
        calculator._effective_tof(midpoint_distance),
        calculator._physics_tof(midpoint_distance) + 0.2,
        rel_tol=1e-9,
        abs_tol=1e-9,
    )


def test_calc_shooter_to_hub_distance_uses_pose_geometry() -> None:
    robot_pose = Pose2d(1.0, 2.0, Rotation2d())
    hub_center = Translation2d(4.0, 3.0)
    shooter_offset = Transform2d(Translation2d(0.5, -0.25), Rotation2d())

    expected_distance = math.hypot(4.0 - 1.5, 3.0 - 1.75)

    assert math.isclose(
        calc_shooter_to_hub_distance(robot_pose, hub_center, shooter_offset),
        expected_distance,
        rel_tol=1e-9,
        abs_tol=1e-9,
    )


def test_calc_shooter_to_hub_distance_respects_heading() -> None:
    hub_center = Translation2d(2.5, 1.0)
    shooter_offset = Transform2d(Translation2d(0.5, 0.0), Rotation2d())

    zero_heading_distance = calc_shooter_to_hub_distance(
        Pose2d(1.0, 1.0, Rotation2d()),
        hub_center,
        shooter_offset,
    )
    right_angle_distance = calc_shooter_to_hub_distance(
        Pose2d(1.0, 1.0, Rotation2d.fromDegrees(90.0)),
        hub_center,
        shooter_offset,
    )

    assert math.isclose(zero_heading_distance, 1.0, rel_tol=1e-9, abs_tol=1e-9)
    assert math.isclose(
        right_angle_distance,
        math.hypot(1.5, -0.5),
        rel_tol=1e-9,
        abs_tol=1e-9,
    )


def test_get_hub_center_matches_drivetrain_defaults() -> None:
    andymark_blue = get_hub_center("AndyMark", DriverStation.Alliance.kBlue)
    andymark_red = get_hub_center("AndyMark", DriverStation.Alliance.kRed)
    welded_blue = get_hub_center("Welded", DriverStation.Alliance.kBlue)
    welded_red = get_hub_center("Welded", DriverStation.Alliance.kRed)

    assert math.isclose(andymark_blue.x, inchesToMeters(181.56), abs_tol=1e-9)
    assert math.isclose(andymark_blue.y, inchesToMeters(158.32), abs_tol=1e-9)
    assert math.isclose(andymark_red.x, inchesToMeters(468.56), abs_tol=1e-9)
    assert math.isclose(andymark_red.y, inchesToMeters(158.32), abs_tol=1e-9)
    assert math.isclose(welded_blue.x, inchesToMeters(182.11), abs_tol=1e-9)
    assert math.isclose(welded_blue.y, inchesToMeters(158.84), abs_tol=1e-9)
    assert math.isclose(welded_red.x, inchesToMeters(469.11), abs_tol=1e-9)
    assert math.isclose(welded_red.y, inchesToMeters(158.84), abs_tol=1e-9)


def test_calc_x_dis_wraps_pose_and_xy_inputs() -> None:
    pose = Pose2d(1.5, 2.0, Rotation2d.fromDegrees(30.0))

    pose_distance = calc_x_dis(
        pose,
        field_type="AndyMark",
        alliance_color=DriverStation.Alliance.kBlue,
    )
    expected_pose_distance = calc_distance_to_hub(
        pose,
        field_type="AndyMark",
        alliance_color=DriverStation.Alliance.kBlue,
    )
    xy_distance = calc_x_dis(
        1.5,
        2.0,
        field_type="AndyMark",
        alliance_color=DriverStation.Alliance.kBlue,
    )
    expected_xy_distance = calc_distance_to_hub(
        Pose2d(1.5, 2.0, Rotation2d()),
        field_type="AndyMark",
        alliance_color=DriverStation.Alliance.kBlue,
    )

    assert math.isclose(pose_distance, expected_pose_distance, rel_tol=1e-9, abs_tol=1e-9)
    assert math.isclose(xy_distance, expected_xy_distance, rel_tol=1e-9, abs_tol=1e-9)
