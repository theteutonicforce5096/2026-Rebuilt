from dataclasses import dataclass
import math
from math import cos, pi, radians, sqrt, tan

from wpilib import DriverStation
from wpimath.geometry import Pose2d, Rotation2d, Transform2d, Translation2d
from wpimath.kinematics import ChassisSpeeds
from wpimath.units import inchesToMeters


default_y_dis = inchesToMeters(72 - 21.25)  # 72 in. (hub height) - 21.25 in. (flywheel height)
default_r = inchesToMeters(2)  # Flywheel radius in meters
default_g = -9.8  # m/s^2
default_θ = 67.5  # Degrees

default_field_type = "AndyMark"
default_shooter_offset = Transform2d(
    Translation2d(inchesToMeters(-7.78), inchesToMeters(-7.95)),
    Rotation2d(),
)

_hub_positions = {
    ("AndyMark", DriverStation.Alliance.kBlue): Translation2d(
        inchesToMeters(181.56),
        inchesToMeters(158.32),
    ),
    ("AndyMark", DriverStation.Alliance.kRed): Translation2d(
        inchesToMeters(468.56),
        inchesToMeters(158.32),
    ),
    ("Welded", DriverStation.Alliance.kBlue): Translation2d(
        inchesToMeters(182.11),
        inchesToMeters(158.84),
    ),
    ("Welded", DriverStation.Alliance.kRed): Translation2d(
        inchesToMeters(469.11),
        inchesToMeters(158.84),
    ),
}


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
    flywheel_rps: float
    time_of_flight_sec: float
    drive_angle: Rotation2d
    drive_angular_velocity_rad_per_sec: float
    confidence: float
    solved_distance_m: float
    iterations_used: int
    warm_start_used: bool


def _calc_profile_velocity_mps(
    x_dis: float,
    y_dis: float = default_y_dis,
    θ: float = default_θ,
    g: float = default_g,
):
    return calc_velocity(x_dis, y_dis, θ, g)


def _get_default_alliance_color():
    alliance_color = DriverStation.getAlliance()
    if alliance_color == DriverStation.Alliance.kRed:
        return DriverStation.Alliance.kRed
    return DriverStation.Alliance.kBlue


def get_hub_center(
    field_type: str = default_field_type,
    alliance_color: DriverStation.Alliance | None = None,
):
    """
        Function to get the hub center translation for the current field setup.
    """

    resolved_alliance = alliance_color if alliance_color is not None else _get_default_alliance_color()
    hub_center = _hub_positions.get((field_type, resolved_alliance))
    if hub_center is None:
        raise ValueError(f"Unsupported field configuration: {field_type}, {resolved_alliance}")

    return hub_center


def calc_shooter_to_hub_distance(
    robot_pose: Pose2d,
    hub_center: Translation2d,
    shooter_offset: Transform2d = default_shooter_offset,
):
    """
        Function to get the shooter-to-hub distance in meters from pose geometry.
    """

    shooter_pose = robot_pose.transformBy(shooter_offset)
    shooter_to_hub = hub_center - shooter_pose.translation()
    return shooter_to_hub.norm()


def calc_distance_to_hub(
    robot_pose: Pose2d,
    field_type: str = default_field_type,
    alliance_color: DriverStation.Alliance | None = None,
    shooter_offset: Transform2d = default_shooter_offset,
):
    """
        Function to get the shooter-to-hub distance in meters using the
        drivetrain's field geometry defaults.
    """

    return calc_shooter_to_hub_distance(
        robot_pose,
        get_hub_center(field_type, alliance_color),
        shooter_offset,
    )


def calc_x_dis(
    robot_pose_or_x,
    y_pose: float | None = None,
    field_type: str = default_field_type,
    alliance_color: DriverStation.Alliance | None = None,
    shooter_offset: Transform2d = default_shooter_offset,
):
    """
        Compatibility wrapper for shooter-to-hub distance in meters.

        If given a Pose2d, heading is respected. If given raw x/y, a zero
        heading pose is assumed to preserve older call sites.
    """

    if isinstance(robot_pose_or_x, Pose2d):
        return calc_distance_to_hub(
            robot_pose_or_x,
            field_type,
            alliance_color,
            shooter_offset,
        )

    if y_pose is None:
        raise ValueError("y_pose must be provided when calc_x_dis is called with raw coordinates")

    return calc_distance_to_hub(
        Pose2d(robot_pose_or_x, y_pose, Rotation2d()),
        field_type,
        alliance_color,
        shooter_offset,
    )


def calc_velocity(
    x_dis: float,
    y_dis: float = default_y_dis,
    θ: float = default_θ,
    g: float = default_g,
):
    """
        Fuction for getting ideal velocity.

        :param x_dis: horizontal distance in meters from target to shooter
        :type x_dis: float
        :param y_dis: vertical distance in meters from target to shooter
        :type y_dis: float
        :param θ: angle in degrees from horizontal that ball shoots out at
        :type θ: float
        :param g: gravitational acceleration in meters per second squared
        :type g: float
    """

    gravity_magnitude = abs(g)
    theta_rad = radians(θ)
    denominator = 2 * (cos(theta_rad) ** 2) * (x_dis * tan(theta_rad) - y_dis)
    if denominator <= 0:
        raise ValueError("Shot is unreachable with the current launch angle and target height")

    ideal_velocity_mps = sqrt((gravity_magnitude * (x_dis ** 2)) / denominator)
    return ideal_velocity_mps


# Calibration Notes:
# these were kinda inconsistent
# TODO this info is useless cuz PID is cooked
# 50 rps for 58 inches (Hub front to robot front) 7.5ft tp back
# 60 rps for 11 ft (Hub front to robot back)
# robot with bumpers ~34 in. long

# target velocity puts ideal velocity in rotations per second
def shoot_speed(ideal_velocity_mps, r: float = default_r):
    """
        Function to get shoot velocity in rotations per second (rps)

        :param r: flywheel radius in meters
        :type r: float
    """

    target_velocity = ideal_velocity_mps / (2 * pi * r)
    return target_velocity


def calc_time_of_flight(
    x_dis: float,
    y_dis: float = default_y_dis,
    θ: float = default_θ,
    g: float = default_g,
):
    """
        Function to get shot time of flight in seconds from the same baseline
        model used for launch velocity.
    """

    ideal_velocity_mps = _calc_profile_velocity_mps(x_dis, y_dis, θ, g)
    horizontal_velocity_mps = ideal_velocity_mps * cos(radians(θ))
    if abs(horizontal_velocity_mps) < 1e-9:
        raise ValueError("Horizontal launch velocity must be non-zero")

    return x_dis / horizontal_velocity_mps


def calc_shot_profile(
    x_dis: float,
    y_dis: float = default_y_dis,
    θ: float = default_θ,
    g: float = default_g,
    r: float = default_r,
):
    """
        Function to get the baseline shot profile for a given distance.

        :returns: Tuple of target flywheel speed in rotations per second and
        time of flight in seconds.
    """

    ideal_velocity_mps = _calc_profile_velocity_mps(x_dis, y_dis, θ, g)
    return (
        shoot_speed(ideal_velocity_mps, r),
        calc_time_of_flight(x_dis, y_dis, θ, g),
    )

__all__ = [
    "Config",
    "LaunchParameters",
    "ShotInputs",
    "calc_distance_to_hub",
    "calc_shot_profile",
    "calc_shooter_to_hub_distance",
    "calc_time_of_flight",
    "calc_velocity",
    "calc_x_dis",
    "default_field_type",
    "default_g",
    "default_r",
    "default_shooter_offset",
    "default_y_dis",
    "default_θ",
    "get_hub_center",
    "shoot_speed",
]
