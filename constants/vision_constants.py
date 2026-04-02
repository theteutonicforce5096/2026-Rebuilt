from math import pi
from typing import Callable

from phoenix6 import swerve

from constants.swerve_drivetrain_constants import SwerveDrivetrainConstants
from subsystems.vision import Vision

class VisionConstants:
    """
    Constants for Vision Subsystem.
    """

    # Vision measurement baseline standard deviations
    _linear_std_dev_baseline = 1.0
    _angular_std_dev_baseline = 1000000.0 * pi

    # Per-camera multipliers applied to the vision standard deviations
    # (back_camera, front_left_camera, front_right_camera)
    _camera_std_dev_factors = (1.0, 0.25, 0.25)

    # Reference speeds used to scale vision standard deviations during fast motion
    _max_linear_speed = SwerveDrivetrainConstants._max_linear_speed * 0.25
    _max_angular_speed = SwerveDrivetrainConstants._max_angular_speed * 0.20

    # Reject vision measurements when robot tilt exceeds this threshold in degrees
    _max_tilt_deg = 5.0

    @classmethod
    def create_vision(
        cls,
        add_vision_measurement,
        get_current_swerve_state: Callable[[], swerve.SwerveDrivetrain.SwerveDriveState],
        get_robot_tilt: Callable[[], tuple[float, float]],
    ) -> Vision:
        """
        Creates a Vision subsystem instance using the configured constant values.

        :param cls: VisionConstants class used as the source of the subsystem constants.
        :type cls: type[VisionConstants]
        :param add_vision_measurement: Callback used to inject accepted vision measurements into drivetrain odometry.
        :type add_vision_measurement: Callable[[wpimath.geometry.Pose2d, float, tuple[float, float, float]], None]
        :param get_current_swerve_state: Function that returns the current drivetrain state.
        :type get_current_swerve_state: Callable[[], phoenix6.swerve.SwerveDrivetrain.SwerveDriveState]
        :param get_robot_tilt: Function that returns the current robot pitch and roll in degrees.
        :type get_robot_tilt: Callable[[], tuple[float, float]]
        :returns: Configured vision subsystem.
        :rtype: subsystems.vision.Vision
        """

        return Vision(
            add_vision_measurement,
            get_current_swerve_state,
            get_robot_tilt,
            SwerveDrivetrainConstants._field_type,
            cls._linear_std_dev_baseline,
            cls._angular_std_dev_baseline,
            cls._camera_std_dev_factors,
            cls._max_linear_speed,
            cls._max_angular_speed,
            cls._max_tilt_deg,
        )
