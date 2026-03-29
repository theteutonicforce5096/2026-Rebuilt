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
    _camera_std_dev_factors = (7.5, 5, 5)

    # Reference speeds used to scale vision standard deviations during fast motion
    _max_linear_speed = SwerveDrivetrainConstants._max_linear_speed * 0.25
    _max_angular_speed = SwerveDrivetrainConstants._max_angular_speed * 0.25

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
        Creates a Vision subsystem instance.
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
