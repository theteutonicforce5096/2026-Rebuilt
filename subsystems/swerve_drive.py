from typing import Callable
import math

from commands2 import Subsystem

from phoenix6 import swerve, utils

from wpilib import DriverStation, Field2d

from wpilib.shuffleboard import Shuffleboard

from wpimath.filter import SlewRateLimiter
from wpimath.geometry import Transform3d, Translation3d, Rotation3d, Rotation2d
from wpimath.controller import ProfiledPIDController
from wpimath.trajectory import TrapezoidProfile

from pathplannerlib.auto import AutoBuilder, RobotConfig
from pathplannerlib.controller import PIDConstants, PPHolonomicDriveController

from subsystems.camera import VisionCamera

class SwerveDrive(Subsystem, swerve.SwerveDrivetrain):
    """
    Class for controlling swerve drive.
    """

    def __init__(self, drive_motor_type, steer_motor_type, encoder_type, drivetrain_constants, modules,
                 max_linear_speed, max_angular_rate):
        """
        Constructor for initializing swerve drivetrain using the specified constants.

        :param drive_motor_type: Type of the drive motor
        :type drive_motor_type: type
        :param steer_motor_type: Type of the steer motor
        :type steer_motor_type: type
        :param encoder_type: Type of the azimuth encoder
        :type encoder_type: type
        :param drivetrain_constants: Drivetrain-wide constants for the swerve drive
        :type drivetrain_constants: swerve.SwerveDrivetrainConstants
        :param modules: Constants for each specific module
        :type modules: list[swerve.SwerveModuleConstants]
        :param max_linear_speed: Max linear speed of drivetrain in meters per second. 
        :type max_linear_speed: float
        :param max_angular_rate: Max angular velocity of drivetrain in radians per second. 
        :type max_angular_rate: float
        """

        # Initialize parent classes
        Subsystem.__init__(self)
        swerve.SwerveDrivetrain.__init__(self, drive_motor_type, steer_motor_type, encoder_type, 
                                         drivetrain_constants, modules)
        
        # Create Limelight instance and configure default values
        self.camera = VisionCamera()
        
        # Create max speeds variables
        self.max_linear_speed = max_linear_speed
        self.max_angular_rate = max_angular_rate

        # Create Apply Robot Speeds Request for PathPlanner
        self.apply_robot_speeds_request = (
            swerve.requests.ApplyRobotSpeeds()
            .with_drive_request_type(swerve.SwerveModule.DriveRequestType.VELOCITY)
            .with_steer_request_type(swerve.SwerveModule.SteerRequestType.POSITION)
            .with_desaturate_wheel_speeds(True)
        )

        AutoBuilder.configure(
            lambda: self.get_state().pose,
            self.reset_pose,
            lambda: self.get_state().speeds,
            lambda speeds, feedforwards: self.set_control(
                self.apply_robot_speeds_request
                .with_speeds(speeds)
                .with_wheel_force_feedforwards_x(feedforwards.robotRelativeForcesXNewtons)
                .with_wheel_force_feedforwards_y(feedforwards.robotRelativeForcesYNewtons)
            ),
            PPHolonomicDriveController(
                PIDConstants(5.0, 0.0, 0.0),
                PIDConstants(5.0, 0.0, 0.0)
            ),
            RobotConfig.fromGUISettings(),
            lambda: (DriverStation.getAlliance() or DriverStation.Alliance.kBlue) == DriverStation.Alliance.kRed,
            self
        )

        # Create request for controlling swerve drive
        # https://www.chiefdelphi.com/t/motion-magic-velocity-control-for-drive-motors-in-phoenix6-swerve-drive-api/483669/6
        self.default_mode_field_centric_request = (
            swerve.requests.FieldCentric()
            .with_forward_perspective(swerve.requests.ForwardPerspectiveValue.OPERATOR_PERSPECTIVE)
            .with_drive_request_type(swerve.SwerveModule.DriveRequestType.VELOCITY)
            .with_steer_request_type(swerve.SwerveModule.SteerRequestType.POSITION)
            .with_deadband(self.max_linear_speed * 0.05)
            .with_rotational_deadband(self.max_angular_rate * 0.05)
            .with_desaturate_wheel_speeds(True)
        )

        self.slow_mode_field_centric_request = (
            swerve.requests.FieldCentric()
            .with_forward_perspective(swerve.requests.ForwardPerspectiveValue.OPERATOR_PERSPECTIVE)
            .with_drive_request_type(swerve.SwerveModule.DriveRequestType.VELOCITY)
            .with_steer_request_type(swerve.SwerveModule.SteerRequestType.POSITION)
            .with_deadband(self.max_linear_speed * 0.025)
            .with_rotational_deadband(self.max_angular_rate * 0.025)
            .with_desaturate_wheel_speeds(True)
        )

        # Create slew rate limiters for limiting robot acceleration
        self.straight_speed_limiter = SlewRateLimiter(self.max_linear_speed * 4, -self.max_linear_speed * 4)
        self.strafe_speed_limiter = SlewRateLimiter(self.max_linear_speed * 4, -self.max_linear_speed * 4)
        self.rotation_speed_limiter = SlewRateLimiter(self.max_angular_rate * 4, -self.max_angular_rate * 4)

    def periodic(self):
        """
        Update pose of the robot in network tables periodically.
        """

        robot_pose, timestamp = self.camera.get_vision_measurement()
        if robot_pose and timestamp:
            self.add_vision_measurement(
                robot_pose.toPose2d(),
                utils.fpga_to_current_time(timestamp),
                (1.0, 1.0, math.pi / 4)
            )

    def reset_slew_rate_limiters(self):
        """
        Reset the slew rate limiters to the current speeds of the drivetrain.
        """

        # Get current state of the drivetrain
        current_state = self.get_state()

        # Reset slew rate limiters to current speed of the drivetrain
        self.straight_speed_limiter.reset(current_state.speeds.vx)
        self.strafe_speed_limiter.reset(current_state.speeds.vy)
        self.rotation_speed_limiter.reset(current_state.speeds.omega)

    def set_forward_perspective(self):
        """
        Set forward perspective of the robot for field oriented drive.
        """

        alliance_color = DriverStation.getAlliance()
        if alliance_color is not None:
            if alliance_color == DriverStation.Alliance.kBlue:
                # Blue alliance sees forward as 0 degrees (toward red alliance wall)
                self.set_operator_perspective_forward(Rotation2d.fromDegrees(0))
            else:
                # Red alliance sees forward as 180 degrees (toward blue alliance wall)
                self.set_operator_perspective_forward(Rotation2d.fromDegrees(180))  

    def _get_operator_drive_request(self, left_trigger_pressed: bool, right_trigger_pressed: bool,
                                   forward_speed: float, strafe_speed: float, rotation_speed: float):
        """
        Get the desired drive request of the operator.

        :param left_trigger_pressed: Whether the left trigger of the operator's controller is pressed or not.
        :type left_trigger_pressed: bool
        :param right_trigger_pressed: Whether the right trigger of the operator's controller is pressed or not.
        :type right_trigger_pressed: bool
        :param forward_speed: Desired forward speed of the operator in terms of percent of max linear speed where forward is positive. 
        :type forward_speed: float
        :param strafe_speed: Desired strafe speed of the operator in terms of percent of max linear speed where right is positive. 
        :type strafe_speed: float
        :param rotation_speed: Desired rotation speed of the operator in terms of percent of max angular speed where clockwise is positive. 
        :type rotation_speed: float
        """
        
        if left_trigger_pressed and right_trigger_pressed:
            operator_drive_request = (
                self.slow_mode_field_centric_request.with_velocity_x(
                    self.straight_speed_limiter.calculate(
                        -(forward_speed * abs(forward_speed * 1)) * self.max_linear_speed
                    )
                ).with_velocity_y(
                    self.strafe_speed_limiter.calculate(
                        -(strafe_speed * abs(strafe_speed * 1)) * self.max_linear_speed
                    )
                ).with_rotational_rate(
                    self.rotation_speed_limiter.calculate(
                        -(rotation_speed * abs(rotation_speed * 1)) * self.max_angular_rate
                    )
                )
            )
        else:
            operator_drive_request = (
                self.default_mode_field_centric_request.with_velocity_x(
                    self.straight_speed_limiter.calculate(
                        -(forward_speed * abs(forward_speed * 0.25)) * self.max_linear_speed
                    )
                ).with_velocity_y(
                    self.strafe_speed_limiter.calculate(
                        -(strafe_speed * abs(strafe_speed * 0.25)) * self.max_linear_speed
                    )
                ).with_rotational_rate(
                    self.rotation_speed_limiter.calculate(
                        -(rotation_speed * abs(rotation_speed * 0.25)) * self.max_angular_rate
                    )
                )
            )

        return operator_drive_request
    
    def get_operator_drive_command(self, left_trigger: Callable[[], bool], right_trigger: Callable[[], bool],
                                   forward_speed: Callable[[], bool], strafe_speed: Callable[[], bool], 
                                   rotation_speed: Callable[[], bool]):
        """
        Get the drive command for driving the robot.

        :param left_trigger: Function that returns whether the left trigger of the operator's controller 
        is pressed or not.
        :type left_trigger: Callable[[], bool]
        :param right_trigger: Function that returns whether the right trigger of the operator's controller 
        is pressed or not.
        :type right_trigger: Callable[[], bool]
        :param forward_speed: Function that returns the desired forward speed of the operator 
        in terms of percent of max linear speed where forward is positive. 
        :type forward_speed: Callable[[], float]
        :param strafe_speed: Function that returns the desired strafe speed of the operator 
        in terms of percent of max linear speed where right is positive. 
        :type strafe_speed: Callable[[], float]
        :param rotation_speed: Function that returns the desired rotation speed of the operator 
        in terms of percent of max angular speed where clockwise is positive. 
        :type rotation_speed: Callable[[], float]
        """

        return self.run(
            lambda: self.set_control(
                self._get_operator_drive_request(
                    left_trigger(),
                    right_trigger(),
                    forward_speed(),
                    strafe_speed(),
                    rotation_speed()
                )
            )
        )
