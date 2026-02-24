from typing import Callable, Any
from math import copysign, pi, atan2

from commands2 import Subsystem

import ntcore

from phoenix6 import swerve, utils

from wpilib import DriverStation

from wpimath.filter import SlewRateLimiter
from wpimath.geometry import Rotation2d
from wpimath.units import inchesToMeters

from pathplannerlib.auto import AutoBuilder, RobotConfig
from pathplannerlib.controller import PIDConstants, PPHolonomicDriveController

from subsystems.camera import VisionCamera

class SwerveDrivetrain(Subsystem, swerve.SwerveDrivetrain):
    """
    Class for controlling swerve drive.
    """

    def __init__(self, drive_motor_type: Any, steer_motor_type: Any, encoder_type: Any, 
                 drivetrain_constants: swerve.SwerveDrivetrainConstants, 
                 modules: list[swerve.SwerveModuleConstants],
                 num_config_attempts: int, max_linear_speed: float, max_angular_rate: float):
        """
        Constructor for initializing swerve drivetrain using the specified constants.

        :param drive_motor_type: Type of the drive motor
        :type drive_motor_type: Any
        :param steer_motor_type: Type of the steer motor
        :type steer_motor_type: Any
        :param encoder_type: Type of the azimuth encoder
        :type encoder_type: Any
        :param drivetrain_constants: Drivetrain-wide constants for the swerve drive
        :type drivetrain_constants: phoenix6.swerve.SwerveDrivetrainConstants
        :param modules: Constants for each specific module
        :type modules: list[phoenix6.swerve.SwerveModuleConstants]
        :param num_config_attempts: Number of times to attempt to configure each device
        :type num_config_attempts: int
        :param max_linear_speed: Max linear speed of drivetrain in meters per second. 
        :type max_linear_speed: float
        :param max_angular_rate: Max angular velocity of drivetrain in radians per second. 
        :type max_angular_rate: float
        """

        # Redefine the number of config attempts to try in phoenix6.swerve.swerve_drivetrain API
        swerve.swerve_drivetrain._NUM_CONFIG_ATTEMPTS = num_config_attempts
        
        # Initialize parent classes
        Subsystem.__init__(self)
        swerve.SwerveDrivetrain.__init__(self, drive_motor_type, steer_motor_type, encoder_type, 
                                         drivetrain_constants, 500.0, modules)
        
        # Create Limelight instance and configure default values
        # self.camera = VisionCamera()
        
        # Create max speeds variables
        self.max_linear_speed = max_linear_speed
        self.max_angular_rate = max_angular_rate

        ### TODO: Move this to constants file, have swerve drive and vision camera class take this as input
        self.field_type = "AndyMark" # Welded for regionals

        # Create hub position variables
        self.hub_x_pos = None
        self.hub_y_pos = None

        # Create current alliance variable
        self.current_alliance = None 
        self._set_forward_perspective()

        # Create slew rate limiters for limiting robot acceleration
        self.straight_speed_limiter = SlewRateLimiter(self.max_linear_speed * 5)
        self.strafe_speed_limiter = SlewRateLimiter(self.max_linear_speed * 5)
        self.rotation_speed_limiter = SlewRateLimiter(self.max_angular_rate * 10)

        # Create requests for controlling swerve drive
        # https://www.chiefdelphi.com/t/motion-magic-velocity-control-for-drive-motors-in-phoenix6-swerve-drive-api/483669/6
        self.default_mode_field_centric_request = (
            swerve.requests.FieldCentric()
            .with_forward_perspective(swerve.requests.ForwardPerspectiveValue.OPERATOR_PERSPECTIVE)
            .with_drive_request_type(swerve.SwerveModule.DriveRequestType.VELOCITY)
            .with_steer_request_type(swerve.SwerveModule.SteerRequestType.POSITION)
            .with_deadband(self.max_linear_speed * 0.01) # Controller deadband of approximately 0.05
            .with_rotational_deadband(self.max_angular_rate * 0.03) # Controller deadband of approximately 0.05
            .with_desaturate_wheel_speeds(True)
        )

        self.max_speed_mode_field_centric_request = (
            swerve.requests.FieldCentric()
            .with_forward_perspective(swerve.requests.ForwardPerspectiveValue.OPERATOR_PERSPECTIVE)
            .with_drive_request_type(swerve.SwerveModule.DriveRequestType.VELOCITY)
            .with_steer_request_type(swerve.SwerveModule.SteerRequestType.POSITION)
            .with_deadband(self.max_linear_speed * 0.01) # Controller deadband of 0.05
            .with_rotational_deadband(self.max_angular_rate * 0.04) # Controller deadband of 0.05
            .with_desaturate_wheel_speeds(True)
        )

        self.rotate_robot_request = (
            swerve.requests.FieldCentricFacingAngle()
            .with_drive_request_type(swerve.SwerveModule.DriveRequestType.VELOCITY)
            .with_steer_request_type(swerve.SwerveModule.SteerRequestType.POSITION)
            .with_forward_perspective(swerve.requests.ForwardPerspectiveValue.OPERATOR_PERSPECTIVE)
            .with_heading_pid(1, 0, 0)
        )

        self.brake_mode_request = (
            swerve.requests.SwerveDriveBrake()
            .with_drive_request_type(swerve.SwerveModule.DriveRequestType.VELOCITY)
            .with_steer_request_type(swerve.SwerveModule.SteerRequestType.POSITION)
        )

        # # Create Apply Robot Speeds Request for PathPlanner
        # self.apply_robot_speeds_request = (
        #     swerve.requests.ApplyRobotSpeeds()
        #     .with_drive_request_type(swerve.SwerveModule.DriveRequestType.VELOCITY)
        #     .with_steer_request_type(swerve.SwerveModule.SteerRequestType.POSITION)
        #     .with_desaturate_wheel_speeds(True)
        # )

        # AutoBuilder.configure(
        #     lambda: self.get_state().pose,
        #     self.reset_pose,
        #     lambda: self.get_state().speeds,
        #     lambda speeds, feedforwards: self.set_control(
        #         self.apply_robot_speeds_request
        #         .with_speeds(speeds)
        #         .with_wheel_force_feedforwards_x(feedforwards.robotRelativeForcesXNewtons)
        #         .with_wheel_force_feedforwards_y(feedforwards.robotRelativeForcesYNewtons)
        #     ),
        #     PPHolonomicDriveController(
        #         PIDConstants(5.0, 0.0, 0.0),
        #         PIDConstants(5.0, 0.0, 0.0)
        #     ),
        #     RobotConfig.fromGUISettings(),
        #     lambda: (DriverStation.getAlliance() or DriverStation.Alliance.kBlue) == DriverStation.Alliance.kRed,
        #     self
        # )

        self.inst = ntcore.NetworkTableInstance.getDefault()
        self.pose_est_table = self.inst.getTable("Pose Esimation")
        
        self.fused_pose_est = self.pose_est_table.getDoubleArrayTopic("Fused Robot Pose").publish()
        self.fused_pose_est_pub = self.pose_est_table.getStringTopic("Field2d").publish()

        self.vision_pose_est = self.pose_est_table.getDoubleArrayTopic("Vision Robot Pose").publish()
        self.vision_pose_est_pub = self.pose_est_table.getStringTopic("Field2d").publish()

    def periodic(self):
        """
        Update pose of the robot in network tables periodically.
        """
        pass
        # robot_pose, timestamp = self.camera.get_vision_measurement()
        # if robot_pose and timestamp:
        #     self.add_vision_measurement(
        #         robot_pose,
        #         utils.fpga_to_current_time(timestamp),
        #         (1.0, 1.0, pi / 8)
        #     )

        #     vision_pose_array = [robot_pose.x, robot_pose.y, robot_pose.rotation().degrees()]
        #     self.vision_pose_est.set(vision_pose_array)

        # current_state = self.get_state()

        # # Telemeterize the poses to Field2d
        # fused_pose_array = [current_state.pose.x, current_state.pose.y, current_state.pose.rotation().degrees()]
        # self.fused_pose_est.set(fused_pose_array)

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

    def _set_forward_perspective(self):
        """
        Set forward perspective of the robot for field oriented drive.
        """

        alliance_color = DriverStation.getAlliance()
        if alliance_color == DriverStation.Alliance.kRed:
            # Red alliance sees forward as 180 degrees (toward blue alliance wall)
            self.set_operator_perspective_forward(Rotation2d.fromDegrees(180))  
            self.current_alliance = DriverStation.Alliance.kRed                
        else:
            # If alliance color is not detected or alliance is blue, default to/set blue alliance perspective
            # Blue alliance sees forward as 0 degrees (toward red alliance wall)
            self.set_operator_perspective_forward(Rotation2d.fromDegrees(0))
            self.current_alliance = DriverStation.Alliance.kBlue

            self._set_hub_position(self.field_type, self.current_alliance)
    
    def _set_hub_position(self, field_type: str, alliance_color: DriverStation.Alliance):
        """
        Set the hub position variables based on the field type and alliance color.

        :param field_type: The type of field being used; either "AndyMark" or "Welded"
        :type field_type: str
        :param alliance_color: The alliance color of the robot; either DriverStation.Alliance.kBlue or DriverStation.Alliance.kRed
        :type alliance_color: DriverStation.Alliance
        """

        if field_type == "AndyMark":
            if alliance_color == DriverStation.Alliance.kBlue:
                self.hub_x_pos = inchesToMeters(181.56)
                self.hub_y_pos = inchesToMeters(158.32)
            else:
                self.hub_x_pos = inchesToMeters(468.56)
                self.hub_y_pos = inchesToMeters(158.32)
        else:
            if alliance_color == DriverStation.Alliance.kBlue:
                self.hub_x_pos = inchesToMeters(182.11)
                self.hub_y_pos = inchesToMeters(158.84)
            else:
                self.hub_x_pos = inchesToMeters(469.11)
                self.hub_y_pos = inchesToMeters(158.84)

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
                self.max_speed_mode_field_centric_request.with_velocity_x(
                    self.straight_speed_limiter.calculate(
                        -copysign((forward_speed) ** 2, forward_speed) * self.max_linear_speed
                    )
                ).with_velocity_y(
                    self.strafe_speed_limiter.calculate(
                        -copysign((strafe_speed) ** 2, strafe_speed) * self.max_linear_speed
                    )
                ).with_rotational_rate(
                    self.rotation_speed_limiter.calculate(
                        -copysign((rotation_speed) ** 2, rotation_speed) * self.max_angular_rate
                    )
                )
            )
        else:
            operator_drive_request = (
                self.default_mode_field_centric_request.with_velocity_x(
                    self.straight_speed_limiter.calculate(
                        -copysign(forward_speed ** 2, forward_speed) * (self.max_linear_speed * 0.75)
                    )
                ).with_velocity_y(
                    self.strafe_speed_limiter.calculate(
                        -copysign(strafe_speed **2, strafe_speed) * (self.max_linear_speed * 0.75)
                    )
                ).with_rotational_rate(
                    self.rotation_speed_limiter.calculate(
                        -copysign(rotation_speed ** 2, rotation_speed) * (self.max_angular_rate * 0.75)
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

    def _auto_align_to_hub(self):
        """
        Rotate the robot to the optimal angle relative to a target.

        :param target_pose: Pose of the target to rotate towards.
        :type target_pose: utils.Pose2d
        :param rotation_threshold: Threshold in degrees for considering the robot to be facing the optimal angle.
        :type rotation_threshold: float
        """

        current_pose = self.get_state().pose

        # Get the optimal angle from the robot to the target
        optimal_angle = atan2(self.hub_y_pos - current_pose.y, self.hub_x_pos - current_pose.x)

        return self.run(
            lambda: self.set_control(
                self.rotate_robot_request.with_target_direction(
                    Rotation2d(optimal_angle)
                )
            )
        ).until(
            lambda: abs(self.get_state().pose.rotation().degrees() - optimal_angle) < 0.5
        )
    
    def set_brake_mode(self):
        """
        Set the drivetrain to brake mode.
        """
        return self.run(
            lambda: self.set_control(self.brake_mode_request)
        )
        