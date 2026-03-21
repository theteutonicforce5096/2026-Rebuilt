from typing import Callable, Any
from math import pi, atan2, cos, sin

from commands2 import Subsystem

from phoenix6 import swerve, utils, hardware, SignalLogger

from wpilib import DriverStation, Field2d, RobotBase, SmartDashboard, SendableChooser

from wpilib.sysid import SysIdRoutineLog
from wpilib.shuffleboard import Shuffleboard
from commands2.sysid import SysIdRoutine

from wpimath.filter import SlewRateLimiter
from wpimath.geometry import Rotation2d, Transform2d, Translation2d
from wpimath.units import inchesToMeters, radiansToDegrees

from pathplannerlib.auto import AutoBuilder, RobotConfig
from pathplannerlib.controller import PIDConstants, PPHolonomicDriveController

class SwerveDrivetrain(Subsystem, swerve.SwerveDrivetrain):
    """
    Class for controlling swerve drive.
    """

    def __init__(self, drive_motor_type: Any, steer_motor_type: Any, encoder_type: Any, 
                 drivetrain_constants: swerve.SwerveDrivetrainConstants, 
                 modules: list[swerve.SwerveModuleConstants],
                 odometry_update_frequency: float, max_linear_speed: float, max_angular_speed: float, 
                 max_linear_accel: float, max_angular_accel: float):
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
        :param odometry_update_frequency: The frequency to run the odometry loop at.
        :type odometry_update_frequency: hertz
        :param max_linear_speed: Max linear speed of drivetrain in meters per second. 
        :type max_linear_speed: float
        :param max_angular_speed: Max angular velocity of drivetrain in radians per second. 
        :type max_angular_speed: float
        :param max_linear_accel: Max linear acceleration of drivetrain in meters per second squared.
        :type max_linear_accel: float
        :param max_angular_accel: Max angular acceleration of drivetrain in radians per second squared.
        :type max_angular_accel: float
        """
        
        # Initialize parent classes
        Subsystem.__init__(self)
        swerve.SwerveDrivetrain.__init__(self, drive_motor_type, steer_motor_type, encoder_type, 
                                         drivetrain_constants, odometry_update_frequency, modules)
        
        if RobotBase.isSimulation() == False:
            for num in range(4):
                module = self.get_module(num)
                module.drive_motor.optimize_bus_utilization()
                module.steer_motor.optimize_bus_utilization()
                module.encoder.optimize_bus_utilization()

            self.pigeon2.optimize_bus_utilization()
        
        # Create max speed and max accel variables
        self.max_linear_speed = max_linear_speed
        self.max_angular_speed = max_angular_speed
        self.max_linear_accel = max_linear_accel
        self.max_angular_accel = max_angular_accel

        ### TODO: Move this to constants file, have swerve drive and vision camera class take this as input
        self.field_type = "AndyMark" # Welded for regionals

        # Create hub position variables
        self.hub_x_pos = None
        self.hub_y_pos = None

        # Create shooter position variable
        # X: -7.78 inches (behind center)
        # Y: -7.95 inches (right of center)
        self.shooter_offset = Transform2d(
            Translation2d(inchesToMeters(-7.78), inchesToMeters(-7.95)),
            Rotation2d(0) # 0 means it always faces the same way as the drivebase
        )

        # Create current alliance variable
        self.current_alliance = None 
        self.set_forward_perspective()

        # Create request for controlling swerve drive
        # https://www.chiefdelphi.com/t/motion-magic-velocity-control-for-drive-motors-in-phoenix6-swerve-drive-api/483669/6
        self.field_centric_request = (
            swerve.requests.FieldCentric()
            .with_forward_perspective(swerve.requests.ForwardPerspectiveValue.OPERATOR_PERSPECTIVE)
            .with_drive_request_type(swerve.SwerveModule.DriveRequestType.VELOCITY)
            .with_steer_request_type(swerve.SwerveModule.SteerRequestType.POSITION)
            .with_desaturate_wheel_speeds(True)
        )

        self.rotate_robot_request = (
            swerve.requests.FieldCentricFacingAngle()
            .with_drive_request_type(swerve.SwerveModule.DriveRequestType.VELOCITY)
            .with_steer_request_type(swerve.SwerveModule.SteerRequestType.POSITION)
            .with_forward_perspective(swerve.requests.ForwardPerspectiveValue.BLUE_ALLIANCE)
            .with_heading_pid(10, 0, 0)
        )

        self.rotate_robot_pid_controller = self.rotate_robot_request.heading_controller
        self.rotate_robot_pid_controller.setTolerance(1.0)

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

         # Swerve requests for SysId characterization
        self.translation_characterization = swerve.requests.SysIdSwerveTranslation()
        self.steer_characterization = swerve.requests.SysIdSwerveSteerGains()
        self.rotation_characterization = swerve.requests.SysIdSwerveRotation()

        # Create SysId routine for characterizing drive.
        self.sys_id_routine_translation = SysIdRoutine(
            SysIdRoutine.Config(
                rampRate = 1.0,
                stepVoltage = 5.0,
                timeout = 5.0,
                recordState = lambda state: SignalLogger.write_string(
                    "SysId_Translation_State", SysIdRoutineLog.stateEnumToString(state)
                )
            ),
            SysIdRoutine.Mechanism(
                lambda output: self.set_control(self.translation_characterization.with_volts(output)),
                lambda log: None,
                self,
            ),
        )

        # Create SysId routine for characterizing steer.
        self.sys_id_routine_steer = SysIdRoutine(
            SysIdRoutine.Config(
                rampRate = 1.0,
                stepVoltage = 4.0,
                timeout = 5.0,
                recordState = lambda state: SignalLogger.write_string(
                    "SysId_Steer_State", SysIdRoutineLog.stateEnumToString(state)
                )
            ),
            SysIdRoutine.Mechanism(
                lambda output: self.set_control(self.steer_characterization.with_volts(output)),
                lambda log: None,
                self,
            ),
        )

        self.sys_id_routine_rotation = SysIdRoutine(
            SysIdRoutine.Config(
                rampRate = pi / 6,
                stepVoltage = 4.0,
                timeout = 5.0,
                recordState = lambda state: SignalLogger.write_string(
                    "SysId_Rotation_State", SysIdRoutineLog.stateEnumToString(state)
                ),
            ),
            SysIdRoutine.Mechanism(
                lambda output: self.set_control(self.rotation_characterization.with_rotational_rate(output)),
                lambda log: None,
                self,
            ),
        )

        # Create widget for selecting SysId routine and set default value
        self.sys_id_routine_to_apply = self.sys_id_routine_translation
        self.sys_id_routines = SendableChooser()
        self.sys_id_routines.setDefaultOption("Translation Routine", self.sys_id_routine_translation)
        self.sys_id_routines.addOption("Steer Routine", self.sys_id_routine_steer)
        self.sys_id_routines.addOption("Rotation Routine", self.sys_id_routine_rotation)

        # Send widget to Shuffleboard 
        Shuffleboard.getTab("SysId").add(f"Drivetrain Routines", self.sys_id_routines).withSize(2, 1)

        self.fused_robot_pose_field = Field2d()
        SmartDashboard.putData("Fused Robot Pose", self.fused_robot_pose_field)

    def periodic(self):
        """
        Update pose of the robot in network tables periodically.
        """

        # Get current state of the robot
        current_state = self.get_state()

        # Telemeterize the pose to Field2d
        self.fused_robot_pose_field.setRobotPose(current_state.pose)

    def set_forward_perspective(self):
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

    def get_robot_tilt(self) -> tuple[float, float]:
        """
        Get the current pitch and roll in degrees of the robot from the Pigeon 2.
        """

        return (self.pigeon2.get_pitch()._value, self.pigeon2.get_roll()._value)


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

        # Get current speeds of the drivetrain
        current_state = self.get_state()
        current_vx = current_state.speeds.vx
        current_vy = current_state.speeds.vy
        current_omega = current_state.speeds.omega
        
        # Square input (Preserve sign)
        requested_vx = abs(forward_speed) * forward_speed * self.max_linear_speed
        requested_vy = abs(strafe_speed) * strafe_speed * self.max_linear_speed
        requested_omega = rotation_speed * rotation_speed * rotation_speed * self.max_angular_speed
        
        # Clamp requested velocity to a window around ACTUAL velocity
        limited_vx = max(current_vx - self.max_linear_accel, min(requested_vx, current_vx + self.max_linear_accel))
        limited_vy = max(current_vy - self.max_linear_accel, min(requested_vy, current_vy + self.max_linear_accel))
        limited_omega = max(current_omega - self.max_angular_accel, min(requested_omega, current_omega + self.max_angular_accel))

        if left_trigger_pressed and right_trigger_pressed:
            scale_factor = 1.0
        else:
            scale_factor = 0.75
        
        operator_drive_request = (
            self.field_centric_request
            .with_velocity_x(requested_vx * scale_factor)
            .with_velocity_y(requested_vy * scale_factor)
            .with_rotational_rate(requested_omega * scale_factor)
            .with_deadband((self.max_linear_speed * scale_factor) * 0.1)
            .with_rotational_deadband((self.max_angular_speed * scale_factor) * 0.05)
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
                    -forward_speed(),
                    -strafe_speed(),
                    -rotation_speed()
                )
            )
        )

    def auto_align_to_hub(self):
        """
        Rotate the robot to the optimal angle relative to a target.

        :param target_pose: Pose of the target to rotate towards.
        :type target_pose: utils.Pose2d
        :param rotation_threshold: Threshold in degrees for considering the robot to be facing the optimal angle.
        :type rotation_threshold: float
        """

        # Get current pose of the robot
        current_pose = self.get_state().pose

        # Get pose of shooter
        shooter_pose = current_pose.transformBy(self.shooter_offset)

        # Calculate optimal angle to hub from shooter
        optimal_angle = Rotation2d(
            self.hub_x_pos - shooter_pose.x, 
            self.hub_y_pos - shooter_pose.y,
        )

        center_of_rotation = shooter_pose.translation() - current_pose.translation()

        return self.run(
            lambda: self.test_pid(optimal_angle, center_of_rotation)
            # lambda: self.set_control(
            #     self.rotate_robot_request.with_target_direction(
            #         Rotation2d(optimal_angle)
            #     )
            # )
        ).until(self.rotate_robot_pid_controller.atSetpoint)
    
        # .until(
        #     lambda: (self.get_state().pose.rotation().degrees() - optimal_angle.degrees()) < 1
        # )
        
    def test_pid(self, optimal_angle: Rotation2d, center_of_rotation: Translation2d):
        print(f"Target: {optimal_angle.degrees()}. Current: {self.get_state().pose.rotation().degrees()}")
        request = (
            self.rotate_robot_request
            .with_target_direction(optimal_angle)
            .with_center_of_rotation(center_of_rotation)

        )

        self.set_control(request)
    
    def set_brake_mode(self):
        """
        Set the drivetrain to brake mode.
        """
        return self.run(
            lambda: self.set_control(self.brake_mode_request)
        )
    
    def set_sys_id_routine(self):
        """
        Set the SysId Routine to run based off of the routine chosen in Shuffleboard.
        """
        self.sys_id_routine_to_apply = self.sys_id_routines.getSelected()

    def sys_id_quasistatic(self, direction: SysIdRoutine.Direction):
        """
        Runs the SysId Quasistatic test in the given direction for the routine specified by self.sys_id_routine_to_apply.

        :param direction: Direction of the SysId Quasistatic test
        :type direction: SysIdRoutine.Direction
        """
        return self.sys_id_routine_to_apply.quasistatic(direction)

    def sys_id_dynamic(self, direction: SysIdRoutine.Direction):
        """
        Runs the SysId Dynamic test in the given direction for the routine specified by self.sys_id_routine_to_apply.

        :param direction: Direction of the SysId Dynamic test
        :type direction: SysIdRoutine.Direction
        """
        return self.sys_id_routine_to_apply.dynamic(direction)
