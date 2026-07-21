from typing import Callable, Any
from math import copysign, hypot, pi

from commands2 import FunctionalCommand, Subsystem
from commands2.sysid import SysIdRoutine

from phoenix6 import swerve, utils, SignalLogger

from wpilib import DriverStation, Field2d, Notifier, RobotBase, RobotController, SendableChooser, SmartDashboard, Timer
from wpilib.shuffleboard import Shuffleboard
from wpilib.sysid import SysIdRoutineLog
from wpimath.geometry import Rotation2d, Translation2d, Pose2d
from wpimath.kinematics import ChassisSpeeds

from pathplannerlib.auto import AutoBuilder, RobotConfig
from pathplannerlib.controller import PIDConstants, PPHolonomicDriveController

from constants.shot_calculator_constants import (
    default_shooter_offset,
    get_hub_center,
    get_hub_reset_pose,
)

class SwerveDrivetrain(Subsystem, swerve.SwerveDrivetrain):
    """
    Class for controlling swerve drive.
    """

    def __init__(self, drive_motor_type: Any, steer_motor_type: Any, encoder_type: Any,
                 drivetrain_constants: swerve.SwerveDrivetrainConstants,
                 modules: list[swerve.SwerveModuleConstants],
                 odometry_update_frequency: float, max_linear_speed: float, max_angular_speed: float,
                 max_linear_rate_of_change: float, max_angular_rate_of_change: float,
                 teleop_precision_scale: float, teleop_default_scale: float, teleop_turbo_scale: float,
                 teleop_rotation_scale: float, teleop_translation_deadband: float,
                 teleop_rotation_deadband: float, field_type: str, wheel_radius: float):
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
        :param max_linear_rate_of_change: Max linear rate of change of drivetrain in meters per second.
        :type max_linear_rate_of_change: float
        :param max_angular_rate_of_change: Max angular rate of change of drivetrain in radians per second.
        :type max_angular_rate_of_change: float
        :param teleop_precision_scale: Fraction of max speed applied when the left trigger is held.
        :type teleop_precision_scale: float
        :param teleop_default_scale: Fraction of max speed applied when no trigger is held.
        :type teleop_default_scale: float
        :param teleop_turbo_scale: Fraction of max speed applied when both triggers are held.
        :type teleop_turbo_scale: float
        :param teleop_rotation_scale: Fraction of max angular speed applied to teleop rotation.
        :type teleop_rotation_scale: float
        :param teleop_translation_deadband: Translation stick deadband as a fraction of full travel.
        :type teleop_translation_deadband: float
        :param teleop_rotation_deadband: Rotation stick deadband as a fraction of full travel.
        :type teleop_rotation_deadband: float
        :param field_type: Current field variant used for field geometry.
        :type field_type: str
        :param wheel_radius: Effective drivetrain wheel radius in meters.
        :type wheel_radius: float
        """
        
        # Initialize parent classes
        Subsystem.__init__(self)
        swerve.SwerveDrivetrain.__init__(self, drive_motor_type, steer_motor_type, encoder_type, 
                                         drivetrain_constants, odometry_update_frequency, modules)
        
        if RobotBase.isSimulation():
            self._start_sim_thread()
        else:
            for num in range(4):
                module = self.get_module(num)
                module.drive_motor.optimize_bus_utilization()
                module.steer_motor.optimize_bus_utilization()
                module.encoder.optimize_bus_utilization()

            self.pigeon2.optimize_bus_utilization()
        
        # Create max speed and max acceleration variables
        self.max_linear_speed = max_linear_speed
        self.max_angular_speed = max_angular_speed
        self.max_linear_rate_of_change = max_linear_rate_of_change
        self.max_angular_rate_of_change = max_angular_rate_of_change

        # Create teleop input shaping variables
        self.teleop_precision_scale = teleop_precision_scale
        self.teleop_default_scale = teleop_default_scale
        self.teleop_turbo_scale = teleop_turbo_scale
        self.teleop_rotation_scale = teleop_rotation_scale
        self.teleop_translation_deadband = teleop_translation_deadband
        self.teleop_rotation_deadband = teleop_rotation_deadband

        # Last operator-perspective command, kept so teleop input can be slew-rate limited
        self._last_commanded_vx = 0.0
        self._last_commanded_vy = 0.0
        self._last_commanded_omega = 0.0

        # Create field type variable
        self.field_type = field_type

        # Create wheel and drive base radius variables for effective wheel radius characterization command
        self.wheel_radius = wheel_radius
        self._drive_base_radius = sum(
            module_location.norm() for module_location in self.module_locations
        ) / len(self.module_locations)

        # Create shooter position variable
        self.shooter_offset = default_shooter_offset

        # Create current alliance variable
        self.current_alliance = None

        # Create requests for controlling swerve drive
        # https://www.chiefdelphi.com/t/motion-magic-velocity-control-for-drive-motors-in-phoenix6-swerve-drive-api/483669/6
        self.field_centric_request = (
            swerve.requests.FieldCentric()
            .with_forward_perspective(swerve.requests.ForwardPerspectiveValue.OPERATOR_PERSPECTIVE)
            .with_drive_request_type(swerve.SwerveModule.DriveRequestType.VELOCITY)
            .with_steer_request_type(swerve.SwerveModule.SteerRequestType.MOTION_MAGIC_EXPO)
            .with_desaturate_wheel_speeds(True)
        )

        # Create Apply Robot Speeds Request for PathPlanner
        self._apply_robot_speeds = (
            swerve.requests.ApplyRobotSpeeds()
            .with_drive_request_type(swerve.SwerveModule.DriveRequestType.VELOCITY)
            .with_steer_request_type(swerve.SwerveModule.SteerRequestType.MOTION_MAGIC_EXPO)
            .with_desaturate_wheel_speeds(True)
        )

        # Request used by the hub and shot auto-align helpers. It faces a field-frame (blue-origin)
        # target angle, so it uses the blue-alliance perspective rather than the operator perspective.
        self.rotate_robot_request = (
            swerve.requests.FieldCentricFacingAngle()
            .with_forward_perspective(swerve.requests.ForwardPerspectiveValue.BLUE_ALLIANCE)
            .with_drive_request_type(swerve.SwerveModule.DriveRequestType.VELOCITY)
            .with_steer_request_type(swerve.SwerveModule.SteerRequestType.MOTION_MAGIC_EXPO)
            .with_desaturate_wheel_speeds(True)
            .with_heading_pid(7.5, 0, 0)
        )

        self.rotate_robot_pid_controller = self.rotate_robot_request.heading_controller
        self.rotate_robot_pid_controller.setTolerance(Rotation2d.fromDegrees(1.0).radians())

        self.set_forward_perspective()
        self.reset_teleop_drive_state()

        self._configure_auto_builder()

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
        Shuffleboard.getTab("SysId").add("Drivetrain Routines", self.sys_id_routines).withSize(2, 1)

        self.fused_robot_pose_field = Field2d()
        SmartDashboard.putData("Fused Robot Pose", self.fused_robot_pose_field)

    def _configure_auto_builder(self):
        """
        Configure PathPlanner to use this drivetrain for autonomous paths.
        """
        config = RobotConfig.fromGUISettings()
        AutoBuilder.configure(
            lambda: self.get_state().pose,   # Supplier of current robot pose
            self.reset_pose,                 # Consumer for seeding pose against auto
            lambda: self.get_state().speeds, # Supplier of current robot speeds
            # Consumer of ChassisSpeeds and feedforwards to drive the robot
            lambda speeds, feedforwards: self.set_control(
                self._apply_robot_speeds
                .with_speeds(ChassisSpeeds.discretize(speeds, 0.020))
                .with_wheel_force_feedforwards_x(feedforwards.robotRelativeForcesXNewtons)
                .with_wheel_force_feedforwards_y(feedforwards.robotRelativeForcesYNewtons)
            ),
            PPHolonomicDriveController(
                # PID constants for translation
                PIDConstants(10.0, 0.0, 0.0),
                # PID constants for rotation
                PIDConstants(7.0, 0.0, 0.0)
            ),
            config,
            # Assume the path needs to be flipped for Red vs Blue, this is normally the case
            lambda: (DriverStation.getAlliance() or DriverStation.Alliance.kBlue) == DriverStation.Alliance.kRed,
            self # Subsystem for requirements
        )

    def _start_sim_thread(self):
        """
        Start the faster simulation notifier used for Phoenix swerve sim.
        """
        def _sim_periodic():
            """
            Advance the Phoenix drivetrain simulator at the notifier rate.
            """
            current_time = utils.get_current_time_seconds()
            delta_time = current_time - self._last_sim_time
            self._last_sim_time = current_time

            self.update_sim_state(delta_time, RobotController.getBatteryVoltage())

        # Run simulation at a faster rate so PID gains behave more reasonably
        self._last_sim_time = utils.get_current_time_seconds()
        self._sim_notifier = Notifier(_sim_periodic)
        self._sim_notifier.startPeriodic(0.004) # 4ms

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

    def reset_teleop_drive_state(self):
        """
        Seed the teleop slew-rate state from the robot's current velocity.

        The default drive command is interrupted and rescheduled often, so the slew state must be
        reset to the robot's actual velocity (in the operator-perspective command frame) rather than
        to zero. Seeding to zero while the robot is moving would command a sudden decel then re-accel.
        """

        current_state = self.get_state()

        # Convert the measured robot-relative velocity into the field frame, then into the
        # operator-perspective command frame used by the field-centric request.
        field_relative_speeds = ChassisSpeeds.fromRobotRelativeSpeeds(
            current_state.speeds,
            current_state.pose.rotation()
        )
        command_velocity = Translation2d(
            field_relative_speeds.vx, field_relative_speeds.vy
        ).rotateBy(-self.get_operator_forward_direction())

        self._last_commanded_vx = command_velocity.x
        self._last_commanded_vy = command_velocity.y
        self._last_commanded_omega = current_state.speeds.omega

    def reset_pose_hub(self):
        """
        Reset the robot pose to a hub-facing reference pose for the current alliance.
        """
        self.reset_pose(get_hub_reset_pose(self.current_alliance))

    def get_robot_tilt(self) -> tuple[float, float]:
        """
        Get the current pitch and roll in degrees of the robot from the Pigeon 2.
        """

        return (self.pigeon2.get_pitch().value_as_double, self.pigeon2.get_roll().value_as_double)

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
        :param strafe_speed: Desired strafe speed of the operator in terms of percent of max linear speed where left is positive. 
        :type strafe_speed: float
        :param rotation_speed: Desired rotation speed of the operator in terms of percent of max angular speed where counterclockwise is positive. 
        :type rotation_speed: float
        """

        # Select the throttle scaling from the trigger state. Both triggers is turbo, left trigger
        # alone is precision, and no trigger (or right trigger alone) is the default scale.
        if left_trigger_pressed and right_trigger_pressed:
            throttle_scale = self.teleop_turbo_scale
        elif left_trigger_pressed:
            throttle_scale = self.teleop_precision_scale
        else:
            throttle_scale = self.teleop_default_scale

        max_linear_speed = self.max_linear_speed * throttle_scale
        max_angular_speed = self.max_angular_speed * throttle_scale * self.teleop_rotation_scale

        # Shape the translation inputs: deadband with rescaling, then square to soften the low end
        shaped_forward = self._apply_deadband(forward_speed, self.teleop_translation_deadband)
        shaped_strafe = self._apply_deadband(strafe_speed, self.teleop_translation_deadband)
        requested_vx = abs(shaped_forward) * shaped_forward * max_linear_speed
        requested_vy = abs(shaped_strafe) * shaped_strafe * max_linear_speed

        # Shape the rotation input: deadband with rescaling, then cube for finer control near center
        shaped_rotation = self._apply_deadband(rotation_speed, self.teleop_rotation_deadband)
        requested_omega = shaped_rotation * shaped_rotation * shaped_rotation * max_angular_speed

        # Slew-rate limit the translation vector (preserving direction) and the rotation separately
        limited_vx, limited_vy = self._slew_translation(requested_vx, requested_vy)
        limited_omega = self._slew_rotation(requested_omega)

        return (
            self.field_centric_request
            .with_velocity_x(limited_vx)
            .with_velocity_y(limited_vy)
            .with_rotational_rate(limited_omega)
        )

    @staticmethod
    def _apply_deadband(value: float, deadband: float) -> float:
        """
        Apply a deadband to a joystick value and rescale the remaining range to full travel.

        :param value: Raw joystick value in the range [-1, 1].
        :type value: float
        :param deadband: Deadband as a fraction of full travel.
        :type deadband: float
        :returns: Rescaled value that ramps from 0 at the deadband edge to 1 at full stick.
        :rtype: float
        """
        if abs(value) <= deadband:
            return 0.0

        return (value - copysign(deadband, value)) / (1.0 - deadband)

    def _slew_translation(self, requested_vx: float, requested_vy: float) -> tuple[float, float]:
        """
        Slew-rate limit the translation command as a vector so acceleration keeps its direction.

        :param requested_vx: Requested operator-perspective x velocity in meters per second.
        :type requested_vx: float
        :param requested_vy: Requested operator-perspective y velocity in meters per second.
        :type requested_vy: float
        :returns: The limited (vx, vy) command in meters per second.
        :rtype: tuple[float, float]
        """
        delta_vx = requested_vx - self._last_commanded_vx
        delta_vy = requested_vy - self._last_commanded_vy

        # Scale the whole delta vector down if its magnitude exceeds the per-loop limit
        delta_norm = hypot(delta_vx, delta_vy)
        if delta_norm > self.max_linear_rate_of_change:
            scale = self.max_linear_rate_of_change / delta_norm
            delta_vx *= scale
            delta_vy *= scale

        self._last_commanded_vx += delta_vx
        self._last_commanded_vy += delta_vy
        return self._last_commanded_vx, self._last_commanded_vy

    def _slew_rotation(self, requested_omega: float) -> float:
        """
        Slew-rate limit the rotation command toward the requested rate.

        :param requested_omega: Requested rotational rate in radians per second.
        :type requested_omega: float
        :returns: The limited rotational rate in radians per second.
        :rtype: float
        """
        delta_omega = requested_omega - self._last_commanded_omega
        if delta_omega > self.max_angular_rate_of_change:
            delta_omega = self.max_angular_rate_of_change
        elif delta_omega < -self.max_angular_rate_of_change:
            delta_omega = -self.max_angular_rate_of_change

        self._last_commanded_omega += delta_omega
        return self._last_commanded_omega
    
    def get_operator_drive_command(self, left_trigger: Callable[[], bool], right_trigger: Callable[[], bool],
                                   forward_speed: Callable[[], float], strafe_speed: Callable[[], float],
                                   rotation_speed: Callable[[], float]):
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
        in terms of percent of max linear speed where left is positive. 
        :type strafe_speed: Callable[[], float]
        :param rotation_speed: Function that returns the desired rotation speed of the operator 
        in terms of percent of max angular speed where counterclockwise is positive. 
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

    def _get_hub_center(self) -> Translation2d:
        """
        Return the hub center for the current alliance and field variant.
        """
        return get_hub_center(self.field_type, self.current_alliance)

    def _get_shooter_center_of_rotation(self) -> Translation2d:
        """
        Return the shooter offset used as the drivetrain center of rotation.
        """
        return self.shooter_offset.translation()

    def _get_hub_alignment_angle(self) -> Rotation2d:
        """
        Compute the hub-facing heading from the shooter reference point.
        """
        current_pose = self.get_state().pose
        shooter_pose = current_pose.transformBy(self.shooter_offset)
        hub_center = self._get_hub_center()

        aim_angle = Rotation2d(
            hub_center.x - shooter_pose.x,
            hub_center.y - shooter_pose.y,
        )

        return aim_angle - self.shooter_offset.rotation()

    def _apply_alignment_target(self, target_angle: Rotation2d):
        """
        Apply a rotate-in-place request about the shooter center of rotation.

        :param target_angle: Field-relative heading to rotate the robot toward.
        :type target_angle: wpimath.geometry.Rotation2d
        """
        request = (
            self.rotate_robot_request
            .with_target_direction(target_angle)
            .with_center_of_rotation(self._get_shooter_center_of_rotation())
        )

        self.set_control(request)

    def get_hub_alignment_error(self) -> Rotation2d:
        """
        Return the current signed heading error between robot and hub aim angle.
        """
        return self._get_hub_alignment_angle() - self.get_state().pose.rotation()

    def is_hub_alignment_within_tolerance(self, tolerance_deg: float = 2.5) -> bool:
        """
        Check whether hub alignment error is within the requested tolerance.

        :param tolerance_deg: Maximum allowed absolute hub alignment error in degrees.
        :type tolerance_deg: float
        :returns: True when the current hub alignment error is within tolerance.
        :rtype: bool
        """
        return abs(self.get_hub_alignment_error().degrees()) <= tolerance_deg

    def create_hold_hub_alignment_command(self):
        """
        Build a command that continuously rotates the robot toward the hub.
        """
        return self.runOnce(
            lambda: self.rotate_robot_pid_controller.reset()
        ).andThen(
            self.run(
                lambda: self._apply_alignment_target(self._get_hub_alignment_angle())
            )
        )

    def create_stop_command(self):
        """
        Build a one-shot command that zeros all chassis velocity requests.
        """
        return self.runOnce(
            lambda: self.set_control(
                self.field_centric_request
                .with_velocity_x(0.0)
                .with_velocity_y(0.0)
                .with_rotational_rate(0.0)
            )
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

    def create_module_alignment_diagnostic_command(self):
        """
        Point all modules forward and publish each module's azimuth error for calibration.

        Use this in test mode to check whether the modules truly point straight when commanded to
        0 degrees. A nonzero error that differs per module points to CANcoder offsets needing
        recalibration, which is the usual cause of the robot yawing while driving straight.
        """
        point_forward_request = swerve.requests.PointWheelsAt().with_module_direction(Rotation2d())

        def publish_module_errors():
            """
            Command the modules forward and report each module's azimuth error in degrees.
            """
            self.set_control(point_forward_request)
            for module_index in range(len(self.modules)):
                current_angle = self.get_module(module_index).get_current_state().angle
                SmartDashboard.putNumber(
                    f"Module {module_index} Azimuth Error (deg)",
                    current_angle.degrees(),
                )

        return self.run(publish_module_errors)

    def create_effective_wheel_radius_characterization_command(self):
        """
        Spin the robot through a timed profile and print the effective wheel radius.
        """
        ramp_duration_sec = 2
        hold_duration_sec = 1
        characterization_duration_sec = (2.0 * ramp_duration_sec) + hold_duration_sec
        initial_yaw_deg = 0.0
        initial_distances_m = [0.0] * len(self.modules)
        timer = Timer()

        def initialize():
            """
            Capture the starting yaw and module distances before the spin test.
            """
            nonlocal initial_yaw_deg, initial_distances_m
            initial_yaw_deg = self.pigeon2.get_yaw().value_as_double
            initial_distances_m = [
                self.get_module(module_index).get_position(True).distance
                for module_index in range(len(self.modules))
            ]
            timer.restart()

        def execute():
            """
            Ramp up, hold, and ramp down rotational speed for the characterization.
            """
            elapsed_sec = timer.get()
            if elapsed_sec < ramp_duration_sec:
                requested_omega_rad_per_sec = (self.max_angular_speed * 0.5) * (elapsed_sec / ramp_duration_sec)
            elif elapsed_sec < (ramp_duration_sec + hold_duration_sec):
                requested_omega_rad_per_sec = self.max_angular_speed * 0.5
            else:
                requested_omega_rad_per_sec = (self.max_angular_speed * 0.5) * max(
                    0.0,
                    (characterization_duration_sec - elapsed_sec) / ramp_duration_sec,
                )

            self.set_control(
                self.field_centric_request
                .with_velocity_x(0.0)
                .with_velocity_y(0.0)
                .with_rotational_rate(-requested_omega_rad_per_sec)
            )

        def end(interrupted: bool):
            """
            Stop the robot and print the computed effective wheel radius.

            :param interrupted: Whether the characterization command ended due to interruption.
            :type interrupted: bool
            """
            timer.stop()
            self.set_control(
                self.field_centric_request
                .with_velocity_x(0.0)
                .with_velocity_y(0.0)
                .with_rotational_rate(0.0)
            )

            final_yaw_deg = self.pigeon2.get_yaw().value_as_double
            yaw_delta_rad = abs(final_yaw_deg - initial_yaw_deg) * pi / 180.0
            wheel_distance_delta_m = [
                abs(self.get_module(module_index).get_position(True).distance - initial_distances_m[module_index])
                for module_index in range(len(self.modules))
            ]
            average_wheel_delta_m = sum(wheel_distance_delta_m) / len(wheel_distance_delta_m)

            if average_wheel_delta_m <= 1e-9 or yaw_delta_rad <= 1e-9:
                print("Effective wheel radius characterization did not collect enough movement data.")
                return

            effective_wheel_radius_m = (
                self.wheel_radius
                * self._drive_base_radius
                * yaw_delta_rad
                / average_wheel_delta_m
            )
            print(
                "Effective wheel radius:"
                f" {effective_wheel_radius_m:.6f} m"
                f" ({effective_wheel_radius_m / 0.0254:.3f} in)"
            )

        return FunctionalCommand(
            initialize,
            execute,
            end,
            lambda: timer.get() >= characterization_duration_sec,
            self,
        )
