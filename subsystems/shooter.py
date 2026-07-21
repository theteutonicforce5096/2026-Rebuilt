from typing import Any, Callable, Final

from commands2 import Subsystem
from commands2 import (
    DeferredCommand,
    ParallelCommandGroup,
    ParallelDeadlineGroup,
    RepeatCommand,
    SequentialCommandGroup,
    WaitUntilCommand,
)

from phoenix6 import CANBus, SignalLogger
from phoenix6.controls import VelocityVoltage, VoltageOut
from phoenix6.configs import TalonFXConfiguration, TalonFXSConfiguration
from phoenix6.hardware import TalonFX, TalonFXS

from commands2.sysid import SysIdRoutine
from ntcore import NetworkTableInstance
from wpilib import RobotBase, SendableChooser, Timer
from wpilib.shuffleboard import Shuffleboard
from wpilib.sysid import SysIdRoutineLog
from wpimath.geometry import Rotation2d, Transform2d, Translation2d

from constants.shot_calculator import ShotCalculator
from constants.shot_calculator_constants import (
    Config as ShotCalculatorConfig,
    calc_shooter_to_hub_distance,
)
from subsystems.device_config import configure_device

class Shooter(Subsystem):
    """
    Class for controlling shooter.
    """

    DEFAULT_FLYWHEEL_INTAKE_TARGET_RPS: Final[float] = 40.0

    # Seconds without an intake-motor surge before the hopper is considered empty
    EMPTY_TIMEOUT_SEC: Final[float] = 10.0

    def __init__(self, canbus: CANBus, flywheel_motor_id: int, flywheel_intake_motor_id: int,
                 flywheel_motor_configs: TalonFXSConfiguration,
                 flywheel_intake_motor_configs: TalonFXConfiguration,
                 num_config_attempts: int,
                 get_current_swerve_state: Callable[[], Any],
                 get_robot_tilt: Callable[[], tuple[float, float]],
                 get_hub_center: Callable[[], Translation2d],
                 launcher_offset_x: float, launcher_offset_y: float,
                 flywheel_target_offset_rps: float):
        """
        Constructor for initializing shooter using the specified constants.

        :param canbus: CANBus instance that electronics are on
        :type canbus: phoenix6.CANBus
        :param flywheel_motor_id: CAN ID of the flywheel motor
        :type flywheel_motor_id: int
        :param flywheel_intake_motor_id: CAN ID of the flywheel intake motor
        :type flywheel_intake_motor_id: int
        :param flywheel_motor_configs: Configs for the flywheel motor
        :type flywheel_motor_configs: phoenix6.configs.TalonFXSConfiguration
        :param flywheel_intake_motor_configs: Configs for the flywheel intake motor
        :type flywheel_intake_motor_configs: phoenix6.configs.TalonFXConfiguration
        :param num_config_attempts: Number of times to attempt to configure each device
        :type num_config_attempts: int
        :param get_current_swerve_state: Function that returns the drivetrain state used by the shot solver.
        :type get_current_swerve_state: Callable[[], Any]
        :param get_robot_tilt: Function that returns the current robot pitch and roll in degrees.
        :type get_robot_tilt: Callable[[], tuple[float, float]]
        :param get_hub_center: Function that returns the current hub center translation for the active alliance.
        :type get_hub_center: Callable[[], wpimath.geometry.Translation2d]
        :param launcher_offset_x: Forward launcher offset from the robot reference point in meters.
        :type launcher_offset_x: float
        :param launcher_offset_y: Lateral launcher offset from the robot reference point in meters.
        :type launcher_offset_y: float
        :param flywheel_target_offset_rps: Constant flywheel speed offset added to every shot target
            to trim consistent undershooting or overshooting.
        :type flywheel_target_offset_rps: float
        """

        # Initialize parent classes
        Subsystem.__init__(self)

        # Create motors
        self.flywheel_motor = TalonFXS(flywheel_motor_id, canbus)
        self.flywheel_intake_motor = TalonFX(flywheel_intake_motor_id, canbus)

        # Apply motor configs
        configure_device(self.flywheel_motor, flywheel_motor_configs, num_config_attempts)
        configure_device(self.flywheel_intake_motor, flywheel_intake_motor_configs, num_config_attempts)

        if not RobotBase.isSimulation():
            self.flywheel_motor.optimize_bus_utilization()
            self.flywheel_intake_motor.optimize_bus_utilization()

        # Create VelocityVoltage request
        self.velocity_pid_request = VelocityVoltage(velocity = 0)
        self.intake_velocity_pid_request = VelocityVoltage(velocity = 0) 
        
        self.voltage_request = VoltageOut(output = 0)

        # Empty detection state
        self.empty_time = Timer.getFPGATimestamp()
        self.surging = False

        # What to publish over networktables for shooter
        self._network_table_instance = NetworkTableInstance.getDefault()

        #Create SysId routine for characterizing flywheel motor
        self.flywheel_motor_sys_id_routine = SysIdRoutine(
            SysIdRoutine.Config(
                rampRate = 0.5,
                stepVoltage = 9.0,
                timeout = 15.0,
                recordState = lambda state: SignalLogger.write_string(
                    "SysId_Flywheel_Motor_State", SysIdRoutineLog.stateEnumToString(state)
                )
            ),
            SysIdRoutine.Mechanism(
                lambda output: self.flywheel_motor.set_control(self.voltage_request.with_output(output)),
                lambda log: None,
                self,
            ),
        )

        #Create SysId routine for characterizing flywheel intake motor
        self.flywheel_intake_motor_sys_id_routine = SysIdRoutine(
            SysIdRoutine.Config(
                rampRate = 0.5,
                stepVoltage = 9.0,
                timeout = 15.0,
                recordState = lambda state: SignalLogger.write_string(
                    "SysId_Flywheel_Intake_Motor_State", SysIdRoutineLog.stateEnumToString(state)
                )
            ),
            SysIdRoutine.Mechanism(
                lambda output: self.flywheel_intake_motor.set_control(self.voltage_request.with_output(output)),
                lambda log: None,
                self,
            ),
        )

        # Create widget for selecting SysId routine and set default value
        self.sys_id_routine_to_apply = self.flywheel_motor_sys_id_routine
        self.sys_id_routines = SendableChooser()
        self.sys_id_routines.setDefaultOption("Flywheel Motor Routine", self.flywheel_motor_sys_id_routine)
        self.sys_id_routines.addOption("Flywheel Intake Motor Routine", self.flywheel_intake_motor_sys_id_routine)

        # Send widget to Shuffleboard 
        Shuffleboard.getTab("SysId").add("Shooter Routines", self.sys_id_routines).withSize(2, 1)

        # Shooter state
        self._shooter_table = self._network_table_instance.getTable("Shooter State")
        self.desired_ball_speed = self._shooter_table.getFloatTopic("Desired Ball Speed in Rotations per Second").publish()
        self.desired_ball_speed_sub = self._shooter_table.getFloatTopic("Desired Ball Speed in Rotations per Second").subscribe(60)
        self.desired_flywheel_intake_speed = self._shooter_table.getFloatTopic("Desired Flywheel Intake Speed in Rotations per Second").publish()
        self.desired_flywheel_intake_speed_sub = self._shooter_table.getFloatTopic("Desired Flywheel Intake Speed in Rotations per Second").subscribe(40)

        self._get_current_swerve_state = get_current_swerve_state
        self._get_robot_tilt = get_robot_tilt
        self._get_hub_center = get_hub_center

        self.shot_calculator = ShotCalculator(
            ShotCalculatorConfig(
                launcher_offset_x = launcher_offset_x,
                launcher_offset_y = launcher_offset_y,
            )
        )
        self._last_flywheel_target_rps = 0.0

        # Constant flywheel speed offset added to every shot target for undershoot/overshoot trim
        self.flywheel_target_offset_rps = flywheel_target_offset_rps

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

    def _apply_distance_based_shot(self, feed: bool):
        """
        Spin the flywheel to the current distance-based target, optionally feeding.

        The distance-to-hub lookup is recomputed on every call so the target tracks the robot as it
        moves. When ``feed`` is set, the flywheel intake runs only once the flywheel has reached its
        setpoint; otherwise the intake is held stopped so the flywheel can spin up first.

        :param feed: Whether to run the flywheel intake once the flywheel is at speed.
        :type feed: bool
        """
        _, flywheel_target_velocity, intake_target_velocity = self.get_current_auto_shot_targets()
        self.set_flywheel_velocities(
            flywheel_target_velocity + self.flywheel_target_offset_rps,
            intake_target_velocity if (feed and self.is_flywheel_at_setpoint()) else 0.0,
        )

    def is_flywheel_at_setpoint(self, tolerance_rps: float = 1.0) -> bool:
        """
        Check whether the main flywheel is within the requested closed-loop tolerance.

        :param tolerance_rps: Closed-loop error tolerance in rotations per second.
        :type tolerance_rps: float
        :returns: True when the commanded flywheel target has been reached within tolerance.
        :rtype: bool
        """
        return (
            self._last_flywheel_target_rps > 0.0
            and self.flywheel_motor.get_closed_loop_error().is_near(0, tolerance_rps)
        )

    def create_manual_shoot_command(self):
        """
        Build the manual shoot command from the NetworkTables-set targets.
        """
        return DeferredCommand(
            lambda: self.shoot(
                self.desired_ball_speed_sub.get(),
                (
                    self.desired_flywheel_intake_speed_sub.get()
                    if self.is_flywheel_at_setpoint()
                    else 0.0
                )
            ),
            self
        )

    def create_manual_feed_command(self, hopper):
        """
        Feed manually only once the main flywheel has reached speed.

        :param hopper: Hopper subsystem used to feed balls toward the shooter.
        :type hopper: subsystems.hopper.Hopper
        :returns: Deferred command that either feeds or holds the hopper stopped.
        :rtype: commands2.Command
        """
        return DeferredCommand(
            lambda: (
                hopper.create_feed_cycle_command()
                if self.is_flywheel_at_setpoint()
                else hopper.create_stop_command()
            ),
            hopper
        )

    def get_current_auto_shot_targets(self) -> tuple[float, float, float]:
        """
        Measure the current shooter-to-hub distance and return the distance-based shot targets.
        """
        current_state = self._get_current_swerve_state()
        shooter_offset = Transform2d(
            Translation2d(
                self.shot_calculator.config.launcher_offset_x,
                self.shot_calculator.config.launcher_offset_y,
            ),
            Rotation2d(),
        )
        distance_m = calc_shooter_to_hub_distance(
            current_state.pose,
            self._get_hub_center(),
            shooter_offset,
        )
        flywheel_target_velocity = self.shot_calculator.get_profile_for_distance(distance_m)
        return distance_m, flywheel_target_velocity, self.DEFAULT_FLYWHEEL_INTAKE_TARGET_RPS

    def create_auto_run_shooter_command(self, hopper, drivetrain):
        """
        Defer auto-shot target capture until the command is actually scheduled.

        :param hopper: Hopper subsystem used to feed balls.
        :type hopper: subsystems.hopper.Hopper
        :param drivetrain: Drivetrain subsystem used for hub alignment during the shot.
        :type drivetrain: subsystems.swerve_drivetrain.SwerveDrivetrain
        :returns: Deferred autonomous shooter command.
        :rtype: commands2.Command
        """
        return DeferredCommand(
            lambda: self._build_auto_run_shooter_command(hopper, drivetrain),
            self, 
            hopper,
            drivetrain,
        )

    def _build_auto_run_shooter_command(self, hopper, drivetrain):
        """
        Build the auto-shot sequence with alignment and empty detection.

        The flywheel target is recomputed every loop from the current distance to the hub, so the
        shot stays correct as the robot moves during the sequence.

        :param hopper: Hopper subsystem used to feed balls.
        :type hopper: subsystems.hopper.Hopper
        :param drivetrain: Drivetrain subsystem used for hub alignment during the shot.
        :type drivetrain: subsystems.swerve_drivetrain.SwerveDrivetrain
        :returns: Fully constructed autonomous shooter command sequence.
        :rtype: commands2.Command
        """
        alignment_timeout = Timer()

        def ready_to_feed():
            """
            Gate feeding on flywheel readiness plus alignment or timeout.

            :returns: True when the shooter is ready to begin feeding.
            :rtype: bool
            """
            return self.is_flywheel_at_setpoint() and (
                drivetrain.is_hub_alignment_within_tolerance(2.5)
                or alignment_timeout.hasElapsed(1.0)
            )

        return SequentialCommandGroup(
            self.runOnce(lambda: self.reset_empty_time()),
            self.runOnce(lambda: alignment_timeout.restart()),
            ParallelDeadlineGroup(
                WaitUntilCommand(ready_to_feed),
                self.run(lambda: self._apply_distance_based_shot(feed=False)),
                drivetrain.create_hold_hub_alignment_command(),
            ),
            ParallelDeadlineGroup(
                WaitUntilCommand(lambda: self.detect_empty()),
                self.run(lambda: self._apply_distance_based_shot(feed=True)),
                RepeatCommand(self.create_manual_feed_command(hopper)),
                drivetrain.create_hold_hub_alignment_command(),
            ),
            ParallelCommandGroup(
                hopper.create_stop_command(),
                self.create_stop_command(),
                drivetrain.create_stop_command(),
            ),
        )

    def create_auto_shoot_command(self):
        """
        Spin the flywheel to the current distance-based target and feed the flywheel intake at speed.
        """
        return self.runOnce(
            lambda: self._apply_distance_based_shot(feed=True)
        )

    def create_auto_feed_command(self, hopper):
        """
        Feed the hopper only once the flywheel has reached its setpoint.

        :param hopper: Hopper subsystem used to feed balls.
        :type hopper: subsystems.hopper.Hopper
        :returns: Deferred command that either feeds or holds the hopper stopped.
        :rtype: commands2.Command
        """
        return DeferredCommand(
            lambda: (
                hopper.create_feed_cycle_command()
                if self.is_flywheel_at_setpoint()
                else hopper.create_stop_command()
            ),
            hopper
        )

    def create_stop_command(self):
        """
        Build a one-shot command that stops both shooter motors.
        """
        return self.runOnce(
            lambda: self.set_flywheel_velocities(0, 0)
        )
    
#Shoot functions
    def shoot(self, flywheel_target_velocity, intake_motor_velocity):
        """
        Build a one-shot command that applies shooter velocity setpoints.

        :param flywheel_target_velocity: Desired main flywheel velocity in rotations per second.
        :type flywheel_target_velocity: float
        :param intake_motor_velocity: Desired flywheel-intake velocity in rotations per second.
        :type intake_motor_velocity: float
        :returns: Command that applies the requested shooter setpoints once.
        :rtype: commands2.Command
        """
        return self.runOnce(
            lambda: self.set_flywheel_velocities(flywheel_target_velocity, intake_motor_velocity)
        )
    
    def set_flywheel_velocities(self, flywheel_target_velocity, intake_motor_velocity):
        """
        Apply the requested flywheel and flywheel-intake closed-loop velocities.

        :param flywheel_target_velocity: Desired main flywheel velocity in rotations per second.
        :type flywheel_target_velocity: float
        :param intake_motor_velocity: Desired flywheel-intake velocity in rotations per second.
        :type intake_motor_velocity: float
        """
        self._last_flywheel_target_rps = flywheel_target_velocity
        self.flywheel_motor.set_control(
            self.velocity_pid_request.with_velocity(flywheel_target_velocity)
        )
        self.flywheel_intake_motor.set_control(
            self.intake_velocity_pid_request.with_velocity(intake_motor_velocity)
        )

#Detect amp surge functions (to determine if the hopper is empty)
    def detect_empty(self):
        """
        Detect hopper-empty behavior by timing the last intake-motor surge.
        """
        # A surge is any moment the intake motor is not tracking its velocity setpoint
        self.surging = not self.flywheel_intake_motor.get_closed_loop_error().is_near(0, 1)

        if self.surging:
            self.empty_time = Timer.getFPGATimestamp()
            # Every amp surge resets empty_time, so empty_time holds the timestamp of the last surge.
            # The hopper is considered empty once no surge has occurred for EMPTY_TIMEOUT_SEC seconds.

        return (Timer.getFPGATimestamp() - self.empty_time) >= self.EMPTY_TIMEOUT_SEC
    
    def reset_empty_time(self):
        """
        Reset the empty-detection timer before starting a feed sequence.
        """
        self.empty_time = Timer.getFPGATimestamp()
        self.surging = True

