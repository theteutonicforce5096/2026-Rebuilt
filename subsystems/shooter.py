from typing import Any, Callable, Final

from commands2 import (
    DeferredCommand,
    ParallelCommandGroup,
    ParallelDeadlineGroup,
    RepeatCommand,
    SequentialCommandGroup,
    Subsystem,
    WaitUntilCommand,
)
from commands2.sysid import SysIdRoutine
from ntcore import NetworkTableInstance
from phoenix6 import CANBus, SignalLogger
from phoenix6.configs import TalonFXConfiguration, TalonFXSConfiguration
from phoenix6.controls import VelocityVoltage, VoltageOut
from phoenix6.hardware import TalonFX, TalonFXS
from wpilib import RobotBase, SmartDashboard, Timer
from wpilib.sysid import SysIdRoutineLog
from wpimath.geometry import Rotation2d, Transform2d, Translation2d

from constants.shot_calculator_constants import calc_shooter_to_hub_distance
from subsystems.device_config import check_signal_status, configure_device
from subsystems.shot_calculator import ShotCalculator


class Shooter(Subsystem):
    """
    Runs the flywheels that launch balls into the hub.

    A main flywheel sets the launch speed and a smaller intake wheel feeds balls into it, both
    under closed-loop velocity control. Shot targets come either from the operator's dashboard
    entries or from a distance-based lookup, and the same sequence builder covers the manual,
    calculated, and autonomous shots.
    """

    DEFAULT_FLYWHEEL_TARGET_RPS: Final[float] = 60.0
    DEFAULT_FLYWHEEL_INTAKE_TARGET_RPS: Final[float] = 40.0

    # Ceiling on the operator-editable dashboard shot targets, just past the calibration table's
    # fastest entry
    MAX_FLYWHEEL_TARGET_RPS: Final[float] = 100.0

    # Seconds without an intake-motor surge before the hopper is considered empty
    EMPTY_TIMEOUT_SEC: Final[float] = 10.0

    # Closed-loop error above which the intake motor counts as surging (a ball feeding through)
    SURGE_ERROR_TOLERANCE_RPS: Final[float] = 1.0

    def __init__(
        self,
        canbus: CANBus,
        flywheel_motor_id: int,
        flywheel_intake_motor_id: int,
        flywheel_motor_configs: TalonFXSConfiguration,
        flywheel_intake_motor_configs: TalonFXConfiguration,
        num_config_attempts: int,
        get_current_swerve_state: Callable[[], Any],
        get_hub_center: Callable[[], Translation2d],
        launcher_offset_x: float,
        launcher_offset_y: float,
        flywheel_target_offset_rps: float,
    ):
        """
        Initialize the shooter using the specified constants.

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
        :param get_current_swerve_state: Function that returns the drivetrain state used by
            the shot solver.
        :type get_current_swerve_state: Callable[[], Any]
        :param get_hub_center: Function that returns the current hub center translation for the
            active alliance.
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
        configure_device(
            self.flywheel_intake_motor, flywheel_intake_motor_configs, num_config_attempts
        )

        if not RobotBase.isSimulation():
            # Trim every signal, then raise the ones the control logic reads each loop. The
            # closed-loop errors drive both setpoint and empty detection, so a stale one would
            # feed balls at the wrong moment. The velocities are dashboard-only.
            check_signal_status(
                self.flywheel_motor.optimize_bus_utilization(),
                "Shooter flywheel bus optimization",
            )
            check_signal_status(
                self.flywheel_intake_motor.optimize_bus_utilization(),
                "Shooter flywheel intake bus optimization",
            )
            check_signal_status(
                self.flywheel_motor.get_closed_loop_error().set_update_frequency(100.0),
                "Shooter flywheel closed-loop error update rate",
            )
            check_signal_status(
                self.flywheel_intake_motor.get_closed_loop_error().set_update_frequency(100.0),
                "Shooter flywheel intake closed-loop error update rate",
            )
            check_signal_status(
                self.flywheel_motor.get_velocity().set_update_frequency(50.0),
                "Shooter flywheel velocity update rate",
            )
            check_signal_status(
                self.flywheel_intake_motor.get_velocity().set_update_frequency(50.0),
                "Shooter flywheel intake velocity update rate",
            )

        # Create control requests
        self.velocity_pid_request = VelocityVoltage(velocity=0)
        self.intake_velocity_pid_request = VelocityVoltage(velocity=0)

        self.voltage_request = VoltageOut(output=0)

        # Empty detection state: the timestamp of the last current surge, and whether one is
        # happening right now
        self.empty_time = Timer.getFPGATimestamp()
        self.surging = False

        self._network_table_instance = NetworkTableInstance.getDefault()

        # Create SysId routine for characterizing flywheel motor
        def record_flywheel_state(state):
            SignalLogger.write_string(
                "SysId_Flywheel_Motor_State", SysIdRoutineLog.stateEnumToString(state)
            )

        def drive_flywheel(output):
            self.flywheel_motor.set_control(self.voltage_request.with_output(output))

        self.flywheel_motor_sys_id_routine = SysIdRoutine(
            SysIdRoutine.Config(
                rampRate=0.5,
                stepVoltage=9.0,
                timeout=15.0,
                recordState=record_flywheel_state,
            ),
            SysIdRoutine.Mechanism(
                drive_flywheel,
                lambda log: None,
                self,
            ),
        )

        # Create SysId routine for characterizing flywheel intake motor
        def record_flywheel_intake_state(state):
            SignalLogger.write_string(
                "SysId_Flywheel_Intake_Motor_State", SysIdRoutineLog.stateEnumToString(state)
            )

        def drive_flywheel_intake(output):
            self.flywheel_intake_motor.set_control(self.voltage_request.with_output(output))

        self.flywheel_intake_motor_sys_id_routine = SysIdRoutine(
            SysIdRoutine.Config(
                rampRate=0.5,
                stepVoltage=9.0,
                timeout=15.0,
                recordState=record_flywheel_intake_state,
            ),
            SysIdRoutine.Mechanism(
                drive_flywheel_intake,
                lambda log: None,
                self,
            ),
        )

        # Routines this subsystem can characterize. RobotContainer gathers these, along with the
        # other subsystems' routines, into the single SysId chooser on the dashboard.
        self.sys_id_routines = [
            ("Flywheel", self.flywheel_motor_sys_id_routine),
            ("Flywheel Intake", self.flywheel_intake_motor_sys_id_routine),
        ]

        # Operator-editable shot targets, published under the SmartDashboard table so they land
        # in the same tree as the rest of the robot's telemetry. A manual shot reads these every
        # loop, and reading through a subscriber is cheaper than a keyed SmartDashboard lookup.
        self._shooter_table = self._network_table_instance.getTable("SmartDashboard").getSubTable(
            "Shooter"
        )
        self.desired_flywheel_speed = self._shooter_table.getFloatTopic(
            "Desired Flywheel Speed (rps)"
        ).publish()
        self.desired_flywheel_speed_sub = self._shooter_table.getFloatTopic(
            "Desired Flywheel Speed (rps)"
        ).subscribe(self.DEFAULT_FLYWHEEL_TARGET_RPS)
        self.desired_flywheel_intake_speed = self._shooter_table.getFloatTopic(
            "Desired Flywheel Intake Speed (rps)"
        ).publish()
        self.desired_flywheel_intake_speed_sub = self._shooter_table.getFloatTopic(
            "Desired Flywheel Intake Speed (rps)"
        ).subscribe(self.DEFAULT_FLYWHEEL_INTAKE_TARGET_RPS)

        # Seed the widgets once at startup so they come up showing the same defaults the
        # subscribers fall back to. Publishing only here leaves later operator edits alone.
        self.desired_flywheel_speed.set(self.DEFAULT_FLYWHEEL_TARGET_RPS)
        self.desired_flywheel_intake_speed.set(self.DEFAULT_FLYWHEEL_INTAKE_TARGET_RPS)

        self._get_current_swerve_state = get_current_swerve_state
        self._get_hub_center = get_hub_center

        self.shot_calculator = ShotCalculator()
        self._last_flywheel_target_rps = 0.0

        # The launcher offset never changes, so the transform is built once here and reused by
        # every distance-based shot solve.
        self._shooter_offset = Transform2d(
            Translation2d(launcher_offset_x, launcher_offset_y),
            Rotation2d(),
        )

        # Constant flywheel speed offset added to every shot target for undershoot/overshoot trim
        self.flywheel_target_offset_rps = flywheel_target_offset_rps

    def periodic(self):
        """Publish the shooter's targets, velocities, and readiness to the dashboard."""
        SmartDashboard.putNumber("Shooter/Target (rps)", self._last_flywheel_target_rps)
        SmartDashboard.putNumber(
            "Shooter/Flywheel Velocity (rps)",
            self.flywheel_motor.get_velocity().value_as_double,
        )
        SmartDashboard.putNumber(
            "Shooter/Intake Velocity (rps)",
            self.flywheel_intake_motor.get_velocity().value_as_double,
        )
        SmartDashboard.putNumber(
            "Shooter/Flywheel Error (rps)",
            self.flywheel_motor.get_closed_loop_error().value_as_double,
        )
        SmartDashboard.putNumber(
            "Shooter/Intake Error (rps)",
            self.flywheel_intake_motor.get_closed_loop_error().value_as_double,
        )
        SmartDashboard.putBoolean("Shooter/At Setpoint", self.is_flywheel_at_setpoint())

        # A calculated shot holds fire outside the calibrated band, so both the distance and the
        # verdict are published; without them a held shot looks like a broken one from the driver
        # station.
        shot_distance = self.get_shot_distance()
        SmartDashboard.putNumber("Shooter/Distance (m)", shot_distance)
        SmartDashboard.putBoolean(
            "Shooter/In Range", self.shot_calculator.is_distance_in_range(shot_distance)
        )

        # detect_empty refreshes the surge state as a side effect, so it runs before surging is
        # published. Calling it here is safe because every shot reseeds the timer when it starts,
        # so this extra call cannot shorten a real empty timeout.
        is_empty = self.detect_empty()
        SmartDashboard.putBoolean("Shooter/Surging", self.surging)
        SmartDashboard.putBoolean("Shooter/Empty", is_empty)

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

    def get_shot_distance(self) -> float:
        """
        Measure the shooter-to-hub distance from the current fused pose.

        :returns: Distance in meters from the shooter to the hub center.
        :rtype: float
        """
        return calc_shooter_to_hub_distance(
            self._get_current_swerve_state().pose,
            self._get_hub_center(),
            self._shooter_offset,
        )

    def is_shot_in_range(self) -> bool:
        """
        Report whether the robot is standing where a calculated shot has calibration behind it.

        Outside the calibrated band the shot table clamps to its nearest measured point, and close
        enough to the hub the physics model has no solution at all, so a distance-based shot has
        to hold fire rather than launch on an extrapolated target.

        :returns: True when the current shot distance falls inside the calibrated range.
        :rtype: bool
        """
        return self.shot_calculator.is_distance_in_range(self.get_shot_distance())

    def get_current_auto_shot_targets(self) -> tuple[float, float, float]:
        """
        Solve the distance-based shot targets from where the robot is standing right now.

        Out of range the targets come back zeroed, which reads as "not at setpoint" everywhere
        downstream and so holds the feed and the flywheel intake off.

        :returns: Shooter-to-hub distance in meters, flywheel target in rotations per second,
            and flywheel-intake target in rotations per second.
        :rtype: tuple[float, float, float]
        """
        distance_m = self.get_shot_distance()
        if not self.shot_calculator.is_distance_in_range(distance_m):
            return distance_m, 0.0, 0.0

        flywheel_target_velocity = self.shot_calculator.get_profile_for_distance(distance_m)
        return distance_m, flywheel_target_velocity, self.DEFAULT_FLYWHEEL_INTAKE_TARGET_RPS

    def create_shot_command(
        self,
        hopper,
        drivetrain=None,
        *,
        distance_based,
        end_when,
        release_to_driver_after_aim=False,
    ):
        """
        Build the shooting sequence used by the manual, calculated, and autonomous shots.

        The sequence spins the flywheel up, waits until it is ready, then feeds the hopper until
        the end condition trips. The three shots differ only in where the flywheel target comes
        from and whether the robot aims itself first, so these are supplied by the caller.

        By default the robot holds its aim for the whole shot and the hopper only runs while the
        flywheel is at speed, which is what an unattended shot wants. Handing control back is opt-in
        (see release_to_driver_after_aim) because it only makes sense where a driver is there to
        take it.

        :param hopper: Hopper subsystem used to feed balls toward the shooter.
        :type hopper: subsystems.hopper.Hopper
        :param drivetrain: Drivetrain subsystem to aim at the hub while spinning up, or None to
            skip aiming (manual shot).
        :type drivetrain: subsystems.swerve_drivetrain.SwerveDrivetrain | None
        :param distance_based: True solves the flywheel target from the live hub distance; False
            reads the operator's NetworkTables targets.
        :type distance_based: bool
        :param end_when: Predicate that ends the feeding phase (button released, hopper empty, ...).
        :type end_when: Callable[[], bool]
        :param release_to_driver_after_aim: True hands the drivetrain back once aiming finishes and
            lets the hopper keep running while the flywheel chases the moving target, for a shot a
            driver may reposition mid-way. False holds aim and gates the hopper on flywheel speed
            for the whole shot.
        :type release_to_driver_after_aim: bool
        :returns: Full shooting sequence command.
        :rtype: commands2.Command
        """
        alignment_timeout = Timer()

        def targets():
            if distance_based:
                _, flywheel_rps, intake_rps = self.get_current_auto_shot_targets()
                return flywheel_rps, intake_rps
            # The dashboard entries are operator-typed, so clamp them to a range the flywheel
            # can actually run at before commanding them.
            return (
                min(
                    max(self.desired_flywheel_speed_sub.get(), 0.0),
                    self.MAX_FLYWHEEL_TARGET_RPS,
                ),
                min(
                    max(self.desired_flywheel_intake_speed_sub.get(), 0.0),
                    self.MAX_FLYWHEEL_TARGET_RPS,
                ),
            )

        # The most recent in-range solve. Driving outside the calibrated band mid-shot would
        # otherwise solve to zero and spin the flywheel down, so the last good speed is held
        # instead and the shot carries on at the edge of what the table can vouch for.
        last_valid: list[tuple[float, float]] = []

        def apply(feed):
            flywheel_rps, intake_rps = targets()
            if distance_based:
                if flywheel_rps > 0.0:
                    last_valid[:] = [(flywheel_rps, intake_rps)]
                elif feed and last_valid:
                    flywheel_rps, intake_rps = last_valid[0]

            self.set_flywheel_velocities(
                # A zeroed target means there is no shot to take, so the trim offset stays off it
                # and the flywheel is left stopped rather than spun to the offset alone.
                flywheel_rps + self.flywheel_target_offset_rps if flywheel_rps > 0.0 else 0.0,
                # Waiting for the flywheel is right whenever the target sits still: a dip means a
                # ball or the battery loaded it down, and pausing lets it recover. It is wrong only
                # when the driver is moving the robot, because the target then moves too - the
                # calibration shifts about 7.5 rotations per second per metre against a one
                # rotation per second tolerance, so the feed would stop at every reposition, which
                # is when it is least wanted.
                intake_rps
                if feed and (release_to_driver_after_aim or self.is_flywheel_at_setpoint())
                else 0.0,
            )

        def ready():
            # Feeding starts once the flywheel is at speed and, when aligning, the robot is on
            # target. The timeout keeps a shot from stalling forever if alignment never settles,
            # but it must not release a shot the calibration cannot aim, so the range check is
            # outside it.
            if distance_based and not self.is_shot_in_range():
                return False

            return self.is_flywheel_at_setpoint() and (
                drivetrain is None
                or drivetrain.is_hub_alignment_within_tolerance()
                or alignment_timeout.hasElapsed(1.0)
            )

        def feed_cycle():
            if release_to_driver_after_aim:
                return hopper.create_feed_cycle_command()
            # Only run a hopper cycle once the flywheel is at speed, otherwise hold it stopped.
            return DeferredCommand(
                lambda: (
                    hopper.create_feed_cycle_command()
                    if self.is_flywheel_at_setpoint()
                    else hopper.create_stop_command()
                ),
                hopper,
            )

        # Alignment is only added when a drivetrain is supplied, which is what makes the manual
        # shot a fixed-heading shot.
        spin_up_extras = []
        feed_extras = []
        stop_extras = []
        if drivetrain is not None:
            if release_to_driver_after_aim:
                # Proxied so the drivetrain is not folded into this composition's requirements: a
                # command group reserves the union of everything it contains for its whole life,
                # which would keep the driver locked out long after aiming finished. Proxied, it is
                # held only while the spin-up phase runs and is handed back the moment that phase
                # ends, whether alignment settled or ran out its timeout.
                spin_up_extras.append(drivetrain.create_hold_hub_alignment_command().asProxy())
            else:
                # Nobody is waiting to take the drivetrain, so the robot holds its aim for the
                # whole shot and stops itself at the end.
                spin_up_extras.append(drivetrain.create_hold_hub_alignment_command())
                feed_extras.append(drivetrain.create_hold_hub_alignment_command())
                stop_extras.append(drivetrain.create_stop_command())

        return SequentialCommandGroup(
            self.runOnce(lambda: self.reset_empty_time()),
            # Cleared alongside the timer because the command instance is reused across presses,
            # so a speed held from the previous shot must not leak into this one.
            self.runOnce(lambda: last_valid.clear()),
            self.runOnce(lambda: alignment_timeout.restart()),
            # Spin up (and align) until ready, but let the end condition bail out early.
            ParallelDeadlineGroup(
                WaitUntilCommand(lambda: ready() or end_when()),
                self.run(lambda: apply(False)),
                *spin_up_extras,
            ),
            # Feed until the end condition trips. A distance-based target keeps resolving from the
            # live pose, so a robot that moves during this phase steers the flywheel toward the
            # shot it has moved to rather than leaving it solved for the one it left.
            ParallelDeadlineGroup(
                WaitUntilCommand(end_when),
                self.run(lambda: apply(True)),
                RepeatCommand(feed_cycle()),
                *feed_extras,
            ),
            ParallelCommandGroup(
                hopper.create_stop_command(),
                self.create_stop_command(),
                *stop_extras,
            ),
        )

    def create_stop_command(self):
        """
        Build a one-shot command that stops both shooter motors.

        :returns: Command that zeroes both flywheel targets.
        :rtype: commands2.Command
        """
        return self.runOnce(lambda: self.set_flywheel_velocities(0, 0))

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

    def detect_empty(self):
        """
        Report whether the hopper has run dry, based on how long ago the last ball fed through.

        Each ball briefly loads the flywheel-intake motor as it passes, which shows up as the
        motor falling behind its velocity setpoint. Timing the gap between those surges avoids
        needing a dedicated ball sensor.

        :returns: True once no ball has passed through for EMPTY_TIMEOUT_SEC seconds.
        :rtype: bool
        """
        # A surge is any moment the intake motor is not tracking its velocity setpoint
        self.surging = not self.flywheel_intake_motor.get_closed_loop_error().is_near(
            0, self.SURGE_ERROR_TOLERANCE_RPS
        )

        # Every surge pushes the timestamp forward, so empty_time holds the moment the last ball
        # went through.
        if self.surging:
            self.empty_time = Timer.getFPGATimestamp()

        return (Timer.getFPGATimestamp() - self.empty_time) >= self.EMPTY_TIMEOUT_SEC

    def reset_empty_time(self):
        """Restart the empty-detection timer so a new feed sequence starts with a full timeout."""
        self.empty_time = Timer.getFPGATimestamp()
