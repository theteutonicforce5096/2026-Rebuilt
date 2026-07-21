import commands2

from commands2.sysid import SysIdRoutine

from pathplannerlib.auto import AutoBuilder, NamedCommands

from wpilib import DriverStation, SendableChooser, SmartDashboard

from constants.shot_calculator_constants import get_hub_center
from constants.swerve_drivetrain_constants import SwerveDrivetrainConstants
from constants.shooter_constants import ShooterConstants
from constants.hopper_constants import HopperConstants
from constants.intake_constants import IntakeConstants
from constants.vision_constants import VisionConstants
from constants.led_constants import LEDConstants

class RobotContainer:
    """
    Construct the robot subsystems, controller bindings, and autonomous chooser.
    """

    # Trigger travel past which a controller trigger counts as pressed
    _TRIGGER_DEADBAND = 0.10

    def __init__(self):
        """
        Create the shared robot subsystems and operator interface wiring.
        """
        # Define max speed variables
        self.max_linear_speed = SwerveDrivetrainConstants._max_linear_speed
        self.max_angular_rate = SwerveDrivetrainConstants._max_angular_speed

        # Create controller
        self.controller = commands2.button.CommandXboxController(0)

        # Create drivetrain subsystem
        self.drivetrain = SwerveDrivetrainConstants.create_drivetrain()

        launcher_offset = self.drivetrain.shooter_offset.translation()

        # Create shooter subsystem
        self.shooter = ShooterConstants.create_shooter(
            self.drivetrain.get_state,
            self.drivetrain.get_robot_tilt,
            lambda: get_hub_center(
                self.drivetrain.field_type,
                self.drivetrain.current_alliance,
            ),
            launcher_offset.x,
            launcher_offset.y,
        )

        # Create hopper subsystem
        self.hopper = HopperConstants.create_hopper()

        # Create intake subsystem
        self.intake = IntakeConstants.create_intake()

        # Create LED subsystem
        self.led = LEDConstants.create_led()

        # Create vision subsystem
        self.camera = VisionConstants.create_vision(
            self.drivetrain.add_vision_measurement,
            self.drivetrain.get_state,
            self.drivetrain.get_robot_tilt,
        )

        self.register_named_commands()

        self.auto_chooser = AutoBuilder.buildAutoChooser()
        SmartDashboard.putData("Auto Chooser", self.auto_chooser)

        # Chooser for which subsystem the test-mode SysId buttons characterize.
        # Each subsystem selects its own routine through its per-subsystem "Routines" chooser.
        self.sys_id_subsystem_chooser = SendableChooser()
        self.sys_id_subsystem_chooser.setDefaultOption("Drivetrain", self.drivetrain)
        self.sys_id_subsystem_chooser.addOption("Shooter", self.shooter)
        self.sys_id_subsystem_chooser.addOption("Hopper", self.hopper)
        SmartDashboard.putData("SysId Subsystem", self.sys_id_subsystem_chooser)

        self.create_button_bindings()

    def get_selected_auto_command(self):
        """
        Return the autonomous command selected on the dashboard chooser.
        """
        return self.auto_chooser.getSelected()

    def register_named_commands(self):
        """
        Register PathPlanner named commands used by autonomous routines.
        """
        NamedCommands.registerCommand(
            "Run Intake",
            self.intake.runOnce(lambda: self.intake.set_intake_speed(IntakeConstants._intake_volts))
        )

        NamedCommands.registerCommand(
            "Turn Off Intake",
            self.intake.runOnce(lambda: self.intake.set_intake_speed(0))
        )

        NamedCommands.registerCommand(
            "Auto Run Shooter",
            commands2.ParallelCommandGroup(
                commands2.SequentialCommandGroup(
                    commands2.WaitCommand(ShooterConstants._auto_arm_down_delay_sec),
                    self.intake.runOnce(lambda: self.intake.arm_down_intermediate())
                ),
                self.shooter.create_auto_run_shooter_command(
                    self.hopper,
                    self.drivetrain,
                )
            )
        )

    def create_commands_auto(self):
        """
        Put subsystems into a known safe state before autonomous begins.
        """
        self.intake.arm_down()
        self.intake.set_intake_speed(0)
        self.hopper.set_hopper_speeds(0, 0)
        self.shooter.set_flywheel_velocities(0, 0)
        self.led.auto_in_progress()

    def create_commands_teleop(self):
        """
        Reset operator-facing state and safe subsystem outputs for teleop.
        """
        # Set the forward perspective of the robot for field oriented driving
        self.drivetrain.set_forward_perspective()
        self.drivetrain.reset_teleop_drive_state()

        self.intake.arm_down()
        self.intake.set_intake_speed(0)
        self.hopper.set_hopper_speeds(0, 0)
        self.shooter.set_flywheel_velocities(0, 0)
        self.led.default()

    def create_button_bindings(self):
        """
        Bind controller buttons to robot actions and default commands.

        All bindings are created once here. Mode-specific bindings are gated by a
        mode-scoped trigger (teleop vs. test) so they only fire in their own mode and
        do not stack when a mode is re-entered. The color buttons drive shooting in
        teleop and SysId characterization in test.
        """
        # Mode-scope triggers: a mode-specific binding is only live while its mode is enabled.
        teleop = commands2.button.Trigger(lambda: DriverStation.isTeleopEnabled())
        test = commands2.button.Trigger(lambda: DriverStation.isTestEnabled())

        # Set default command for drivetrain
        self.drivetrain.setDefaultCommand(
            self.drivetrain.get_operator_drive_command(
                lambda: self.controller.getLeftTriggerAxis() > self._TRIGGER_DEADBAND,
                lambda: self.controller.getRightTriggerAxis() > self._TRIGGER_DEADBAND,
                lambda: -self.controller.getLeftY(),
                lambda: -self.controller.getLeftX(),
                lambda: -self.controller.getRightX()
            ).beforeStarting(
                self.drivetrain.runOnce(
                    lambda: self.drivetrain.reset_teleop_drive_state()
                )
            )
        )

        # --- Teleop bindings ---

        # Set button binding for reseting field centric heading
        (self.controller.leftBumper() & self.controller.rightBumper() & teleop).onTrue(
            self.drivetrain.runOnce(
                lambda: (
                    self.drivetrain.seed_field_centric(),
                    self.drivetrain.set_forward_perspective(),
                    self.drivetrain.reset_teleop_drive_state()
                )
            )
        )

        (self.controller.a() & teleop).onTrue(
            commands2.SequentialCommandGroup(
                self.intake.runOnce(
                    lambda: self.intake.set_intake_speed(IntakeConstants._intake_volts)
                ),
                commands2.WaitCommand(IntakeConstants._intake_min_run_sec),
                commands2.WaitUntilCommand(
                    lambda: self.controller.getHID().getAButton()
                ),
                self.intake.runOnce(
                    lambda: self.intake.set_intake_speed(0)
                )
            )
        )

        (self.controller.povRight() & teleop).onTrue(
            commands2.ParallelCommandGroup(
                self.intake.runOnce(
                    lambda: self.intake.set_intake_speed(IntakeConstants._eject_volts)
                ),
                self.hopper.run_hopper(
                    HopperConstants._eject_mecanum_velocity,
                    HopperConstants._eject_agitator_volts,
                ),
                self.shooter.runOnce(
                    lambda: self.shooter.set_flywheel_velocities(
                        ShooterConstants._eject_flywheel_velocity,
                        ShooterConstants._eject_flywheel_velocity,
                    )
                )
            )
        )

        (self.controller.povLeft() & teleop).onTrue(
            self.drivetrain.runOnce(
                lambda: self.drivetrain.reset_pose_hub()
            )
        )

        (self.controller.povDown() & teleop).onTrue(
            self.intake.runOnce(
                lambda: self.intake.arm_down()
            )
        )

        (self.controller.povUp() & teleop).onTrue(
            self.intake.runOnce(
                lambda: self.intake.arm_up()
            )
        )

        (self.controller.x() & teleop).onTrue(
            commands2.SequentialCommandGroup(
                self.led.runOnce(
                    lambda: self.led.shooting_manual()
                ),
                self.intake.runOnce(lambda: self.intake.set_intake_speed(IntakeConstants._intake_volts)),
                commands2.ParallelDeadlineGroup(
                    commands2.WaitUntilCommand(
                        lambda: self.controller.getHID().getYButton()
                    ),
                    commands2.RepeatCommand(
                        self.shooter.create_manual_shoot_command()
                    ),
                    commands2.RepeatCommand(
                        self.shooter.create_manual_feed_command(self.hopper)
                    )
                ),
                commands2.ParallelCommandGroup(
                    self.intake.runOnce(
                        lambda: self.intake.set_intake_speed(0)
                    ),
                    self.hopper.create_stop_command(),
                    self.shooter.create_stop_command(),
                    self.led.runOnce(
                        lambda: self.led.default()
                    )
                )
            )
        )

        (self.controller.b() & teleop).onTrue(
            commands2.SequentialCommandGroup(
                self.led.runOnce(
                    lambda: self.led.shooting_calculated()
                ),
                self.intake.runOnce(lambda: self.intake.set_intake_speed(IntakeConstants._intake_volts)),
                commands2.ParallelCommandGroup(
                    commands2.RepeatCommand(
                        self.shooter.create_auto_shoot_command()
                    ),
                    commands2.RepeatCommand(
                        self.shooter.create_auto_feed_command(self.hopper)
                    )
                ).until(
                    lambda: self.controller.getHID().getYButton()
                ),
                commands2.ParallelCommandGroup(
                    self.intake.runOnce(
                        lambda: self.intake.set_intake_speed(0)
                    ),
                    self.hopper.create_stop_command(),
                    self.shooter.create_stop_command(),
                    self.led.runOnce(
                        lambda: self.led.default()
                    )
                )
            )
        )

        (self.controller.y() & teleop).onTrue(
            commands2.ParallelCommandGroup(
                self.led.runOnce(
                    lambda: self.led.default()
                ),
                self.intake.runOnce(
                    lambda: self.intake.set_intake_speed(0)
                ),
                self.hopper.create_stop_command(),
                self.shooter.create_stop_command()
            )
        )

        # Flash the "five seconds left" LED state near the end of a real teleop period,
        # then return to the default state. Gated to a real timing source: getMatchTime()
        # returns -1 without one, so require 0 < time <= 5 plus FMS or a practice match.
        five_seconds_left = commands2.button.Trigger(
            lambda: DriverStation.isTeleopEnabled()
            and 0 < DriverStation.getMatchTime() <= 5
            and (
                DriverStation.isFMSAttached()
                or DriverStation.getMatchType() == DriverStation.MatchType.kPractice
            )
        )
        five_seconds_left.onTrue(
            self.led.runOnce(lambda: self.led.five_seconds_left())
        ).onFalse(
            self.led.runOnce(lambda: self.led.default())
        )

        # --- Test bindings (SysId characterization + drivetrain diagnostics) ---

        # Color buttons run SysId on the dashboard-selected subsystem + its selected routine.
        (self.controller.y() & test).whileTrue(
            self._create_sys_id_command(
                lambda subsystem: subsystem.sys_id_quasistatic(SysIdRoutine.Direction.kForward)
            )
        )
        (self.controller.a() & test).whileTrue(
            self._create_sys_id_command(
                lambda subsystem: subsystem.sys_id_quasistatic(SysIdRoutine.Direction.kReverse)
            )
        )
        (self.controller.b() & test).whileTrue(
            self._create_sys_id_command(
                lambda subsystem: subsystem.sys_id_dynamic(SysIdRoutine.Direction.kForward)
            )
        )
        (self.controller.x() & test).whileTrue(
            self._create_sys_id_command(
                lambda subsystem: subsystem.sys_id_dynamic(SysIdRoutine.Direction.kReverse)
            )
        )

        (self.controller.povUp() & test).onTrue(
            self.drivetrain.create_effective_wheel_radius_characterization_command()
        )

        # Hold the modules forward and publish their azimuth errors for offset calibration
        (self.controller.povDown() & test).whileTrue(
            self.drivetrain.create_module_alignment_diagnostic_command()
        )

    def _create_sys_id_command(self, run_routine):
        """
        Build a deferred SysId command targeting the dashboard-selected subsystem.

        The subsystem and its routine are resolved at schedule time so the dashboard
        choosers can be changed between runs. The command requires all characterizable
        subsystems so the deferred inner command's requirement is always satisfied.

        :param run_routine: Callable mapping the selected subsystem to the SysId command
            to run (quasistatic/dynamic in a given direction).
        :type run_routine: Callable[[commands2.Subsystem], commands2.Command]
        :returns: Deferred command that runs the selected subsystem's SysId routine.
        :rtype: commands2.Command
        """
        def build():
            subsystem = self.sys_id_subsystem_chooser.getSelected()
            subsystem.set_sys_id_routine()
            return run_routine(subsystem)

        return commands2.DeferredCommand(
            build, self.drivetrain, self.shooter, self.hopper
        )

    def create_commands_test(self):
        """
        Reset subsystem outputs for test mode.

        Test-mode button bindings live in create_button_bindings, scoped to test mode,
        so they are created once instead of re-bound on every entry into test mode.
        """
        self.intake.set_intake_speed(0)
        self.hopper.set_hopper_speeds(0, 0)
        self.shooter.set_flywheel_velocities(0, 0)
        self.led.default()
