import commands2
from pathlib import Path

from commands2.sysid import SysIdRoutine
from phoenix6 import SignalLogger

from pathplannerlib.auto import AutoBuilder, NamedCommands, PathPlannerAuto

from wpimath.geometry import Pose2d, Rotation2d
from wpimath.units import inchesToMeters, feetToMeters
from wpilib import DriverStation, SmartDashboard, getDeployDirectory

from constants.shot_calculator_constants import get_hub_center
from constants.swerve_drivetrain_constants import SwerveDrivetrainConstants
from constants.shooter_constants import ShooterConstants
from constants.hopper_constants import HopperConstants
from constants.intake_constants import IntakeConstants
from constants.vision_constants import VisionConstants

class RobotContainer:
    def __init__(self):
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

        # Create vision subsystem
        self.camera = VisionConstants.create_vision(
            self.drivetrain.add_vision_measurement,
            self.drivetrain.get_state,
            self.drivetrain.get_robot_tilt,
        )

        # # Set starting pose for testing auto shooting (3 meters away from hub on red alliance)
        # alliance_color = DriverStation.getAlliance()
        # if alliance_color == DriverStation.Alliance.kRed:
        #     self.drivetrain.reset_pose(
        #         Pose2d(inchesToMeters(468.56 + 23.51) + feetToMeters(5), inchesToMeters(158.32), Rotation2d.fromDegrees(0))
        #     )
        # else:
        #     self.drivetrain.reset_pose(
        #         Pose2d(inchesToMeters(168.56 - 23.51) - feetToMeters(5), inchesToMeters(158.32), Rotation2d.fromDegrees(180))
        #     )
        
        self._last_observed_auto_signature: tuple[str | None, bool] | None = None
        self._last_seeded_auto_signature: tuple[str | None, bool] | None = None
        self._selected_auto_pose_seeded = False

        self.register_named_commands()

        self.auto_chooser = AutoBuilder.buildAutoChooser()
        SmartDashboard.putData("Auto Chooser", self.auto_chooser)

        self.create_button_bindings()

    def get_selected_auto_command(self):
        return self.auto_chooser.getSelected()

    def register_named_commands(self):
        NamedCommands.registerCommand(
            "Run Intake",
            self.intake.runOnce(lambda: self.intake.set_intake_speed(12.0))
        )

        NamedCommands.registerCommand(
            "Turn Off Intake",
            self.intake.runOnce(lambda: self.intake.set_intake_speed(0))
        )

        NamedCommands.registerCommand(
            "Auto Run Shooter",
            commands2.ParallelCommandGroup(
                self.intake.runOnce(lambda: self.intake.arm_down_intermediate()),
                self.shooter.create_auto_run_shooter_command(
                    self.hopper,
                    self.drivetrain,
                )
            )
        )

    def create_commands_auto(self):
        self.intake.arm_down()
        self.intake.set_intake_speed(0)
        self.hopper.run_hopper(0, 0),
        self.shooter.set_flywheel_velocities(0, 0)

    def create_commands_teleop(self):
        # Set the forward perspective of the robot for field oriented driving
        self.drivetrain.set_forward_perspective()
        self.drivetrain.reset_operator_heading_tracking()

        self.intake.arm_down()
        self.intake.set_intake_speed(0)
        self.hopper.run_hopper(0, 0),
        self.shooter.set_flywheel_velocities(0, 0)

    def create_button_bindings(self):
        # Set button binding for reseting field centric heading
        (self.controller.leftBumper() & self.controller.rightBumper()).onTrue(
            self.drivetrain.runOnce(
                lambda: (
                    self.drivetrain.seed_field_centric(),
                    self.drivetrain.set_forward_perspective(),
                    self.drivetrain.reset_operator_heading_tracking()
                )
            )
        )

        # Set default command for drivetrain
        self.drivetrain.setDefaultCommand(
            self.drivetrain.get_operator_drive_command(
                lambda: self.controller.getLeftTriggerAxis() > 0.10,
                lambda: self.controller.getRightTriggerAxis() > 0.10,
                lambda: self.controller.getLeftY(),
                lambda: self.controller.getLeftX(),
                lambda: self.controller.getRightX()
            )
        )

        self.controller.a().onTrue(
            commands2.SequentialCommandGroup(
                self.intake.runOnce(
                    lambda: self.intake.set_intake_speed(12)
                ),
                commands2.WaitCommand(2),
                commands2.WaitUntilCommand(
                    lambda: self.controller.getHID().getAButton()
                ),
                self.intake.runOnce(
                    lambda: self.intake.set_intake_speed(0)
                )
            )
        )

        self.controller.povRight().onTrue(
            commands2.ParallelCommandGroup(
                self.intake.runOnce(
                    lambda: self.intake.set_intake_speed(-12)
                ),
                self.hopper.run_hopper(-20, -3),
                self.shooter.set_flywheel_velocities(-30, -30)
            )
        )

        self.controller.povLeft().onTrue(
            self.drivetrain.runOnce(
                lambda: self.drivetrain.reset_pose_hub()
            )
        )

#GPT stuffs (bad)
        # self.controller.povUp().onTrue(
        #     self.intake.runOnce(
        #         lambda: self.intake.arm_up_manual()
        #     )
        # )

        # self.controller.povDown().onTrue(
        #     self.intake.runOnce(
        #         lambda: self.intake.arm_down_manual()
        #     )
        # )

#         self.controller.povLeft().onTrue(

#             self.intake.arm_down_please()
#         )

#         self.controller.povRight().onTrue(
#             self.intake.arm_up_please()
#         )

        self.controller.povDown().onTrue(
            self.intake.runOnce(
                lambda: self.intake.arm_down()
            )
        )

        self.controller.povUp().onTrue(
            self.intake.runOnce(
                lambda: self.intake.arm_up()
            )
        )
    
        self.controller.x().onTrue(
            commands2.SequentialCommandGroup(
                self.intake.runOnce(lambda: self.intake.arm_down_intermediate()),
                self.intake.runOnce(lambda: self.intake.set_intake_speed(12)),
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
                    # RepeatCommand(
                    #     self.drivetrain.auto_align_to_hub()
                    # )
                ),
                commands2.ParallelCommandGroup(
                    self.intake.runOnce(
                        lambda: self.intake.set_intake_speed(0)
                    ),
                    self.hopper.create_stop_command(),
                    self.shooter.create_stop_command()
                )
            )
        )

        self.controller.b().onTrue(
            commands2.SequentialCommandGroup(
                self.intake.runOnce(lambda: self.intake.arm_down_intermediate()),
                self.intake.runOnce(lambda: self.intake.set_intake_speed(12)),
                commands2.SequentialCommandGroup(
                    commands2.InstantCommand(
                        lambda: self.shooter.reset_calculated_shot_state()
                    ),
                    commands2.InstantCommand(
                        lambda: self.shooter.reset_empty_time()
                    )
                ),
                commands2.ParallelCommandGroup(
                    # commands2.WaitUntilCommand(
                    #     lambda: self.shooter.detect_empty()
                    # ),
                    commands2.RepeatCommand(
                        self.shooter.create_calculated_shoot_command()
                    ),
                    commands2.RepeatCommand(
                        self.shooter.create_calculated_feed_command(self.hopper)
                    )
                    # commands2.RepeatCommand(
                    #     self.drivetrain.auto_align_to_shot_angle(
                    #         self.shooter.get_latest_calculated_shot
                    #     )
                    # ),
                ).until(
                    lambda: self.controller.getHID().getYButton()
                ),
                commands2.ParallelCommandGroup(
                    self.intake.runOnce(
                        lambda: self.intake.set_intake_speed(0)
                    ),
                    self.hopper.create_stop_command(),
                    self.shooter.create_stop_command()
                )
            )
        )

        self.controller.y().onTrue(
            commands2.ParallelCommandGroup(
                self.intake.runOnce(
                    lambda: self.intake.set_intake_speed(0)
                ),
                self.hopper.create_stop_command(),
                self.shooter.create_stop_command()
            )
        )

    def create_commands_test(self):
        self.intake.set_intake_speed(0)
        self.hopper.run_hopper(0, 0),
        self.shooter.set_flywheel_velocities(0, 0)

        self.controller.x().onTrue(
            self.drivetrain.create_effective_wheel_radius_characterization_command()
        )
