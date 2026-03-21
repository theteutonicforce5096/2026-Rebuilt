import commands2
from commands2 import WaitUntilCommand, ParallelDeadlineGroup, ParallelCommandGroup, SequentialCommandGroup, RepeatCommand, WaitCommand
from commands2.sysid import SysIdRoutine
from phoenix6 import SignalLogger

from pathplannerlib.auto import AutoBuilder, PathConstraints, PathPlannerAuto, NamedCommands
from pathplannerlib.path import PathPlannerPath, IdealStartingState, GoalEndState

from wpimath.geometry import Pose2d, Rotation2d, Translation2d
from wpimath.units import inchesToMeters, feetToMeters

from constants.physics import get_hub_center
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

        #Create controller
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

        # #Create hoppper subsystem
        self.hopper = HopperConstants.create_hopper()

        # #Create intake subsystem
        self.intake = IntakeConstants.create_intake()

        # Create vision subsystem
        self.camera = VisionConstants.create_vision(
            self.drivetrain.add_vision_measurement,
            self.drivetrain.get_state,
            self.drivetrain.get_robot_tilt
        )

        # Set starting pose for testing auto shooting (3 meters away from hub on red alliance)
        self.drivetrain.reset_pose(
            Pose2d(inchesToMeters(468.56 + 23.51) + feetToMeters(10), inchesToMeters(158.32), Rotation2d.fromDegrees(180))
        )
        
        NamedCommands.registerCommand("shot_one", '''command goes here''')
        NamedCommands.registerCommand("shot_two", '''command goes here''')

        self.create_button_bindings()

    def create_commands_auto(self):
        pass

    def create_commands_teleop(self):   
        # Set the forward perspective of the robot for field oriented driving
        self.drivetrain.set_forward_perspective()
        
    def create_button_bindings(self):
        # Set button binding for reseting field centric heading
        (self.controller.leftBumper() & self.controller.rightBumper()).onTrue(
            self.drivetrain.runOnce(lambda: self.drivetrain.seed_field_centric())
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
            SequentialCommandGroup(
                self.intake.runOnce(
                    lambda: self.intake.set_intake_speed(5)
                ),
                # WaitCommand(2),
                WaitUntilCommand(
                    lambda: self.controller.getHID().getAButtonPressed()
                ),
                self.intake.runOnce(
                    lambda: self.intake.set_intake_speed(0)
                )   
            )
        )

        self.controller.povUp().onTrue(
            self.intake.arm_up()
        )

        self.controller.povDown().onTrue(
            self.intake.arm_down()
        )

        self.controller.x().onTrue(
            SequentialCommandGroup(
                ParallelDeadlineGroup(
                    WaitUntilCommand(
                        lambda: self.controller.getHID().getYButton()
                    ),
                    ParallelCommandGroup(
                        RepeatCommand(
                            self.shooter.create_manual_shoot_command()
                        ),
                        RepeatCommand(
                            self.hopper.create_feed_cycle_command()
                        ),
                        RepeatCommand(
                            self.drivetrain.auto_align_to_hub()
                        )
                    )
                ),
                ParallelCommandGroup(
                    self.hopper.create_stop_command(),
                    self.shooter.create_stop_command()
                )
            )
        )

        self.controller.b().onTrue(
            SequentialCommandGroup(
                ParallelDeadlineGroup(
                    SequentialCommandGroup(        
                        commands2.InstantCommand(
                            lambda: self.shooter.reset_calculated_shot_state()
                        ),
                        commands2.InstantCommand(
                            lambda: self.shooter.reset_empty_time()
                        ),
                        ParallelCommandGroup(
                            WaitUntilCommand(
                                lambda: self.shooter.detect_empty()
                            ),
                            WaitUntilCommand(
                                lambda: self.controller.getHID().getYButton()
                            )
                        )
                    ),
                    ParallelCommandGroup(
                        RepeatCommand(
                            self.shooter.create_calculated_shoot_command()
                        ),
                        RepeatCommand(
                            self.shooter.create_calculated_feed_command(self.hopper)
                        ),
                        RepeatCommand(
                            self.drivetrain.auto_align_to_shot_angle(
                                self.shooter.get_latest_calculated_shot
                            )
                        )
                    )
                ),
                ParallelCommandGroup(
                    self.hopper.create_stop_command(),
                    self.shooter.create_stop_command()
                )
            )
        )

        self.controller.y().onTrue(
            ParallelCommandGroup(
                self.hopper.create_stop_command(),
                self.shooter.create_stop_command()
            )
        )

    def create_commands_auto(self):
        self.intake.runOnce(
            lambda: self.intake.arm_down()
        ).schedule()

    def create_commands_teleop(self):   
        # Set the forward perspective of the robot for field oriented driving
        self.drivetrain.set_forward_perspective()
        
        # # Estimated button bindings for shooter (subject to change):
        #     - X button: Shoot at speeds specified by network tables
        #     - B button: Shoot at calculated speed based on distance to target
        #     - POV down: Stop shooting no matter what mode is being used
        #     - A button: Auto move button (figure out to where later)
        #     - Y button: Cancel auto move command
     
        #     )
        # )
        # # TODO: Add in shooter functions and check if this even works????
        # self.controller.leftTrigger().onTrue(
        #     commands2.ParallelRaceGroup(
        #         commands2.SequentialCommandGroup(
        #             commands2.DeferredCommand(lambda: self.drivetrain._auto_align_to_hub(self.shooter),
        #                                       self.drivetrain), 
        #             commands2.ParallelDeadlineGroup(
        #                 commands2.DeferredCommand(
        #                     lambda: self.shooter.auto_shoot_distance(
        #                         self.drivetrain.get_distance_to_hub()
        #                     )
        #                 ), self.shooter
        #             ).until(lambda: self.shooter.no_change_amps()),
        #             self.drivetrain.set_brake_mode()
        #         )
        #     ,
        #     commands2.WaitUntilCommand(lambda: self.controller.getHID().getBButton())
        #     )
        # )
        
        # self.controller.rightTrigger().onTrue(
        #     self.shooter.runOnce(
        #         lambda: self.shooter.shoot(
        #             shoot_speed(
        #                 calc_velocity(
        #                     calc_x_dis(
        #                         self.drivetrain.get_state().pose.X(), 
        #                         self.drivetrain.get_state().pose.Y()))), 
        #             flywheel_intake_speed()
        #         ))
        # )

        # self.controller.x().onTrue(
        #     self.shooter.runOnce(
        #         lambda: self.shooter.stop())
        # )
        
        # #Max rps of flywheel (neo vortex) = 113
        # #Max rps of flywheel intake (falcon 500) = 106
        # #We convert the percents to rps
        # self.controller.x().onTrue(
        #     self.shooter.runOnce(
        #         lambda: self.shooter.shoot(
        #             # 0.5 * 113, 
        #             # 0.25 * 106)
        #             self.shooter.desired_ball_speed_sub.get() * 113, 
        #             self.shooter.desired_flywheel_intake_speed_sub.get() * 106)
        #         )
        #     )


    def create_commands_test(self):
        # # Set the SysId routine to run
        self.drivetrain.set_sys_id_routine()
    
        # Set button bindings for starting and stopping SignalLogger
        self.controller.leftBumper().onTrue(commands2.cmd.runOnce(SignalLogger.start))
        self.controller.rightBumper().onTrue(commands2.cmd.runOnce(SignalLogger.stop))

        # Set button bindings for performing various parts of SysID routine
        self.controller.y().whileTrue(self.drivetrain.sys_id_dynamic(SysIdRoutine.Direction.kForward))
        self.controller.a().whileTrue(self.drivetrain.sys_id_dynamic(SysIdRoutine.Direction.kReverse))
        self.controller.b().whileTrue(self.drivetrain.sys_id_quasistatic(SysIdRoutine.Direction.kForward))
        self.controller.x().whileTrue(self.drivetrain.sys_id_quasistatic(SysIdRoutine.Direction.kReverse))
