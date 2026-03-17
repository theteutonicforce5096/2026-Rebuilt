import commands2
from commands2.sysid import SysIdRoutine
import wpilib
from wpilib import DriverStation
from phoenix6.configs import TalonFXConfiguration, TalonFXSConfiguration, CANcoderConfiguration
from phoenix6 import signals
from phoenix6 import SignalLogger

from pathplannerlib.auto import AutoBuilder, PathConstraints
from pathplannerlib.path import PathPlannerPath, IdealStartingState, GoalEndState
from pathplannerlib.auto import PathPlannerAuto

from wpimath.geometry import Pose2d, Rotation2d
from wpimath.units import inchesToMeters, feetToMeters

from constants.swerve_drivetrain_constants import SwerveDrivetrainConstants
from constants.shooter_constants import ShooterConstants
from constants.hopper_constants import HopperConstants
from constants.intake_constants import IntakeConstants
from subsystems.LED_controller import LED
from constants.physics import calc_velocity, calc_x_dis, shoot_speed, flywheel_intake_speed


class RobotContainer:
    def __init__(self):
        # Define max speed variables
        self.max_linear_speed = SwerveDrivetrainConstants._max_linear_speed
        self.max_angular_rate = SwerveDrivetrainConstants._max_angular_speed

        #Create controller
        self.controller = commands2.button.CommandXboxController(0)
        
        # Create drivetrain subsystem
        self.drivetrain = SwerveDrivetrainConstants.create_drivetrain()
                     
        # Create shooter subsystem
        self.shooter = ShooterConstants.create_shooter()

        # #Create hoppper subsystem
        # self.hopper = HopperConstants.create_hopper()

        # #Create intake subsystem
        # self.intake = IntakeConstants.create_intake()

        # Initialize LEDs
        # self.led = LED()
        # self.last_time = None
        # ^^^ I need this for LEDs

        # Set starting pose for testing auto shooting (3 meters away from hub on red alliance)
        self.drivetrain.reset_pose(
            Pose2d(inchesToMeters(468.56 + 23.51) + feetToMeters(10), inchesToMeters(158.32), Rotation2d.fromDegrees(180))
        )
        
        self.create_button_bindings()


    # def create_commands_periodic(self):
    #     if DriverStation.isAutonomousEnabled():
    #         self.led.auto_in_progress()

    #     self.MATCHTIME = int(DriverStation.getMatchTime())

    #     # I need this so the LEDs won't constantly be trying to change to the same animation
    #     if self.MATCHTIME != self.last_time:
    #         self.last_time = self.MATCHTIME
    #         if DriverStation.isTeleopEnabled():
    #             # The numbers are 5 seconds before each phase switch
    #             if self.MATCHTIME in (115, 90, 65, 40, 25): 
    #                 self.led.five_seconds_left()

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
          
        # self.controller.y().onTrue(
        #     commands2.DeferredCommand(
        #         self.drivetrain.auto_align_to_hub,
        #         self.drivetrain
        #     ).until(
        #         lambda: self.controller.getHID().getBButton()
        #     )
        # )

        self.controller.a().onTrue(
            commands2.SequentialCommandGroup(
                commands2.DeferredCommand(
                    self.drivetrain.auto_align_to_hub,
                    self.drivetrain
                ).until(
                    lambda: self.controller.getHID().getXButton()
                ),
                commands2.ParallelCommandGroup(
                    self.shooter.shoot(
                        self.shooter.desired_ball_speed_sub.get(),
                        self.shooter.desired_flywheel_intake_speed_sub.get()
                    ),
                    self.drivetrain.set_brake_mode().until(
                        lambda: self.controller.getHID().getBButton()
                    )
                ),
                self.shooter.runOnce(lambda: self.shooter.stop())
            )
        )

    def create_commands_auto(self):
        pass

    def create_commands_teleop(self):   
        # Set the forward perspective of the robot for field oriented driving
        self.drivetrain.set_forward_perspective()
        
        # # Estimated button bindings for shooter (subject to change):
        #     - Left trigger: Shoot at calculated speed based on distance to target
        #     - X button: Shoot at speeds specified by network tables
        #     - B button: Stop shooting no matter what mode is being used
        #     - A button: Auto move button (figure out to where later)
        #     - Y button: Cancel auto move command

        # TODO: Add in shooter functions and check if this even works????
     
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
        
        # self.controller.b().onTrue(
        #     self.shooter.runOnce(
        #         lambda: self.shooter.stop_networktable()
        #     )
        # )

        #Button bindings for hopper
        # self.controller.rightBumper().onTrue(
        #     commands2.SequentialCommandGroup(
        #         self.hopper.hop(.25, 1),
        #         commands2.WaitUntilCommand(
        #             lambda: self.controller.getHID().getRightBumperButtonReleased()
        #         ),
        #         self.hopper.hop(0, 0)
        #     )
        # )

        # #Button bindings for intake
        # self.controller.leftBumper().onTrue(
        #     self.intake.runOnce(lambda: self.intake.intake_running())
        #     )

        # self.controller.leftBumper().onFalse(
        #     self.intake.runOnce(lambda: self.intake.intake_stopped())
        #     )
        
        # self.controller.povDown().onTrue(
        #     self.intake.runOnce(lambda: self.intake.arm_down())
        #     )
        
        # self.controller.povDown().onFalse(
        #     self.intake.runOnce(lambda: self.intake.arm_stopped())
        #     )

        # self.controller.povUp().onTrue(
        #     self.intake.runOnce(lambda: self.intake.arm_up())
        #     )

        # self.controller.povUp().onFalse(
        #     self.intake.runOnce(lambda: self.intake.arm_stopped())
        #     )

    def create_commands_test(self):
        # self.shooter.flywheel_motor.get_velocity().set_update_frequency(1000.0)
        # self.shooter.flywheel_motor.get_position().set_update_frequency(1000.0)
        # self.shooter.flywheel_motor.get_motor_voltage().set_update_frequency(1000.0)

        # self.shooter.flywheel_intake_motor.get_velocity().set_update_frequency(1000.0)
        # self.shooter.flywheel_intake_motor.get_position().set_update_frequency(1000.0)
        # self.shooter.flywheel_intake_motor.get_motor_voltage().set_update_frequency(1000.0)

        # # Set the SysId routine to run
        # self.shooter.set_sys_id_routine()

        # Set the SysId routine to run
        self.shooter.set_sys_id_routine()
    
        # # Set button bindings for starting and stopping SignalLogger
        # self.controller.leftBumper().onTrue(commands2.cmd.runOnce(SignalLogger.start))
        # self.controller.rightBumper().onTrue(commands2.cmd.runOnce(SignalLogger.stop))

        # # Set button bindings for performing various parts of SysID routine
        # self.controller.y().whileTrue(self.shooter.sys_id_dynamic(SysIdRoutine.Direction.kForward))
        # self.controller.a().whileTrue(self.shooter.sys_id_dynamic(SysIdRoutine.Direction.kReverse))
        # self.controller.b().whileTrue(self.shooter.sys_id_quasistatic(SysIdRoutine.Direction.kForward))
        # self.controller.x().whileTrue(self.shooter.sys_id_quasistatic(SysIdRoutine.Direction.kReverse))

        self.controller.x().onTrue(
            commands2.SequentialCommandGroup(
                self.shooter.set_voltage(self.shooter.flywheel_motor, 1.0, 5),
                self.shooter.set_voltage(self.shooter.flywheel_motor, 3.0, 5),
                self.shooter.set_voltage(self.shooter.flywheel_motor, 5.0, 5),
                self.shooter.set_voltage(self.shooter.flywheel_motor, 6.0, 5),
                self.shooter.set_voltage(self.shooter.flywheel_motor, 7.0, 5),
                self.shooter.set_voltage(self.shooter.flywheel_motor, 10.0, 5),
                self.shooter.set_voltage(self.shooter.flywheel_motor, 0, 5),
            )
        )

        self.controller.b().onTrue(
            commands2.SequentialCommandGroup(
                self.shooter.set_voltage(self.shooter.flywheel_intake_motor, 1.0, 5),
                self.shooter.set_voltage(self.shooter.flywheel_intake_motor, 3.0, 5),
                self.shooter.set_voltage(self.shooter.flywheel_intake_motor, 5.0, 5),
                self.shooter.set_voltage(self.shooter.flywheel_intake_motor, 6.0, 5),
                self.shooter.set_voltage(self.shooter.flywheel_intake_motor, 7.0, 5),
                self.shooter.set_voltage(self.shooter.flywheel_intake_motor, 10.0, 5),
                self.shooter.set_voltage(self.shooter.flywheel_intake_motor, 0, 5),
            )
        )

    #     self.controller.y().onTrue(
    #         commands2.SequentialCommandGroup(
    #             self.shooter.runOnce(
    #                 lambda: SignalLogger.start()
    #             ),
    #             self.shooter.set_voltage(self.shooter.flywheel_intake_motor, 0.5, 1.0),
    #             self.shooter.set_voltage(self.shooter.flywheel_intake_motor, 1.0, 1.0),
    #             self.shooter.set_voltage(self.shooter.flywheel_intake_motor, 1.5, 1.0),
    #             self.shooter.set_voltage(self.shooter.flywheel_intake_motor, 2.0, 1.0),
    #             self.shooter.set_voltage(self.shooter.flywheel_intake_motor, 3.0, 2.0),
    #             self.shooter.set_voltage(self.shooter.flywheel_intake_motor, 6.0, 2.0),
    #             self.shooter.set_voltage(self.shooter.flywheel_intake_motor, 9.0, 2.0),
    #             self.shooter.set_voltage(self.shooter.flywheel_intake_motor, 12.0, 5.0),
    #             self.shooter.runOnce(
    #                 lambda: SignalLogger.stop()
    #             )
    #         )
    #     )

    #     # # https://v6.docs.ctr-electronics.com/en/latest/docs/api-reference/mechanisms/swerve/swerve-requests.html#swerve-requests-with-direct-control
    #     # self.drive_request = (
    #     #     swerve.requests.RobotCentric()
    #     #     .with_drive_request_type(swerve.SwerveModule.DriveRequestType.VELOCITY)
    #     #     .with_steer_request_type(swerve.SwerveModule.SteerRequestType.POSITION)
    #     #     .with_deadband(self.max_linear_speed * 0.05)
    #     #     .with_rotational_deadband(self.max_angular_rate * 0.05)
    #     #     .with_desaturate_wheel_speeds(True)
    #     # )

    #     # self.controller.y().onTrue(
    #     #     self.drivetrain.runOnce(
    #     #         self.drivetrain.set_control(
    #     #             self.drive_request.with_velocity_x(1)
    #     #         )
    #     #     ).andThen(
    #     #         WaitCommand(0.75)
    #     #     ).andThen(
    #     #         self.drivetrain.runOnce(self.drive_request)
    #     #     )
    #     # )

    #     # self.controller.a().onTrue(
    #     #     self.drivetrain.runOnce(
    #     #         self.drivetrain.set_control(
    #     #             self.drive_request.with_velocity_x(-1)
    #     #         )
    #     #     ).andThen(
    #     #         WaitCommand(0.75)
    #     #     ).andThen(
    #     #         self.drivetrain.runOnce(self.drive_request)
    #     #     )
    #     # )
