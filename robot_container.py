import commands2

from constants.swerve_drivetrain_constants import SwerveDrivetrainConstants
from constants.shooter_constants import ShooterConstants
from constants.physics import calc_velocity, calc_x_dis, shoot_speed, flywheel_intake_speed

from pathplannerlib.auto import AutoBuilder
class RobotContainer:
    def __init__(self):
        # Create drivetrain subsystem
        self.drivetrain = SwerveDrivetrainConstants.create_drivetrain()

        # Create shooter subsystem
        self.shooter = ShooterConstants.create_shooter()

        # Create controller
        self.controller = commands2.button.CommandXboxController(0)
        
        # Define max speed variables
        self.max_linear_speed = SwerveDrivetrainConstants.max_linear_speed
        self.max_angular_rate = SwerveDrivetrainConstants.max_angular_rate

    def create_commands_auto(self):
        pass

    def create_commands_teleop(self):   
        # Set the forward perspective of the robot for field oriented driving
        self.drivetrain.set_forward_perspective()
        
        # Reset slew rate limiters for controlling acceleration
        self.drivetrain.reset_slew_rate_limiters()

        # Set button binding for reseting field centric heading
        (self.controller.leftBumper() & self.controller.rightBumper()).onTrue(
            self.drivetrain.runOnce(lambda: self.drivetrain.seed_field_centric())
        )

        # Set default command for drivetrain
        self.drivetrain.setDefaultCommand(
            self.drivetrain.get_operator_drive_command(
                lambda: self.controller.getLeftTriggerAxis() > 0.05,
                lambda: self.controller.getRightTriggerAxis() > 0.05,
                lambda: self.controller.getLeftY(),
                lambda: self.controller.getLeftX(),
                lambda: self.controller.getRightX()
            ) 
        )

        # # Set button bindings for shooter
        # # TODO: Chng 
        # self.controller.rightTrigger().onTrue(
        #     commands2.ParallelDeadlineGroup(
        #         self.drivetrain.auto_shoot(self.shooter)
        #     ).until(
        #         lambda: self.controller.getHID().getBButton()
        # #     )
        # # )
        
        self.controller.rightTrigger().onTrue(
            self.shooter.runOnce(
                lambda: self.shooter.shoot(
                    shoot_speed(
                        calc_velocity(
                            calc_x_dis(
                                self.drivetrain.get_state().pose.X(), 
                                self.drivetrain.get_state().pose.Y()))), 
                    flywheel_intake_speed()
                ))
        )

        self.controller.leftTrigger().onTrue(
            self.shooter.runOnce(
                lambda: self.shooter.stop())
        )
        
        #Max rps of flywheel (neo vortex) = 113
        #Max rps of flywheel intake (falcon 500) = 106
        #We convert the percents to rps
        self.controller.a().onTrue(
            self.shooter.runOnce(
                lambda: self.shooter.shoot(
                    self.shooter.desired_ball_speed_sub.get() * 113), 
                    self.shooter.desired_flywheel_intake_speed_sub.get() * 106)
            )

    def create_commands_test(self):
        pass