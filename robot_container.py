import commands2
from commands2 import WaitCommand

from phoenix6 import swerve

from constants.swerve_drivetrain_constants import SwerveDrivetrainConstants
from constants.shooter_constants import ShooterConstants
from constants.physics import calc_velocity, calc_x_dis, shoot_speed, flywheel_intake_speed

class RobotContainer:
    def __init__(self):
        # Create controller
        self.controller = commands2.button.CommandXboxController(0)
        
        # Define max speed variables
        self.max_linear_speed = SwerveDrivetrainConstants._max_linear_speed
        self.max_angular_rate = SwerveDrivetrainConstants._max_angular_rate

        # Create drivetrain subsystem
        self.drivetrain = SwerveDrivetrainConstants.create_drivetrain()
                     
        # Create shooter subsystem
        self.shooter = ShooterConstants.create_shooter()

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
        # # TODO: Make parallel deadline group for brake mode, etc.
        # self.controller.rightTrigger().onTrue(
        #     commands2.ParallelDeadlineGroup(
        #         self.drivetrain.auto_shoot(self.shooter)
        #     ).until(
        #         lambda: self.controller.getHID().getBButton()
        # #     )
        # # )
        
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

        # self.controller.leftTrigger().onTrue(
        #     self.shooter.runOnce(
        #         lambda: self.shooter.stop())
        # )
        
        #Max rps of flywheel (neo vortex) = 113
        #Max rps of flywheel intake (falcon 500) = 106
        #We convert the percents to rps
        self.controller.x().onTrue(
            self.shooter.runOnce(
                lambda: self.shooter.shoot(
                    # 0.5 * 113, 
                    # 0.25 * 106)
                    self.shooter.desired_ball_speed_sub.get() * 113, 
                    self.shooter.desired_flywheel_intake_speed_sub.get() * 106)
                )
            )
        
        self.controller.b().onTrue(
            self.shooter.runOnce(
                lambda: self.shooter.stop_networktable()
            )
        )

    def create_commands_test(self):
        self.drive_request = (
            swerve.requests.RobotCentric()
            .with_drive_request_type(swerve.SwerveModule.DriveRequestType.VELOCITY)
            .with_steer_request_type(swerve.SwerveModule.SteerRequestType.POSITION)
            .with_deadband(self.max_linear_speed * 0.05)
            .with_rotational_deadband(self.max_angular_rate * 0.05)
            .with_desaturate_wheel_speeds(True)
        )

        self.controller.y().onTrue(
            self.drivetrain.runOnce(
                self.drivetrain.set_control(
                    self.drive_request.with_velocity_x(1)
                )
            ).andThen(
                WaitCommand(0.75)
            ).andThen(
                self.drivetrain.runOnce(self.drive_request)
            )
        )

        self.controller.a().onTrue(
            self.drivetrain.runOnce(
                self.drivetrain.set_control(
                    self.drive_request.with_velocity_x(-1)
                )
            ).andThen(
                WaitCommand(0.75)
            ).andThen(
                self.drivetrain.runOnce(self.drive_request)
            )
        )

