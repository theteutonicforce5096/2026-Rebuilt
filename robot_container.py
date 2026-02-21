import commands2

from constants.swerve_drivetrain_constants import SwerveDrivetrainConstants
from constants.shooter_constants import ShooterConstants

from pathplannerlib.auto import AutoBuilder
class RobotContainer:
    def __init__(self):
        # Create drivetrain subsystem
        self.drivetrain = SwerveDrivetrainConstants.create_drivetrain()

        # Create shooter subsystem
        self.drivetrain = ShooterConstants.create_shooter()

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

    def create_commands_test(self):
        pass