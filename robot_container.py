import commands2

from subsystems.swerve_drivetrain_constants import SwerveDrivetrainConstants

class RobotContainer:
    def __init__(self):
        # Initialize drivetrain subsystem
        self.drivetrain = SwerveDrivetrainConstants.create_drivetrain()

        # Initialize controller
        self.controller = commands2.button.CommandXboxController(0)
        
        # Define max speed variables
        self.max_linear_speed = SwerveDrivetrainConstants.max_linear_speed
        self.max_angular_rate = SwerveDrivetrainConstants.max_angular_rate

        # self._logger = Telemetry(self.max_linear_speed)
        # self.drivetrain.register_telemetry(
        #     lambda state: self._logger.telemeterize(state)
        # )

    def set_commands_auto(self):
        pass

    def set_commands_teleop(self):   
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