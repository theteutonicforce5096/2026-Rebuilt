import commands2

from subsystems.swerve_drive_constants import SwerveDriveConstants

from pathplannerlib.auto import AutoBuilder, PathConstraints
from pathplannerlib.path import PathPlannerPath, IdealStartingState, GoalEndState
from pathplannerlib.auto import PathPlannerAuto

from wpimath.geometry import Pose2d, Rotation2d

class RobotContainer:
    def __init__(self):
        # Initialize drivetrain subsystem
        self.drivetrain = SwerveDriveConstants.create_drivetrain()

        # Initialize controller
        self.controller = commands2.button.CommandXboxController(0)
        
        # Define max speed variables
        self.max_linear_speed = SwerveDriveConstants.max_linear_speed
        self.max_angular_rate = SwerveDriveConstants.max_angular_rate
    
    def configure_button_bindings_auto(self):
        # Starting position is 2 meters away from tag 18 to back bumper facing tag 18.
        # Pathplanner should do this itself automatically.
        # self.drivetrain.reset_pose(Pose2d(3.6576 - 2 - 0.416, 4.0259, Rotation2d.fromDegrees(180)))

        auto = PathPlannerAuto('Simple Auto').until(
            lambda: self.controller.getHID().getYButtonPressed()
        )
        auto.schedule()

    def configure_button_bindings_teleop(self):   
        # Set the forward perspective of the robot for field oriented driving
        self.drivetrain.set_forward_perspective()
        
        # Reset slew rate limiters for controlling acceleration
        self.drivetrain.reset_slew_rate_limiters()

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
        
        # Set button binding for reseting field centric heading
        (self.controller.leftBumper() & self.controller.rightBumper()).onTrue(
            self.drivetrain.runOnce(lambda: self.drivetrain.seed_field_centric())
        )

        reef_waypoints = PathPlannerPath.waypointsFromPoses(
            [
                Pose2d(3.6576 - 0.416 - 1, 4.0259, Rotation2d.fromDegrees(0)),
                Pose2d(3.6576 - 0.416, 4.0259, Rotation2d.fromDegrees(0))
            ]
        )

        reef_path = PathPlannerPath(
            reef_waypoints,
            PathConstraints(2.5, 1.25, 7.5, 3.75),
            IdealStartingState(0.0, Rotation2d.fromDegrees(0)),
            GoalEndState(0.0, Rotation2d.fromDegrees(0))
        )
        reef_path.preventFlipping = True

        self.controller.a().onTrue(
            AutoBuilder.pathfindThenFollowPath(
                reef_path,
                PathConstraints(2.5, 1.25, 7.5, 3.75)
            ).until(
                lambda: self.controller.getHID().getYButtonPressed()
            )
        )