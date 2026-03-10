import commands2
import wpilib
from wpilib import DriverStation
from subsystems.LED_controller import LED
from pathplannerlib.auto import AutoBuilder, PathConstraints
from pathplannerlib.path import PathPlannerPath, IdealStartingState, GoalEndState
from pathplannerlib.auto import PathPlannerAuto

from wpimath.geometry import Pose2d, Rotation2d

"""
5096 👅👅
"""

class RobotContainer:
    def __init__(self):

        # Initialize LEDs 

        self.led = LED()
        self.last_time = None
        # ^^^ I need this for LEDs

        # # Initialize drivetrain subsystem
        # self.drivetrain = SwerveDriveConstants.create_drivetrain()

        # # Initialize controller
        self.controller = commands2.button.CommandXboxController(0)
        
        # # Define max speed variables
        # self.max_linear_speed = SwerveDriveConstants.max_linear_speed
        # self.max_angular_rate = SwerveDriveConstants.max_angular_rate
    ''' THIS DOES NOT MATTER IN TERMS OF LEDs'''

    
    def robotperiodic(self):
        if DriverStation.isAutonomousEnabled():
            self.led.auto_in_progress()

        self.MATCHTIME = int(DriverStation.getMatchTime())

        # I need this so the LEDs won't constantly be trying to change to the same animation
        if self.MATCHTIME != self.last_time:
            self.last_time = self.MATCHTIME
            if DriverStation.isTeleopEnabled():
                # The numbers are 5 seconds before each phase switch
                if self.MATCHTIME in (115, 90, 65, 40, 25): 
                    self.led.five_seconds_left()
        
    def configure_button_bindings_auto(self):
        pass
    
        ''' THIS DOES NOT MATTER IN TERMS OF LEDS'''
        # # Starting position is 2 meters away from tag 18 to back bumper facing tag 18.
        # # Pathplanner should do this itself automatically.
        # # self.drivetrain.reset_pose(Pose2d(3.6576 - 2 - 0.416, 4.0259, Rotation2d.fromDegrees(180)))

        # auto = PathPlannerAuto('Simple Auto').until(
        #     lambda: self.controller.getHID().getYButtonPressed()
        # )
        # auto.schedule()
        '''THIS DOES NOT MATTER IN TERMS OF LEDS'''

    def configure_button_bindings_teleop(self):
        
        pass

        
        

      
    
    ''' THIS DOES NOT MATTER IN TERMS OF LEDS'''
        # # Set the forward perspective of the robot for field oriented driving
        # self.drivetrain.set_forward_perspective()

        # # Reset slew rate limiters for controlling acceleration
        # self.drivetrain.reset_slew_rate_limiters()

        # # Set default command for drivetrain
        # self.drivetrain.setDefaultCommand(
        #     self.drivetrain.get_operator_drive_command(
        #         lambda: self.controller.getLeftTriggerAxis() > 0.05,
        #         lambda: self.controller.getRightTriggerAxis() > 0.05,
        #         lambda: self.controller.getLeftY(),
        #         lambda: self.controller.getLeftX(),
        #         lambda: self.controller.getRightX()
        #     ) 
        # )
    ''' THIS DOES NOT MATTER IN TERMS OF LEDS'''
        


    ''' THIS DOES NOT MATTER IN TERMS OF LEDS '''
        # # Set button binding for reseting field centric heading
        # (self.controller.leftBumper() & self.controller.rightBumper()).onTrue(
        #     self.drivetrain.runOnce(lambda: self.drivetrain.seed_field_centric())
        # )

        # reef_waypoints = PathPlannerPath.waypointsFromPoses(
        #     [
        #         Pose2d(3.6576 - 0.416 - 1, 4.0259, Rotation2d.fromDegrees(0)),
        #         Pose2d(3.6576 - 0.416, 4.0259, Rotation2d.fromDegrees(0))
        #     ]
        # )

        # reef_path = PathPlannerPath(
        #     reef_waypoints,
        #     PathConstraints(2.5, 1.25, 7.5, 3.75),
        #     IdealStartingState(0.0, Rotation2d.fromDegrees(0)),
        #     GoalEndState(0.0, Rotation2d.fromDegrees(0))
        # )
        # reef_path.preventFlipping = True

        # self.controller.a().onTrue(
        #     AutoBuilder.pathfindThenFollowPath(
        #         reef_path,
        #         PathConstraints(2.5, 1.25, 7.5, 3.75)
        #     ).until(
        #         lambda: self.controller.getHID().getYButtonPressed()
        #     )
        # ),
    ''' THIS DOES NOT MATTER IN TERMS OF LEDS'''