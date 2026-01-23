import commands2

from subsystems.LED_controller import LED


from wpimath.geometry import Pose2d, Rotation2d

class RobotContainer:
    def __init__(self):
        # The robot's subsystems and commands are defined here...
        self.led_controller = LED()


        # bro I highkey don't know what to put here
