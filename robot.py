import commands2
import wpilib

from robot_container import RobotContainer
from subsystems import hopper

class RebuiltRobot(commands2.TimedCommandRobot):
    """
    2026 Robot for Team 5096.
    """
    
    def robotInit(self):
        self.container = RobotContainer()
        self.hopper = hopper.Hopper()
        self.pxn_fightstick = wpilib.Joystick(0)
    
    def robotPeriodic(self):
        commands2.CommandScheduler.getInstance().run()
        
    def autonomousInit(self):
        commands2.CommandScheduler.getInstance().cancelAll()
        self.container.configure_button_bindings_auto()

    def autonomousExit(self):
        commands2.CommandScheduler.getInstance().cancelAll()

    def teleopInit(self):
        commands2.CommandScheduler.getInstance().cancelAll()
        self.container.configure_button_bindings_teleop()
        
        commands2.button.JoystickButton(self.pxn_fightstick, 1).onTrue(
            commands2.InstantCommand(self.hopper.hopper_on)
        )
        commands2.button.JoystickButton(self.pxn_fightstick, 2).onTrue(
            commands2.InstantCommand(self.hopper.hopper_off)
        )

    def teleopExit(self):
        commands2.CommandScheduler.getInstance().cancelAll()
    
    def testInit(self):
        commands2.CommandScheduler.getInstance().cancelAll()

    def testExit(self):
        commands2.CommandScheduler.getInstance().cancelAll()

if __name__ == "__main__":
    wpilib.run(RebuiltRobot)