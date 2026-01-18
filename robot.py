import commands2
import wpilib

from robot_container import RobotContainer

class RebuiltRobot(commands2.TimedCommandRobot):
    """
    2026 Robot for Team 5096.
    """
    
    def robotInit(self):
        self.container = RobotContainer()
    
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

    def teleopExit(self):
        commands2.CommandScheduler.getInstance().cancelAll()
    
    def testInit(self):
        commands2.CommandScheduler.getInstance().cancelAll()

    def testExit(self):
        commands2.CommandScheduler.getInstance().cancelAll()

if __name__ == "__main__":
    wpilib.run(RebuiltRobot)