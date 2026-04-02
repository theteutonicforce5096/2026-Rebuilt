import time
import commands2

from commands2 import Command
from wpilib import DriverStation, PowerDistribution, RobotBase
from phoenix6 import SignalLogger

from robot_container import RobotContainer

class RebuiltRobot(commands2.TimedCommandRobot):
    """
    2026 Robot for Team 5096.
    """
    
    def robotInit(self):
        """
        Initialize the robot container and global logging configuration.
        """
        # Doesn't work because PDH isn't wired to RoboRIO: Enable switchable channel on the REV PDH
        # PowerDistribution().setSwitchableChannel(True)
    
        # Disable Phoenix 6 Auto Signal Logging
        SignalLogger.enable_auto_logging(False)

        # Sleep for 10 seconds only if robot isn't in simulation mode to prevent CANBus motor config errors 
        if RobotBase.isSimulation() == False:
            time.sleep(10) 

        # Create robot container
        self.robot_container = RobotContainer()

        if DriverStation.isFMSAttached():
            SignalLogger.start()
        
    def autonomousInit(self):
        """
        Reset command state and schedule the currently selected autonomous command.
        """
        commands2.CommandScheduler.getInstance().cancelAll()
        self.robot_container.create_commands_auto()
        self.autonomous_command = self.robot_container.get_selected_auto_command()
        if self.autonomous_command is not None:
            self.autonomous_command.schedule()

    def teleopInit(self):
        """
        Cancel autonomous commands and prepare the robot for teleoperated control.
        """
        commands2.CommandScheduler.getInstance().cancelAll()
        self.robot_container.create_commands_teleop()
    
    def teleopExit(self):
        """
        Stop match logging after teleop ends when connected to FMS.
        """
        if DriverStation.isFMSAttached():
            SignalLogger.stop()

    def testInit(self):
        """
        Cancel running commands and configure the robot for test mode.
        """
        commands2.CommandScheduler.getInstance().cancelAll()
        self.robot_container.create_commands_test()
