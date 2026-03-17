import time
import commands2

from wpilib import DriverStation, RobotBase
from phoenix6 import SignalLogger

from robot_container import RobotContainer

class RebuiltRobot(commands2.TimedCommandRobot):
    """
    2026 Robot for Team 5096.
    """
    
    def robotInit(self):
        # Disable Phoenix 6 Auto Signal Logging
        SignalLogger.enable_auto_logging(False)

        # Sleep for 10 seconds only if robot isn't in simulation mode to prevent CANBus motor config errors 
        if RobotBase.isReal():
            time.sleep(10) 

        # Create robot container
        self.robot_container = RobotContainer()

        if DriverStation.isFMSAttached():
            SignalLogger.start()

    def disabledInit(self):
        commands2.CommandScheduler.getInstance().cancelAll()
        
    def autonomousInit(self):
        self.robot_container.create_commands_auto()

    def teleopInit(self):
        self.robot_container.create_commands_teleop()
    
    def teleopExit(self):
        if DriverStation.isFMSAttached():
            SignalLogger.stop()

    def testInit(self):
        self.robot_container.create_commands_test()