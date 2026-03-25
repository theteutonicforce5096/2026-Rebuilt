import time
import commands2

from commands2 import Command
from wpilib import DriverStation
from phoenix6 import utils, SignalLogger

from robot_container import RobotContainer

class RebuiltRobot(commands2.TimedCommandRobot):
    """
    2026 Robot for Team 5096.
    """
    
    def robotInit(self):
        # Disable Phoenix 6 Auto Signal Logging
        SignalLogger.enable_auto_logging(False)

        # Sleep for 10 seconds only if robot isn't in simulation mode to prevent CANBus motor config errors 
        if not utils.is_simulation():
            time.sleep(10) 

        # Create robot container
        self.robot_container = RobotContainer()
        self.autonomous_command: Command | None = None
        self.addPeriodic(self.robot_container.refresh_auto_preview_if_needed, 1.0)

        if DriverStation.isFMSAttached():
            SignalLogger.start()

    def disabledInit(self):
        commands2.CommandScheduler.getInstance().cancelAll()
        self.autonomous_command = None
        
    def autonomousInit(self):
        if self.autonomous_command is not None:
            self.autonomous_command.cancel()
            self.autonomous_command = None

        self.robot_container.ensure_selected_auto_pose_seeded()
        self.robot_container.create_commands_auto()
        self.autonomous_command = self.robot_container.get_selected_auto_command()
        if self.autonomous_command is not None:
            self.autonomous_command.schedule()

    def teleopInit(self):
        if self.autonomous_command is not None:
            self.autonomous_command.cancel()
            self.autonomous_command = None

        self.robot_container.create_commands_teleop()
    
    def teleopExit(self):
        if DriverStation.isFMSAttached():
            SignalLogger.stop()

    def testInit(self):
        if self.autonomous_command is not None:
            self.autonomous_command.cancel()
            self.autonomous_command = None

        self.robot_container.create_commands_test()
