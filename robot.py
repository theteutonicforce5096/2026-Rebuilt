import commands2
from phoenix6 import SignalLogger
from wpilib import DriverStation

from robot_container import RobotContainer


class RebuiltRobot(commands2.TimedCommandRobot):
    """
    2026 Robot for Team 5096.
    """

    def robotInit(self):
        """
        Initialize the robot container and global logging configuration.
        """
        # Disable Phoenix 6 Auto Signal Logging
        SignalLogger.enable_auto_logging(False)

        # Create robot container. Device configuration waits for each device to enumerate
        # on its bus before applying configs (see configure_device).
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
