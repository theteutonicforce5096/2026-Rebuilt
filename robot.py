import commands2
import wpilib

from robot_container import RobotContainer

import cProfile

class RebuiltRobot(commands2.TimedCommandRobot):
    """
    2026 Robot for Team 5096.
    """
    
    def robotInit(self):
        self.container = RobotContainer()
        # init_robot_command = commands2.SequentialCommandGroup(
        #     commands2.WaitCommand(1),
        #     commands2.InstantCommand(lambda: self.container.robot_init())
        # )
        # init_robot_command.schedule()
    
    def robotPeriodic(self):
        commands2.CommandScheduler.getInstance().run()
        
    def autonomousInit(self):
        commands2.CommandScheduler.getInstance().cancelAll()
        self.container.create_commands_auto()

    def autonomousExit(self):
        commands2.CommandScheduler.getInstance().cancelAll()

    def teleopInit(self):
        commands2.CommandScheduler.getInstance().cancelAll()
        self.container.create_commands_teleop()

    def teleopExit(self):
        commands2.CommandScheduler.getInstance().cancelAll()
    
    def testInit(self):
        commands2.CommandScheduler.getInstance().cancelAll()
        self.container.create_commands_test()

    def testExit(self):
        commands2.CommandScheduler.getInstance().cancelAll()