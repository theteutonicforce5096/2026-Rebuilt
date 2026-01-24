'''
TODO: At the moment, extinquish puts out EVERY LED command; we might need to troubleshoot later
'''

import commands2
import wpilib
import wpilib.drive
import subsystems.LED_controller as LED_controller
import robot_container

class MyRobot(commands2.TimedCommandRobot):
    def robotInit(self):
        self.led_controller = LED_controller.LED()
        self.pxn_fightstick = wpilib.Joystick(0)
        self.container = robot_container.RobotContainer()
        """
       5096 👅👅
        """
       

    def autonomousInit(self):
        pass
        # commands2.CommandScheduler.getInstance().run()
        """This function is run once each time the robot enters autonomous mode."""
       

    def autonomousPeriodic(self):
        pass
       
       

    def teleopInit(self):
        commands2.button.JoystickButton(self.pxn_fightstick, 1).onTrue(
            commands2.InstantCommand(self.led_controller.auto_in_progress)
        )
        commands2.button.JoystickButton(self.pxn_fightstick, 2).onTrue(
            commands2.InstantCommand(self.led_controller.extinguish)
        )
        commands2.button.JoystickButton(self.pxn_fightstick, 3).onTrue(
            commands2.InstantCommand(self.led_controller.hopper_full)
        )
        commands2.button.JoystickButton(self.pxn_fightstick, 4).onTrue(
            commands2.InstantCommand(self.led_controller.shooting)
        )
        commands2.button.JoystickButton(self.pxn_fightstick, 7).onTrue(
            commands2.InstantCommand(self.led_controller.five_seconds_left)
        )
        # commands2.CommandScheduler.getInstance().cancelAll()
        """This function is called once each time the robot enters teleoperated mode."""
        
    def teleopPeriodic(self):
        pass
        """This function is called periodically during teleoperated mode."""
        
    def testInit(self):
        pass
        # commands2.CommandScheduler.getInstance().cancelAll()
        """This function is called once each time the robot enters test mode."""
        
    def testPeriodic(self):
        pass
        # commands2.CommandScheduler.getInstance().cancelAll()
        """This function is called periodically during test mode."""
        

if __name__ == "__main__":
    wpilib.run(MyRobot)