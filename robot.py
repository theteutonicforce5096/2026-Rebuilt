# TODO: insert robot code here
#!/usr/bin/env python3
#
# Copyright (c) FIRST and other WPILib contributors.
# Open Source Software; you can modify and/or share it under the terms of
# the WPILib BSD license file in the root directory of this project.


#TODO: CONVERT TO COMMAND BASED AND CHANGE ANIMATIONS 

import commands2
import wpilib
import wpilib.drive
import LED_controller

class MyRobot(commands2.TimedCommandRobot):
    def robotInit(self):
        self.led_controller = LED_controller.LED()
        self.pxn_fightstick = wpilib.Joystick(0)
        
        """
       5096 👅👅
        """
       

    def autonomousInit(self):
        pass
        """This function is run once each time the robot enters autonomous mode."""
       

    def autonomousPeriodic(self):
        pass
        """This function is called periodically during autonomous."""

        # Drive for two seconds
       

    def teleopInit(self):
        pass
        """This function is called once each time the robot enters teleoperated mode."""
        pass
    def teleopPeriodic(self):
        if self.pxn_fightstick.getRawButtonPressed(1):
            self.led_controller.auto_in_progress()
        """This function is called periodically during teleoperated mode."""
        
    def testInit(self):
        pass
        """This function is called once each time the robot enters test mode."""
        
    def testPeriodic(self):
        pass
        """This function is called periodically during test mode."""
        

if __name__ == "__main__":
    wpilib.run(MyRobot)