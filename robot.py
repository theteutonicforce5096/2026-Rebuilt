# TODO: insert robot code here
#!/usr/bin/env python3
#
# Copyright (c) FIRST and other WPILib contributors.
# Open Source Software; you can modify and/or share it under the terms of
# the WPILib BSD license file in the root directory of this project.
#
import commands2
import wpilib
import wpilib.drive
import LED_controller

class MyRobot(wpilib.TimedRobot):
    def robotInit(self):
        self.led_controller = LED_controller.LED()
        
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