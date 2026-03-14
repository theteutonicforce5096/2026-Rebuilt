'''
TODO: I am making the hopper and the intake two seperate things so it will make everything easier
'''
import wpilib
import phoenix6
import phoenix6.controls
from phoenix6.hardware import TalonFX
import phoenix6.configs
import phoenix6.signals
import rev 
import commands2
from commands2 import Subsystem
from commands2.cmd import print_

""" 
TODO: 
- It would be beneficial if we had an order to when which motors turn on so nothing gets stuck
1. Shooter Intake
2. Mechanim wheels
- We need two motors, both motors are talon FX, not sure there would need ot be very much tuning
"""

class Hopper(Subsystem): # <-- Telling subsystem that its part of it too

    def __init__(self):

        Subsystem.__init__(self)

        # Jay is it okay if we make the canbus an object 
        # CONSTANTS
        CANBUS = phoenix6.CANBus.roborio()
        
        # The mechanim wheels
        self.mechanim_wheel = TalonFX(50, CANBUS)
        self.mechanim_config = phoenix6.configs.TalonFXConfiguration()
        self.mechanim_wheel.configurator.apply(self.mechanim_config)

        # Wheels in the hopper
        self.agitator_wheel = TalonFX(50, CANBUS)
        self.agitator_config = phoenix6.configs.TalonFXConfiguration()
        self.agitator_wheel.configurator.apply(self.agitator_config)
        

        
    def mechanim_on(self, speed):
        self.mechanim_wheel.set(speed)
        print("mechanim wheels are moving")
    
    def mechanim_off(self, speed):
        self.mechanim_wheel.set(speed)
        print("mechanim wheels are not moving")

    def agitator_on(self, speed):
        self.agitator_wheel.set(speed)
        print("agitator moving")
    
    def agitator_off(self, speed):
        self.agitator_wheel.set(speed)
        print("agitator is stopped")



