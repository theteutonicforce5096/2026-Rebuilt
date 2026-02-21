'''
TODO: I am making the hopper and the intake two seperate things so it will make everything easier
'''
import wpilib
import phoenix6
import phoenix6.controls
import phoenix6.hardware
import phoenix6.configs
import phoenix6.signals
import rev 
import commands2
from commands2 import Subsystem
from commands2.cmd import print_



class Hopper(commands2.Subsystem): # <-- Telling subsystem that its part of it too

    def __init__(self, CAN_ID):
        # Note to Riley:
        # The reason we need Subsystem.__init__ is because 
        # it needs to know the parent class's setup code and 
        # that commands 2 knows that Hopper exists
        Subsystem.__init__(self) 
    
        # # Idek if we need spark max I'm going to be completely honest like genuinely
        # self.Neo550_1 = rev.SparkMax.MotorType.kBrushless
        # self.Neo_configs = rev.SparkMax.configure()
        # self.Neo550_1(self.Neo_configs)

        # The device and canbus will have to be changed when I have more information
        self.talon_motor = phoenix6.hardware.TalonFXS(device_id=0,canbus='rio')
        self.talon_config = phoenix6.configs.talon_fxs_configs.TalonFXSConfiguration()
        

        
    def hopper_on(self):
        self.talon_motor.set(1)
        print("IM MOVING 👅")
    
    def hopper_off(self):
        self.talon_motor.set(0)
        print("I'm not moving 😔")


