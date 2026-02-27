'''
TODO: I am making the hopper and the intake two seperate things so it will make everything easier
'''
import wpilib
import phoenix6
import phoenix6.controls
from phoenix6.hardware import TalonFXS
import phoenix6.configs
import phoenix6.signals
import rev 
import commands2
from commands2 import Subsystem
from commands2.cmd import print_



class Intake(Subsystem): # <-- Telling subsystem that its part of it too

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
        self.talon_motor2 = TalonFXS(CAN_ID)
        self.talon_config2 = phoenix6.configs.talon_fxs_configs.TalonFXSConfiguration()
        

        def intake_running(self):
            self.talon_motor2.set(0.5) # Set to 50% power, can be changed later
            # print("Intake is running!!!")

        def intake_stopped(self):
            self.talon_motor2.set(0) # Stop the motor
            
   
