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

    def __init__(self):
        # Note to Riley:
        # The reason we need Subsystem.__init__ is because 
        # it needs to know the parent class's setup code and 
        # that commands 2 knows that Hopper exists
        Subsystem.__init__(self) 

        # Objects that we need
        CANBUS = phoenix6.CANBus.roborio()
        MOTOROUTPUT = phoenix6.configs.config_groups.MotorOutputConfigs()
        INVERTED = phoenix6.signals.spn_enums.InvertedValue(1)
        MOTOROUTPUT.inverted = INVERTED
        
        # CANID is set to 40
        self.talon_motor2 = TalonFXS(40, CANBUS)
        
        self.talon_config2 = phoenix6.configs.TalonFXSConfiguration()
        self.talon_config2.commutation.motor_arrangement = phoenix6.signals.MotorArrangementValue(phoenix6.signals.MotorArrangementValue.NEO550_JST)
        self.talon_config2.motor_output = MOTOROUTPUT
        # Store the result of the cofig apply to variale
        self.talon_motor2.configurator.apply(self.talon_config2)
        

    def intake_running(self):
        self.talon_motor2.set(0.5) # Set to 50% power, can be changed later
            # print("Intake is running!!!")
        print("Sanity Check Run")

    def intake_stopped(self):
        self.talon_motor2.set(0) # Stop the motor
        print("Sanity Check Stop")
            

