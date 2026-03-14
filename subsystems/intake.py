'''
TODO: I am making the hopper and the intake two seperate things so it will make everything easier
'''
import wpilib
import phoenix6
import phoenix6.controls
from phoenix6.hardware import TalonFXS
from phoenix6.hardware import TalonFX
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
        # that commands 2 knows that it exists
        Subsystem.__init__(self) 

        # Objects that we need
        CANBUS = phoenix6.CANBus.roborio()
        MOTOROUTPUT = phoenix6.configs.config_groups.MotorOutputConfigs()
        INVERTED = phoenix6.signals.spn_enums.InvertedValue(1)
        MOTOROUTPUT.inverted = INVERTED
        
        # CANID is set to 40
        self.intake_wheel = TalonFXS(40, CANBUS)
        self.wheel_config = phoenix6.configs.TalonFXSConfiguration()
        self.wheel_config.commutation.motor_arrangement = phoenix6.signals.MotorArrangementValue(phoenix6.signals.MotorArrangementValue.NEO550_JST)
        self.wheel_config.motor_output = MOTOROUTPUT
        # Store the result of the cofig apply to variale
        self.intake_wheel.configurator.apply(self.wheel_config)
        
        # We need PID for arm 
        self.intake_arm = TalonFX(41, CANBUS)
        self.arm_config = phoenix6.configs.TalonFXConfiguration()
        self.intake_arm.configurator.apply(self.arm_config)

    def intake_running(self):
        self.intake_wheel.set(0.5) # Set to 50% power, can be changed later
            # print("Intake is running!!!")
        print("intake wheels are running")

    def intake_stopped(self):
        self.intake_wheel.set(0) # Stop the motor
        print("intake wheels are stopped")

    def arm_down(self):
        self.intake_arm.set(.5)
        print("arm is going to the floor")
    
    def arm_up(self):
        self.intake_arm.set(-.5)
        print("arm is going up")
    
    def arm_stopped(self):
        self.intake_arm.set(0)
        print("arm is stopped")

    
            

