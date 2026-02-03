'''
TODO: I am making the hopper and the intake two seperate things so it will make everything easier
'''
import wpilib
import phoenix6
import phoenix6.controls
import phoenix6.hardware
import phoenix6.configs
import phoenix6.signals

class Hopper:

    def __init__(self):
        # The device and canbus will have to be changed when I have more information
        self.talon_motor = phoenix6.hardware.TalonFXS(device_id=0,canbus='rio')
        self.talon_config = phoenix6.configs.talon_fxs_configs.TalonFXSConfiguration()
        # I got this okay I think I am going to configure this maybe but alsg
        self.talon_config.future_proof_configs = True
        #TODO: I was in the middle of configuring this you gotta reference talon config each time.
        
    def hopper_on(self):
        self.talon_motor.set(1)
        print("IM MOVING 👅")
    
    def hopper_off(self):
        self.talon_motor.set(0)
        print("I'm not moving 😔")


