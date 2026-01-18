import wpilib
import commands2
import phoenix6
from phoenix6 import hardware
from phoenix6.configs.candle_configs import LEDConfigs, CANdleConfiguration
from phoenix6.hardware import Controls


class LED():
    """ 
Controls LEDS 👅👅
    """
    def __init__(self):
# Hardware Init
        self.candle = hardware.CANdle(1)
        self.led_configs = LEDConfigs( 
            strip_type = "RGB", 
            brightness_scalar = 1.0, 
            loss_of_signal_behavior = "KEEP" 
            )
            self.candle_config = CANdleConfiguration().with_led(self.led_configs)

        # Loss of signal behavior = keep means if candle stops getting Led frame updates, it will hold the last output
        # literally how it was before in phoenix 5
        # omg phoenix 5 is so much better I freaking hate this library
    def auto_in_progress(self):
        """ Sets LEDS to indicate driver has no control over robot """
        self.candle.setLEDs(73, 157, 208)  # Naural Blue

    def hopper_full(self):
        """ Sets LEDS to indicate hopper is full """
        self.candle.setLEDs(127, 255, 0)  # Chartreuse 

    def shooting(self):
        """ Sets LEDS to indicate robot is shooting """
        self.candle.setLEDs(184, 15, 10)  # Crimson

    def five_seconds_left(self):
        """ Sets LEDS to indicate five seconds left before hub active switches """
        self.candle.setLEDs(255, 140, 0)  # Dark Orange
    
"""    #TODO: FIRST TEST SINGLE COLORS, THEN TEST ANIMATIONS LATER 
if we can get that stupid CANdle error fixed sl;dfjfsdlk """
    
        
# I miss you phoenix 5