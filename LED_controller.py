import wpilib
import commands2
import phoenix6
from phoenix6 import hardware
from phoenix6.configs.candle_configs import LEDConfigs, CANdleConfiguration
from phoenix6.hardware import CANdle
from phoenix6.signals import rgbw_color
import phoenix6.controls as controls


class LED():
    """ 
Controls LEDS 👅👅
    """
    def __init__(self):
# Hardware Init
        self.candle = CANdle(3)
        self.rgb = phoenix6.signals
        
    def auto_in_progress(self):
        """ Sets LEDS to indicate autonomous is in progress """
        self.solid_color_control = controls.SolidColor(0,83,rgbw_color.RGBWColor(red=53, green=157, blue=87))
        self.candle.set_control(self.solid_color_control)
        # Naural Blue

    def hopper_full(self):
        """ Sets LEDS to indicate hopper is full """
        self.solid_color_control = controls.SolidColor(0,83,rgbw_color.RGBWColor(red=127, green=255, blue=0))
        self.candle.set_control(self.solid_color_control)
        
#         self.candle.setLEDs(127, 255, 0)  # Chartreuse 

    def shooting(self):
        """ Sets LEDS to indicate robot is shooting """
        self.solid_color_control = controls.SolidColor(0,83,rgbw_color.RGBWColor(red=184, green=15, blue=10))
        self.candle.set_control(self.solid_color_control)

    def five_seconds_left(self):
        """ Sets LEDS to indicate five seconds left before hub active switches """
        self.solid_color_control = controls.SolidColor(0,83,rgbw_color.RGBWColor(red=255, green=140, blue=0))
        self.candle.set_control(self.solid_color_control)
        # Orange
    #TODO: HOPPER FULL NEEDS TO BE FLASHING
    # FIVE SECONDS LEFT NEEDS TO BE FLASHING
        
# # I miss you phoenix 5