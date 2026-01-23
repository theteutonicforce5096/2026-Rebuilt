"""
TODO: I jst put 83 as the default number
of LEDS; change later
"""
import wpilib
import commands2
import phoenix6
from phoenix6 import hardware
from phoenix6.configs.candle_configs import LEDConfigs, CANdleConfiguration
from phoenix6.hardware import CANdle
from phoenix6.signals import rgbw_color
import phoenix6.controls as controls
from phoenix6.controls import LarsonAnimation
from phoenix6.controls import FireAnimation


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
        self.animation_control = controls.LarsonAnimation(
            0, 83, 0, rgbw_color.RGBWColor(red=255, green=242, blue=0), size=5, bounce_mode=True, frame_rate=14)
        self.candle.set_control(self.animation_control)
        # ANIMATION SLOT 0 ^^^


    def shooting(self):
        """ Sets LEDS to indicate robot is shooting """
        self.animation_control = controls.FireAnimation(
            0, 83, 2, 1, phoenix6.signals.spn_enums.AnimationDirectionValue.FORWARD, sparking= 0.6, cooling= 0.3, frame_rate= 60 )
        self.candle.set_control(self.animation_control)
        # ANIMATION SLOT 2 ^^^

    def five_seconds_left(self):
        """ Sets LEDS to indicate five seconds left before hub active switches """
        self.animation_control = controls.LarsonAnimation(
            0, 83, 1, rgbw_color.RGBWColor(red=255, green=0, blue=0), size=10, bounce_mode=True, frame_rate=14)
        self.candle.set_control(self.animation_control)
       # ANIMATION SLOT 1 ^^^

    
        
# # I miss you phoenix 5