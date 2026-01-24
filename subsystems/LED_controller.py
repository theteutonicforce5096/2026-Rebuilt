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
from phoenix6.controls import EmptyAnimation


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
        self.extinguish()
        self.solid_color_control = controls.SolidColor(
            led_start_index=0,
            led_end_index=83,
            color=rgbw_color.RGBWColor(red=53, green=157, blue=87)) # Naural Blue
        self.candle.set_control(self.solid_color_control)
        

    def hopper_full(self):
        """ Sets LEDS to indicate hopper is full """
        self.extinguish()
        self.animation_control = controls.LarsonAnimation(
            led_start_index=0,
            led_end_index=83,
            slot=0,
            color=rgbw_color.RGBWColor(red=255, green=242, blue=0), # Yellow
            size=10,
            bounce_mode=phoenix6.signals.spn_enums.LarsonBounceValue.FRONT,
            frame_rate=120)
        self.candle.set_control(self.animation_control)
        # ANIMATION SLOT 0 ^^^


    def shooting(self):
        """ Sets LEDS to indicate robot is shooting """
        self.extinguish()
        self.animation_control = controls.FireAnimation(
            led_start_index=0,
            led_end_index=83,
            slot=2,
            brightness=1,
            direction=phoenix6.signals.spn_enums.AnimationDirectionValue.FORWARD,
            sparking= 0.6,
            cooling= 0.3,
            frame_rate= 100 )
        self.candle.set_control(self.animation_control)
        # ANIMATION SLOT 2 ^^^

    def five_seconds_left(self):
        """ Sets LEDS to indicate five seconds left before hub active switches """
        self.extinguish()
        self.animation_control = controls.LarsonAnimation(
            led_start_index=0,
            led_end_index=83,  
            slot=1,
            color=rgbw_color.RGBWColor(red=255, green=0, blue=0), # Red
            size=14,
            bounce_mode= phoenix6.signals.spn_enums.LarsonBounceValue.BACK,
            frame_rate=160)
        self.candle.set_control(self.animation_control)
        # ANIMATION SLOT 1 ^^^

    def extinguish(self):
        """ Turns off LEDS """
        self.solid_color_control = controls.SolidColor(
            led_start_index=0,
            led_end_index=83,
            color=rgbw_color.RGBWColor(red=0, green=0, blue=0))
        self.animation_control = controls.EmptyAnimation(0)
        self.candle.set_control(self.animation_control)
        self.animation_control = controls.EmptyAnimation(1)
        self.candle.set_control(self.animation_control)
        self.animation_control = controls.EmptyAnimation(2)
        self.candle.set_control(self.animation_control)
        self.candle.set_control(self.solid_color_control)
        # ANIMATION SLOT 7
        # OFF ^^^

    
        
# # I miss you phoenix 5