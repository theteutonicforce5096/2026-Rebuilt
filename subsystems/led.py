import wpilib
import commands2
import phoenix6
from phoenix6 import hardware
from phoenix6.hardware import CANdle
from phoenix6.signals import rgbw_color
import phoenix6.controls as controls
from commands2 import Subsystem


class LED(Subsystem):
    """
    Class for controlling LEDs.
    """

    def __init__(self, canbus: phoenix6.CANBus, led_id: int, led_configs: phoenix6.configs.CANdleConfiguration, num_config_attempts: int):
        """
        Constructor for initializing LEDs using the specified constants.

        :param canbus: CANBus instance that electronics are on
        :type canbus: phoenix6.CANBus
        :param led_id: CAN ID of the LED controller
        :type led_id: int
        :param led_configs: Configs for the LED controller
        :type led_configs: phoenix6.configs.CANdleConfiguration
        :param num_config_attempts: Number of times to attempt to configure the device
        :type num_config_attempts: int
        """

        Subsystem.__init__(self) 
        
        # Create LED
        self.candle = CANdle(led_id, canbus)
        
        # LED end index because I don't want to keep changing it cause ts is annoying
        self.led_end_index = 12

        # METHODS
        
    def auto_in_progress(self):
        # self.extinguish()
        print("I genuiely cannot do ts anymore")
        self.solid_color_control = controls.SolidColor(0, self.led_end_index, rgbw_color.RGBWColor(53, 147, 87, 0)) # Natural Blue
        self.candle.set_control(self.solid_color_control)

    def hopper_full(self): # 225, 242, 0
        self.extinguish()
        self.animation_control = controls.LarsonAnimation(0, self.led_end_index, 0, rgbw_color.RGBWColor(225, 242, 0, 0))
        self.candle.set_control(self.animation_control)

    def shooting (self):
        self.extinguish()
        self.animation_control = controls.FireAnimation(0, self.led_end_index, 1, 1, phoenix6.signals.spn_enums.AnimationDirectionValue.FORWARD, 0.6, 0.3, 60)
        self.candle.set_control(self.animation_control)

    def five_seconds_left(self):
        self.extinguish()
        self.animation_control = controls.LarsonAnimation(0, self.led_end_index, 2, rgbw_color.RGBWColor(225, 0, 0, 0), 3, phoenix6.signals.spn_enums.LarsonBounceValue.FRONT, 25)
        self.candle.set_control(self.animation_control)
    
    def extinguish(self):
        self.solid_color_control = controls.SolidColor(0, self.led_end_index, rgbw_color.RGBWColor(0,0,0,0))
        self.animation_control = controls.EmptyAnimation(0)
        self.candle.set_control(self.animation_control)
        self.animation_control = controls.EmptyAnimation(1)
        self.candle.set_control(self.animation_control)
        self.animation_control = controls.EmptyAnimation(2)
        self.candle.set_control(self.animation_control)
        