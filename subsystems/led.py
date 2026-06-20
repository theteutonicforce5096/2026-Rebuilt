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

    def __init__(self, canbus: phoenix6.CANBus, led_id: int, led_configs: phoenix6.configs.CANdleConfiguration, strip_type: phoenix6.configs.candle_configs.LEDConfigs.with_strip_type, num_config_attempts: int):
        """
        Constructor for initializing LEDs using the specified constants.

        :param canbus: CANBus instance that electronics are on
        :type canbus: phoenix6.CANBus
        :param led_id: CAN ID of the LED controller
        :type led_id: int
        :param led_configs: Configs for the LED controller
        :type led_configs: phoenix6.configs.CANdleConfiguration
        :param strip_type: Type of LED strip
        :type strip_type: phoenix6.configs.candle_configs.LEDConfigs.with_strip_type
        :param num_config_attempts: Number of times to attempt to configure the device
        :type num_config_attempts: int
        """

        Subsystem.__init__(self) 
        
        # Create LED
        self.candle = CANdle(led_id, canbus)
        
        # METHODS
        
    def auto_in_progress(self):
        # self.extinguish()
        print("I genuiely cannot do ts anymore")
        self.solid_color_control = controls.SolidColor(
            led_start_index= 0,
            color = rgbw_color.RGBWColor(red = 53, green = 157, blue = 87)) # Natural Blue
        self.candle.set_control(self.solid_color_control)

    def hopper_full(self):
        self.extinguish()
        self.animation_control = controls.LarsonAnimation(
            led_start_index = 0,
            slot = 0,
            color = rgbw_color.RGBWColor(red = 255, green = 242, blue = 0), # Yellow
            size = 10,
            bounce_mode = phoenix6.signals.spn_enums.LarsonBounceValue.FRONT,
            frame_rate = 120
        )
        self.candle.set_control(self.animation_control)

    def shooting (self):
        self.extinguish()
        self.animation_control = controls.FireAnimation(
            led_start_index = 0,
            slot = 2, 
            brightness = 1, 
            direction = phoenix6.signals.spn_enums.AnimationDirectionValue.FORWARD,
            sparking = 0.6,
            cooling = 0.3, 
            frame_rate = 100
        )
        self.candle.set_control(self.animation_control)

    def five_seconds_left(self):
        self.extinguish()
        self.animation_control = controls.LarsonAnimation(
            led_start_index = 0,
            slot = 1,
            color = rgbw_color.RGBWColor(red = 255, green = 0, blue = 0), # Red
            size = 10,
            bounce_mode = phoenix6.signals.spn_enums.LarsonBounceValue.FRONT,
            frame_rate = 120
        )
        self.candle.set_control(self.animation_control)
    
    def extinguish(self):
        self.solid_color_control = controls.SolidColor(
            led_start_index = 0,
            color = rgbw_color.RGBWColor(red = 0, green = 0, blue = 0))
        self.animation_control = controls.EmptyAnimation(0)
        self.candle.set_control(self.animation_control)
        self.animation_control = controls.EmptyAnimation(1)
        self.candle.set_control(self.animation_control)
        self.animation_control = controls.EmptyAnimation(2)
        self.candle.set_control(self.animation_control)
        