import phoenix6
from phoenix6.hardware import CANdle
from phoenix6.signals import rgbw_color
import phoenix6.controls as controls
from commands2 import Subsystem

from subsystems.device_config import configure_device


class LED(Subsystem):
    """
    Class for controlling LEDs.
    """

    def __init__(self, canbus: phoenix6.CANBus, led_id: int, led_configs: phoenix6.configs.CANdleConfiguration, num_config_attempts: int, led_end_index: int):
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
        :param led_end_index: Number of LEDs on the strip
        :type led_end_index: int
        """

        Subsystem.__init__(self)

        # Create LED
        self.candle = CANdle(led_id, canbus)

        # Apply LED configs
        configure_device(self.candle, led_configs, num_config_attempts)

        # Define LED end index
        self.led_end_index = led_end_index

    def auto_in_progress(self):
        """
        Set LEDs to a solid color to indicate autonomous is running.
        """
        self.animation_control = controls.SolidColor(0, self.led_end_index, rgbw_color.RGBWColor(53, 147, 87, 0))
        self.candle.set_control(self.animation_control)

    def hopper_full(self):
        """
        Play an animation to indicate the hopper is full.
        """
        self.extinguish()
        self.animation_control = controls.LarsonAnimation(0, self.led_end_index, 0, rgbw_color.RGBWColor(225, 242, 0, 0))
        self.candle.set_control(self.animation_control)

    def shooting_manual(self):
        """
        Play an animation to indicate a manual shot is in progress.
        """
        self.extinguish()
        self.animation_control = controls.FireAnimation(0, self.led_end_index, 1, 1, phoenix6.signals.spn_enums.AnimationDirectionValue.FORWARD, 0.6, 0.3, 60)
        self.candle.set_control(self.animation_control)

    def shooting_calculated(self):
        """
        Play an animation to indicate a calculated shot is in progress.
        """
        self.extinguish()
        self.animation_control = controls.LarsonAnimation(0, self.led_end_index, 0, rgbw_color.RGBWColor(40, 60, 255, 0))
        self.candle.set_control(self.animation_control)

    def default(self):
        """
        Play the default idle animation.
        """
        self.extinguish()
        self.animation_control = controls.StrobeAnimation(0, self.led_end_index, 4, rgbw_color.RGBWColor(102, 225, 0, 0), 4)
        self.candle.set_control(self.animation_control)

    def pride(self):
        """
        Play a rainbow animation.
        """
        self.extinguish()
        self.animation_control = controls.RainbowAnimation(0, self.led_end_index, 3, 1, phoenix6.signals.spn_enums.AnimationDirectionValue.FORWARD, 100)
        self.candle.set_control(self.animation_control)

    def five_seconds_left(self):
        """
        Play an animation to indicate five seconds are left in the match.
        """
        self.extinguish()
        self.animation_control = controls.LarsonAnimation(0, self.led_end_index, 2, rgbw_color.RGBWColor(225, 0, 0, 0), 3, phoenix6.signals.spn_enums.LarsonBounceValue.FRONT, 25)
        self.candle.set_control(self.animation_control)

    # Number of CANdle animation slots cleared when turning the strip off
    _NUM_ANIMATION_SLOTS = 5

    def extinguish(self):
        """
        Turn off all LED animations.
        """
        for animation_slot in range(self._NUM_ANIMATION_SLOTS):
            self.animation_control = controls.EmptyAnimation(animation_slot)
            self.candle.set_control(self.animation_control)
