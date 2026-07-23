from phoenix6 import CANBus
from phoenix6.configs import CANdleConfiguration
from phoenix6.signals import StripTypeValue

from subsystems.led import LED


class LEDConstants:
    """
    Constants for LED Subsystem
    """

    # CANBus instance - Kellen wired it so that it would start from the roborio
    _canbus = CANBus("rio")

    # CAN IDs
    _led_id = 3

    # Number of times to attempt to configure each device
    _num_config_attempts = 3

    # LED Configs (CANdle - TalonFXS)
    _led_configs = CANdleConfiguration()
    _led_configs.led.with_strip_type(StripTypeValue.GRB)

    # Number of LEDs on the strip
    _led_end_index = 64

    @classmethod
    def create_led(cls) -> LED:
        """
        Create an LED subsystem instance using the configured constant values.

        :param cls: LEDConstants class used as the source of the subsystem constants.
        :type cls: type[LEDConstants]
        :returns: Configured LED subsystem.
        :rtype: subsystems.led.LED
        """

        return LED(
            cls._canbus, cls._led_id, cls._led_configs, cls._num_config_attempts, cls._led_end_index
        )
