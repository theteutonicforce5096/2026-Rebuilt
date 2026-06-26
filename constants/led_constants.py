from subsystems import led
from subsystems.led import LED
from phoenix6 import hardware
from phoenix6 import CANBus
from phoenix6.configs import TalonFXSConfiguration
from phoenix6.configs import CANdleConfiguration
from phoenix6.configs.candle_configs import LEDConfigs
from commands2 import Subsystem
import phoenix6
from phoenix6.signals import StripTypeValue


class LEDConstants(Subsystem):
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
    _led_configs.led.with_strip_type(phoenix6.signals.StripTypeValue.GRB)


    @classmethod
    def create_led(cls) -> LED:
        """
        Creates an LED subsystem instance.
        """

        return LED(
            cls._canbus,
            cls._led_id,
            cls._led_configs,
            cls._num_config_attempts
        )
