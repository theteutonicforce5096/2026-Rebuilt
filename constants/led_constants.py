from subsystems import led
from subsystems.led import LED
from phoenix6 import hardware
from phoenix6 import CANBus
from phoenix6.configs import TalonFXSConfiguration
from commands2 import Subsystem

class LEDConstants(Subsystem):
    """
    Constants for LED Subsystem
    """

    # CANBus instance
    _canbus = CANBus("rio")

    # CAN IDs
    _led_id = 3

    # Number of times to attempt to configure each device
    _num_config_attempts = 3

    # LED Configs (CANdle - TalonFXS)
    _led_configs = TalonFXSConfiguration()

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
