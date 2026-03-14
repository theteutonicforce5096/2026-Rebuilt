from phoenix6.configs import TalonFXConfiguration
from phoenix6 import CANBus, signals

from subsystems.hopper import Hopper

class HopperConstants:
    """
    Constants for Hopper Subsystem
    """

    # CANBus instance
    _canbus = CANBus("Drivetrain")

#TODO why is it the same ._.
 #TODO i think we added more motors or something
    # CAN IDs
    _mechanim_wheel_id = 50
    _agitator_wheel_id = 50


    # Number of times to attempt to configure each device
    _num_config_attempts = 5

    # Mechanim Wheel Configs (TalonFX)
    _mechanim_wheel_configs = TalonFXConfiguration()
    _mechanim_wheel_configs.current_limits.with_stator_current_limit(20)
    _mechanim_wheel_configs.current_limits.with_stator_current_limit_enable(True)

    # Agitator Wheel Configs (TalonFX)
    _agitator_wheel_configs = TalonFXConfiguration()
    _agitator_wheel_configs.current_limits.with_stator_current_limit(20)
    _agitator_wheel_configs.current_limits.with_stator_current_limit_enable(True)

    @classmethod
    def create_hopper(cls) -> Hopper:
        """
        Creates a Hopper subsystem instance.
        """

        return Hopper(
            cls._canbus,
            cls._mechanim_wheel_id,
            cls._agitator_wheel_id,
            cls._mechanim_wheel_configs,
            cls._agitator_wheel_configs,
            cls._num_config_attempts
        )