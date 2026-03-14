from phoenix6.configs import TalonFXConfiguration
from phoenix6 import CANBus, signals

from subsystems.hopper import Hopper

class HopperConstants:
    """
    Constants for Hopper Subsystem
    """

    # CANBus instance
    _canbus = CANBus("Drivetrain")

    # CAN IDs
    _mechanim_wheel_id = 50
    _agitator_wheel_id = 51

    # Number of times to attempt to configure each device
    _num_config_attempts = 5

#TODO Determine brake or coast mode and check if any need to be inverted
    # Mechanim Wheel Configs (TalonFX)
    _mechanim_wheel_configs = TalonFXConfiguration()
    # _mechanim_wheel_configs.motor_output.with_neutral_mode(signals.NeutralModeValue.COAST)
    _mechanim_wheel_configs.current_limits.with_stator_current_limit(20)
    _mechanim_wheel_configs.current_limits.with_stator_current_limit_enable(True)
    _mechanim_wheel_configs.slot0.with_k_s(0)
    _mechanim_wheel_configs.slot0.with_k_v(0)
    _mechanim_wheel_configs.slot0.with_k_p(0)
    _mechanim_wheel_configs.slot0.with_k_i(0)
    _mechanim_wheel_configs.slot0.with_k_d(0)

    # Agitator Wheel Configs (TalonFX)
    _agitator_wheel_configs = TalonFXConfiguration()
    # _agitator_wheel_configs.motor_output.with_neutral_mode(signals.NeutralModeValue.COAST)
    _agitator_wheel_configs.current_limits.with_stator_current_limit(20)
    _agitator_wheel_configs.current_limits.with_stator_current_limit_enable(True)
    _agitator_wheel_configs.slot0.with_k_s(0)
    _agitator_wheel_configs.slot0.with_k_v(0)
    _agitator_wheel_configs.slot0.with_k_p(0)
    _agitator_wheel_configs.slot0.with_k_i(0)
    _agitator_wheel_configs.slot0.with_k_d(0)

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