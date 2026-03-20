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
    _mecanum_wheel_id = 50
    _agitator_wheel_id = 51

    # Number of times to attempt to configure each device
    _num_config_attempts = 3

    # mecanum Wheel Configs (TalonFX)
    _mecanum_wheel_configs = TalonFXConfiguration()
    _mecanum_wheel_configs.motor_output.with_neutral_mode(signals.NeutralModeValue.BRAKE)
    _mecanum_wheel_configs.current_limits.with_stator_current_limit(20)
    _mecanum_wheel_configs.current_limits.with_stator_current_limit_enable(True)
    _mecanum_wheel_configs.slot0.with_k_s(0.15736)
    _mecanum_wheel_configs.slot0.with_k_v(0.12473)
    _mecanum_wheel_configs.slot0.with_k_p(0)
    _mecanum_wheel_configs.slot0.with_k_i(0)
    _mecanum_wheel_configs.slot0.with_k_d(0)

    # Agitator Wheel Configs (TalonFX)
    _agitator_wheel_configs = TalonFXConfiguration()
    _agitator_wheel_configs.motor_output.with_neutral_mode(signals.NeutralModeValue.COAST)
    _agitator_wheel_configs.current_limits.with_supply_current_limit(10)
    _agitator_wheel_configs.current_limits.with_supply_current_limit_enable(True)
    

    @classmethod
    def create_hopper(cls) -> Hopper:
        """
        Creates a Hopper subsystem instance.
        """

        return Hopper(
            cls._canbus,
            cls._mecanum_wheel_id,
            cls._agitator_wheel_id,
            cls._mecanum_wheel_configs,
            cls._agitator_wheel_configs,
            cls._num_config_attempts
        )