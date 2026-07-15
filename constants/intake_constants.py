from phoenix6.configs import TalonFXSConfiguration
from phoenix6 import CANBus, signals

from subsystems.intake import Intake

class IntakeConstants:
    """
    Constants for Intake Subsystem
    """

    # CANBus instance
    _canbus = CANBus("rio")

    # CAN IDs
    _intake_arm_id = 41

    # Number of times to attempt to configure each device
    _num_config_attempts = 3

    # Arm Position Constants 
    _intake_position = 0
    _stowed_position = 10

    # Stall Detection Constants
    _stall_current_threshold = 30.0 # stator current
    _stall_velocity_threshold = .25 # rps???
    _stall_time_threshold = .25

    # Intake Wheel Configs (Random ahh brushed motor - TalonFXS)
    _intake_arm_configs = TalonFXSConfiguration()
    _intake_arm_configs.commutation.with_motor_arrangement(signals.MotorArrangementValue.NEO550_JST)
    _intake_arm_configs.commutation.with_advanced_hall_support(signals.AdvancedHallSupportValue.ENABLED)
    _intake_arm_configs.motor_output.with_neutral_mode(signals.NeutralModeValue.BRAKE)
    _intake_arm_configs.current_limits.with_stator_current_limit(25)
    _intake_arm_configs.current_limits.with_stator_current_limit_enable(True)
    _intake_arm_configs.slot0.with_k_p(1)
    _intake_arm_configs.slot0.with_k_i(0)
    _intake_arm_configs.slot0.with_k_d(0)

    @classmethod
    def create_intake(cls) -> Intake:
        """
        Creates an Intake subsystem instance using the configured constant values.

        :param cls: IntakeConstants class used as the source of the subsystem constants.
        :type cls: type[IntakeConstants]
        :returns: Configured intake subsystem.
        :rtype: subsystems.intake.Intake
        """

        return Intake(
            cls._canbus,
            cls._intake_arm_id,
            cls._intake_arm_configs,
            cls._num_config_attempts,
            cls._intake_position,
            cls._stowed_position,
            cls._stall_current_threshold,
            cls._stall_velocity_threshold,
            cls._stall_time_threshold
        )