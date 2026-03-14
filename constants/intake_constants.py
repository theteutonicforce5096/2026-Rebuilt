from phoenix6.configs import TalonFXConfiguration, TalonFXSConfiguration
from phoenix6 import CANBus, signals

from subsystems.intake import Intake

class IntakeConstants:
    """
    Constants for Intake Subsystem
    """

    # CANBus instance
    _canbus = CANBus("Drivetrain")

    # CAN IDs
    _intake_wheel_id = 40
    _intake_arm_id = 41

    # Number of times to attempt to configure each device
    _num_config_attempts = 5

#TODO IDK IF I INVERTED IT RIGHT fwejoifepoijfewpjoi
#TODO Determine brake or coast mode and check if any need to be inverted
    # Intake Wheel Configs (TalonFXS)
    _intake_wheel_configs = TalonFXSConfiguration()
    _intake_wheel_configs.commutation.with_motor_arrangement(signals.MotorArrangementValue.NEO550_JST)
    _intake_wheel_configs.commutation.with_advanced_hall_support(signals.AdvancedHallSupportValue.ENABLED)
    # _intake_wheel_configs.motor_output.with_neutral_mode(signals.NeutralModeValue.COAST)
    # _intake_wheel_configs.motor_output.with_inverted(signals.InvertedValue.COUNTER_CLOCKWISE_POSITIVE)
    _intake_wheel_configs.current_limits.with_stator_current_limit(20)
    _intake_wheel_configs.current_limits.with_stator_current_limit_enable(True)

    # Intake Arm Configs (TalonFX)
    _intake_arm_configs = TalonFXConfiguration()
    # _intake_arm_configs.motor_output.with_neutral_mode(signals.NeutralModeValue.BRAKE)
    _intake_arm_configs.current_limits.with_stator_current_limit(40)
    _intake_arm_configs.current_limits.with_stator_current_limit_enable(True)
    #TODO Add encoder for intake arm
    # _intake_arm_configs.slot0.with_k_p(0)
    # _intake_arm_configs.slot0.with_k_i(0)
    # _intake_arm_configs.slot0.with_k_d(0)

    @classmethod
    def create_intake(cls) -> Intake:
        """
        Creates an Intake subsystem instance.
        """

        return Intake(
            cls._canbus,
            cls._intake_wheel_id,
            cls._intake_arm_id,
            cls._intake_wheel_configs,
            cls._intake_arm_configs,
            cls._num_config_attempts
        )