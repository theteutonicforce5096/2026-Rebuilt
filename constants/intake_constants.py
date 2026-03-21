from phoenix6.configs import TalonFXSConfiguration, CANcoderConfiguration
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
    _intake_arm_encoder_id = 42

    # Number of times to attempt to configure each device
    _num_config_attempts = 3

    # Arm Position Constants #TODO confirm values
    _intake_position = 0.48
    _stowed_position = -0.090

    # Intake Wheel Configs (??? - TalonFXS)
    _intake_wheel_configs = TalonFXSConfiguration()
    _intake_wheel_configs.commutation.with_motor_arrangement(signals.MotorArrangementValue.BRUSHED_DC)
    _intake_wheel_configs.commutation.with_advanced_hall_support(signals.AdvancedHallSupportValue.ENABLED)
    _intake_wheel_configs.motor_output.with_neutral_mode(signals.NeutralModeValue.BRAKE)
    _intake_wheel_configs.current_limits.with_stator_current_limit(20)
    _intake_wheel_configs.current_limits.with_stator_current_limit_enable(True)
    _intake_wheel_configs.commutation.with_brushed_motor_wiring(signals.BrushedMotorWiringValue.LEADS_A_AND_C)

    # Intake Arm Configs (NEO 550 - TalonFXS)
    _intake_arm_configs = TalonFXSConfiguration()
    _intake_arm_configs.commutation.with_motor_arrangement(signals.MotorArrangementValue.NEO550_JST)
    _intake_arm_configs.commutation.with_advanced_hall_support(signals.AdvancedHallSupportValue.ENABLED)
    _intake_arm_configs.motor_output.with_inverted(signals.InvertedValue.CLOCKWISE_POSITIVE)
    _intake_arm_configs.motor_output.with_neutral_mode(signals.NeutralModeValue.COAST)
    _intake_arm_configs.current_limits.with_stator_current_limit(40) #May need to increase
    _intake_arm_configs.current_limits.with_stator_current_limit_enable(True)
    _intake_arm_configs.external_feedback.with_feedback_remote_sensor_id(_intake_arm_encoder_id)
    _intake_arm_configs.external_feedback.with_external_feedback_sensor_source(signals.FeedbackSensorSourceValue.FUSED_CANCODER)
    _intake_arm_configs.external_feedback.with_sensor_to_mechanism_ratio(1.0)
    _intake_arm_configs.external_feedback.with_rotor_to_sensor_ratio((24 / 18) * 100)
    _intake_arm_configs.slot0.with_k_g(0)
    _intake_arm_configs.slot0.with_k_v(0)
    _intake_arm_configs.slot0.with_k_p(30)
    _intake_arm_configs.slot0.with_k_i(0)
    _intake_arm_configs.slot0.with_k_d(0)

    _intake_arm_encoder_configs = CANcoderConfiguration()
    _intake_arm_encoder_configs.magnet_sensor.sensor_direction = signals.SensorDirectionValue.CLOCKWISE_POSITIVE


    @classmethod
    def create_intake(cls) -> Intake:
        """
        Creates an Intake subsystem instance.
        """

        return Intake(
            cls._canbus,
            cls._intake_wheel_id,
            cls._intake_arm_id,
            cls._intake_arm_encoder_id,
            cls._intake_wheel_configs,
            cls._intake_arm_configs,
            cls._intake_arm_encoder_configs,
            cls._num_config_attempts,
            cls._intake_position,
            cls._stowed_position
        )