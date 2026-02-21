from phoenix6.configs import TalonFXConfiguration, TalonFXSConfiguration, CANcoderConfiguration
from phoenix6 import signals

from subsystems.shooter import Shooter

class ShooterConstants:
    """
    Constants for Shooter Subsystem
    """

    # CAN IDs
    _flywheel_motor_id = 30
    _flywheel_intake_motor_id = 31
    _flywheel_encoder_id = 32

    #Flywheel Motor Configs (NEO VORTEX - TalonFXS)
    _flywheel_motor_configs = TalonFXSConfiguration()
    _flywheel_motor_configs.slot0.k_s = 2.5
    _flywheel_motor_configs.slot0.k_v = 0
    _flywheel_motor_configs.slot0.k_p = 5
    _flywheel_motor_configs.slot0.k_i = 0
    _flywheel_motor_configs.slot0.k_d = 0
    _flywheel_motor_configs.motor_output.neutral_mode = signals.NeutralModeValue.COAST
    _flywheel_motor_configs.current_limits.stator_current_limit = 100
    _flywheel_motor_configs.current_limits.stator_current_limit_enable = True
    _flywheel_motor_configs.external_feedback.feedback_remote_sensor_id = _flywheel_encoder_id
    _flywheel_motor_configs.external_feedback.external_feedback_sensor_source = signals.FeedbackSensorSourceValue.FUSED_CANCODER
    _flywheel_motor_configs.external_feedback.sensor_to_mechanism_ratio = 1.0
    _flywheel_motor_configs.external_feedback.rotor_to_sensor_ratio = 1.0

    #Flywheel Intake Motor Configs (Falcon 500 - TalonFX)
    _flywheel_intake_motor_configs = TalonFXConfiguration()
    _flywheel_intake_motor_configs.slot0.k_s = 2.5
    _flywheel_intake_motor_configs.slot0.k_v = 0
    _flywheel_intake_motor_configs.slot0.k_p = 5
    _flywheel_intake_motor_configs.slot0.k_i = 0
    _flywheel_intake_motor_configs.slot0.k_d = 0
    _flywheel_intake_motor_configs.motor_output.neutral_mode = signals.NeutralModeValue.COAST
    _flywheel_intake_motor_configs.current_limits.stator_current_limit = 50
    _flywheel_intake_motor_configs.current_limits.stator_current_limit_enable = True

    #Flywheel Encoder Configs (WCP ThroughBore Encoder - CANcoder)
    _flywheel_encoder_configs = CANcoderConfiguration()
    _flywheel_encoder_configs.magnet_sensor.sensor_direction = signals.SensorDirectionValue.COUNTER_CLOCKWISE_POSITIVE

    @classmethod
    def create_shooter(cls) -> Shooter:
        """
        Creates a Shooter subsystem instance.
        """

        return Shooter(
            cls._flywheel_motor_id,
            cls._flywheel_intake_motor_id,
            cls._flywheel_encoder_id,
            cls._flywheel_motor_configs,
            cls._flywheel_intake_motor_configs,
            cls._flywheel_encoder_configs
        )