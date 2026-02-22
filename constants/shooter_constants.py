from phoenix6.configs import TalonFXConfiguration, TalonFXSConfiguration, CANcoderConfiguration
from phoenix6 import CANBus, signals

from subsystems.shooter import Shooter

class ShooterConstants:
    """
    Constants for Shooter Subsystem
    """

    # CANBus instance
    _canbus = CANBus("Drivetrain")

    # CAN IDs
    _flywheel_motor_id = 30
    _flywheel_intake_motor_id = 31
    _flywheel_encoder_id = 32

#TODO Big oscillation thing. Try to test without encoder. Encoder may be causing oscillation. 
#Oscillation detected when doing feedforward tuning on NEO VORTEX, but not when doing Falcon 500
    #Flywheel Motor Configs (NEO VORTEX - TalonFXS)
    _flywheel_motor_configs = TalonFXSConfiguration()
    _flywheel_motor_configs.commutation.with_motor_arrangement(signals.MotorArrangementValue.VORTEX_JST)
    _flywheel_motor_configs.commutation.with_advanced_hall_support(signals.AdvancedHallSupportValue.ENABLED)
    _flywheel_motor_configs.motor_output.with_neutral_mode(signals.NeutralModeValue.COAST)
    _flywheel_motor_configs.motor_output.with_inverted(signals.InvertedValue.CLOCKWISE_POSITIVE)
    _flywheel_motor_configs.current_limits.with_stator_current_limit(60)
    _flywheel_motor_configs.current_limits.with_stator_current_limit_enable(True)
    _flywheel_motor_configs.external_feedback.with_feedback_remote_sensor_id(_flywheel_encoder_id)
    _flywheel_motor_configs.external_feedback.with_external_feedback_sensor_source(signals.FeedbackSensorSourceValue.REMOTE_CANCODER)
    _flywheel_motor_configs.external_feedback.with_sensor_to_mechanism_ratio(1.0)
    _flywheel_motor_configs.external_feedback.with_rotor_to_sensor_ratio(1.0)
    _flywheel_motor_configs.slot0.k_s = 0.22
    _flywheel_motor_configs.slot0.k_v = 0.105
    _flywheel_motor_configs.slot0.k_p = 0.05
    _flywheel_motor_configs.slot0.k_i = 0
    _flywheel_motor_configs.slot0.k_d = 0

    #Flywheel Intake Motor Configs (Falcon 500 - TalonFX)
    _flywheel_intake_motor_configs = TalonFXConfiguration()
    _flywheel_intake_motor_configs.motor_output.with_neutral_mode(signals.NeutralModeValue.COAST)
    _flywheel_intake_motor_configs.motor_output.with_inverted(signals.InvertedValue.CLOCKWISE_POSITIVE)
    _flywheel_intake_motor_configs.current_limits.with_stator_current_limit(40)
    _flywheel_intake_motor_configs.current_limits.with_stator_current_limit_enable(True)
    _flywheel_intake_motor_configs.slot0.k_s = 0.23
    _flywheel_intake_motor_configs.slot0.k_v = 0.1125
    _flywheel_intake_motor_configs.slot0.k_p = 0.25
    _flywheel_intake_motor_configs.slot0.k_i = 0
    _flywheel_intake_motor_configs.slot0.k_d = 0

    #Flywheel Encoder Configs (WCP ThroughBore Encoder - CANcoder)
    _flywheel_encoder_configs = CANcoderConfiguration()
    _flywheel_encoder_configs.magnet_sensor.sensor_direction = signals.SensorDirectionValue.COUNTER_CLOCKWISE_POSITIVE

    @classmethod
    def create_shooter(cls) -> Shooter:
        """
        Creates a Shooter subsystem instance.
        """

        return Shooter(
            cls._canbus,
            cls._flywheel_motor_id,
            cls._flywheel_intake_motor_id,
            cls._flywheel_encoder_id,
            cls._flywheel_motor_configs,
            cls._flywheel_intake_motor_configs,
            cls._flywheel_encoder_configs
        )