from phoenix6.configs import TalonFXConfiguration, CANcoderConfiguration
from phoenix6 import CANBus, signals

from subsystems.intake import Intake

class IntakeConstants:
    """
    Constants for Intake Subsystem
    """

    # CANBus instance
    _canbus = CANBus("Drivetrain")

    # CAN IDs
    _intake_arm_id = 41
    _intake_arm_encoder_id = 42

    # Number of times to attempt to configure each device
    _num_config_attempts = 3

    # Arm Position Constants 
    _intake_position = 0.15
    _stowed_position = 0.559

    # Stall Detection Constants
    _stall_current_threshold = 3.0 # stator current 
    _stall_velocity_threshold = .15 # rps
    _stall_time_threshold = .25 

    # [OBSTRUCTION!!2!!2!!@!12@@!@@!1] Constants
    _arm_movement_pathway_low = .3
    _arm_movement_pathway_high = .52
    _obstruction_current_threshold = 6.0 #stator

    # Intake Arm Configs (Falcon500 - TalonFX)
    _intake_arm_configs = TalonFXConfiguration()
    _intake_arm_configs.motor_output.with_inverted(signals.InvertedValue.COUNTER_CLOCKWISE_POSITIVE)
    _intake_arm_configs.motor_output.with_neutral_mode(signals.NeutralModeValue.BRAKE)
    _intake_arm_configs.current_limits.with_stator_current_limit(12) 
    _intake_arm_configs.current_limits.with_stator_current_limit_enable(True)
    _intake_arm_configs.feedback.with_feedback_remote_sensor_id(_intake_arm_encoder_id)
    _intake_arm_configs.feedback.with_feedback_sensor_source(signals.FeedbackSensorSourceValue.FUSED_CANCODER)
    _intake_arm_configs.feedback.with_sensor_to_mechanism_ratio(1.0)
    _intake_arm_configs.feedback.with_rotor_to_sensor_ratio(125 * (4/3))
    _intake_arm_configs.closed_loop_general.with_continuous_wrap(True)
    _intake_arm_configs.slot0.with_k_p(28)
    _intake_arm_configs.slot0.with_k_i(0)
    _intake_arm_configs.slot0.with_k_d(0)

    _intake_arm_encoder_configs = CANcoderConfiguration()
    _intake_arm_encoder_configs.magnet_sensor.with_sensor_direction(signals.SensorDirectionValue.CLOCKWISE_POSITIVE)
    _intake_arm_encoder_configs.magnet_sensor.with_absolute_sensor_discontinuity_point(1)

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
            cls._intake_arm_encoder_id,
            cls._intake_arm_configs,
            cls._intake_arm_encoder_configs,
            cls._num_config_attempts,
            cls._intake_position,
            cls._stowed_position,
            cls._stall_current_threshold,
            cls._stall_velocity_threshold,
            cls._stall_time_threshold,
            cls._arm_movement_pathway_low,
            cls._arm_movement_pathway_high,
            cls._obstruction_current_threshold
        )