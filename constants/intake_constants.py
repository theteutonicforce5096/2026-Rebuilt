from phoenix6 import CANBus, signals
from phoenix6.configs import CANcoderConfiguration, TalonFXConfiguration, TalonFXSConfiguration

from subsystems.intake import Intake


class IntakeConstants:
    """Hardware IDs, motor configs, arm positions, and stall thresholds for the intake."""

    # CANBus instance
    _canbus = CANBus("Drivetrain")

    # CAN IDs
    _intake_wheel_id = 40
    _intake_arm_id = 41
    _intake_arm_encoder_id = 42

    # Number of times to attempt to configure each device
    _num_config_attempts = 25

    # Arm positions in mechanism rotations, measured off the fused CANcoder
    _intake_position = 0.15
    _stowed_position = 0.559
    _shooting_position = 0.36

    # Stall detection: the arm must draw this much current while moving this slowly for this
    # long before it counts as stalled
    _stall_current_threshold = 3.0  # amps of stator current
    _stall_velocity_threshold = 0.15  # rotations per second
    _stall_time_threshold = 0.25  # seconds

    # Hard timeout on a stall-watched arm move; the farthest no-jam move takes about 5 seconds,
    # so this adds margin without letting a missed stall drive the arm forever
    _arm_move_timeout_sec = 6.0

    # Obstruction detection: the stretch of travel where the arm can pinch against the frame,
    # and the current that means something is caught in it
    _arm_movement_pathway_low = 0.3
    _arm_movement_pathway_high = 0.5
    _obstruction_current_threshold = 20  # amps of stator current
    # Rotations around the shooting position where high current is normal, not an obstruction
    _obstruction_dead_band = 0.01

    # Intake wheel voltages for pulling balls in and spitting them back out
    _intake_volts = 12.0
    _eject_volts = -12.0

    # Minimum time the intake runs before re-pressing the button can cancel it
    _intake_min_run_sec = 2.0

    # Intake Wheel Configs (brushed motor - TalonFXS)
    _intake_wheel_configs = TalonFXSConfiguration()
    _intake_wheel_configs.commutation.with_motor_arrangement(
        signals.MotorArrangementValue.BRUSHED_DC
    )
    _intake_wheel_configs.commutation.with_advanced_hall_support(
        signals.AdvancedHallSupportValue.ENABLED
    )
    _intake_wheel_configs.motor_output.with_neutral_mode(signals.NeutralModeValue.BRAKE)
    _intake_wheel_configs.current_limits.with_stator_current_limit(25)
    _intake_wheel_configs.current_limits.with_stator_current_limit_enable(True)
    _intake_wheel_configs.commutation.with_brushed_motor_wiring(
        signals.BrushedMotorWiringValue.LEADS_A_AND_C
    )

    # Intake Arm Configs (Falcon500 - TalonFX)
    _intake_arm_configs = TalonFXConfiguration()
    _intake_arm_configs.motor_output.with_inverted(signals.InvertedValue.COUNTER_CLOCKWISE_POSITIVE)
    _intake_arm_configs.motor_output.with_neutral_mode(signals.NeutralModeValue.BRAKE)
    _intake_arm_configs.current_limits.with_stator_current_limit(12)
    _intake_arm_configs.current_limits.with_stator_current_limit_enable(True)
    _intake_arm_configs.feedback.with_feedback_remote_sensor_id(_intake_arm_encoder_id)
    _intake_arm_configs.feedback.with_feedback_sensor_source(
        signals.FeedbackSensorSourceValue.FUSED_CANCODER
    )
    _intake_arm_configs.feedback.with_sensor_to_mechanism_ratio(1.0)
    _intake_arm_configs.feedback.with_rotor_to_sensor_ratio(125 * (4 / 3))
    _intake_arm_configs.closed_loop_general.with_continuous_wrap(True)
    _intake_arm_configs.slot0.with_k_p(28)
    _intake_arm_configs.slot0.with_k_i(0)
    _intake_arm_configs.slot0.with_k_d(0)

    _intake_arm_encoder_configs = CANcoderConfiguration()
    _intake_arm_encoder_configs.magnet_sensor.with_sensor_direction(
        signals.SensorDirectionValue.CLOCKWISE_POSITIVE
    )
    _intake_arm_encoder_configs.magnet_sensor.with_absolute_sensor_discontinuity_point(1)

    @classmethod
    def create_intake(cls) -> Intake:
        """
        Create an Intake subsystem instance using the configured constant values.

        :returns: Configured intake subsystem.
        :rtype: subsystems.intake.Intake
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
            cls._stowed_position,
            cls._shooting_position,
            cls._stall_current_threshold,
            cls._stall_velocity_threshold,
            cls._stall_time_threshold,
            cls._arm_movement_pathway_low,
            cls._arm_movement_pathway_high,
            cls._obstruction_current_threshold,
            cls._obstruction_dead_band,
        )
