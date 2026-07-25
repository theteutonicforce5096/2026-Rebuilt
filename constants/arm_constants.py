from phoenix6 import CANBus, signals
from phoenix6.configs import CANcoderConfiguration, TalonFXConfiguration

from subsystems.arm import Arm


class ArmConstants:
    """Hardware IDs, motor configs, positions, and stall thresholds for the intake arm."""

    # CANBus instance
    _canbus = CANBus("Drivetrain")

    # CAN IDs
    _arm_id = 41
    _arm_encoder_id = 42

    # Number of times to attempt to configure each device. Sized so retries span the several
    # seconds after code start when Phoenix license checks can reject configs on a pro bus.
    _num_config_attempts = 20

    # Arm positions in mechanism rotations, measured off the fused CANcoder
    _intake_position = 0.165
    _shooting_position = 0.36
    _stowed_position = 0.534

    # Stall detection: a commanded arm must close at least this much position error over each
    # detection window or it counts as stalled. The slowest healthy move covers 0.369 rotations in
    # about 4.5 seconds, closing roughly 0.016 rotations per window, so this leaves better than 3x
    # margin while a jammed arm closes essentially none.
    _stall_progress_threshold = 0.001  # rotations of error closed per window
    _stall_time_threshold = 0.2  # seconds per detection window

    # Time after a new setpoint during which no stall may be declared. A setpoint that reverses a
    # move in flight makes the arm lose ground while its momentum turns around, which is
    # indistinguishable from a jam until the reversal finishes.
    _stall_grace_sec = 0.25

    # Hard timeout on a stall-watched arm move. The farthest no-jam move takes about 4.5 seconds at
    # the arm's nominal speed, and a move that reaches this limit has its power cut, so the margin
    # above that has to cover a depleted battery: this tolerates the arm running a third slower
    # than nominal before a healthy move is cut short. The stall watch catches a real jam in about
    # a quarter second, so this only ever fires for a jam the watch missed and costs nothing to
    # sit well clear of the healthy case.
    _arm_move_timeout_sec = 5.0

    # Arm Configs (Falcon500 - TalonFX)
    _arm_configs = TalonFXConfiguration()
    _arm_configs.motor_output.with_inverted(signals.InvertedValue.COUNTER_CLOCKWISE_POSITIVE)
    _arm_configs.motor_output.with_neutral_mode(signals.NeutralModeValue.BRAKE)
    # The ~167:1 reduction turns 12 A into roughly 36 N-m at the arm, ample for every move while
    # limiting how hard a jammed arm can push during stall detection
    _arm_configs.current_limits.with_stator_current_limit(12)
    _arm_configs.current_limits.with_stator_current_limit_enable(True)
    _arm_configs.feedback.with_feedback_remote_sensor_id(_arm_encoder_id)
    _arm_configs.feedback.with_feedback_sensor_source(
        signals.FeedbackSensorSourceValue.FUSED_CANCODER
    )
    _arm_configs.feedback.with_sensor_to_mechanism_ratio(1.0)
    _arm_configs.feedback.with_rotor_to_sensor_ratio(125 * (4 / 3))
    _arm_configs.closed_loop_general.with_continuous_wrap(True)
    # Proportional-only control, so the arm rests wherever k_p times the remaining error can no
    # longer break static friction. That resting error is what sets the arrival tolerance and the
    # stall stand-down band in the Arm subsystem.
    _arm_configs.slot0.with_k_p(35)
    _arm_configs.slot0.with_k_i(0)
    _arm_configs.slot0.with_k_d(0)

    _arm_encoder_configs = CANcoderConfiguration()
    _arm_encoder_configs.magnet_sensor.with_sensor_direction(
        signals.SensorDirectionValue.CLOCKWISE_POSITIVE
    )
    _arm_encoder_configs.magnet_sensor.with_absolute_sensor_discontinuity_point(1)

    @classmethod
    def create_arm(cls) -> Arm:
        """
        Create an Arm subsystem instance using the configured constant values.

        :returns: Configured arm subsystem.
        :rtype: subsystems.arm.Arm
        """
        return Arm(
            cls._canbus,
            cls._arm_id,
            cls._arm_encoder_id,
            cls._arm_configs,
            cls._arm_encoder_configs,
            cls._num_config_attempts,
            cls._intake_position,
            cls._stowed_position,
            cls._shooting_position,
            cls._stall_progress_threshold,
            cls._stall_time_threshold,
            cls._stall_grace_sec,
        )
