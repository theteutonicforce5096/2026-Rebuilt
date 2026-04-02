from phoenix6.configs import TalonFXConfiguration, TalonFXSConfiguration
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

    # Number of times to attempt to configure each device
    _num_config_attempts = 5

    # Flywheel Motor Configs (NEO VORTEX - TalonFXS)
    _flywheel_motor_configs = TalonFXSConfiguration()
    _flywheel_motor_configs.commutation.with_motor_arrangement(signals.MotorArrangementValue.VORTEX_JST)
    _flywheel_motor_configs.commutation.with_advanced_hall_support(signals.AdvancedHallSupportValue.ENABLED)
    _flywheel_motor_configs.motor_output.with_neutral_mode(signals.NeutralModeValue.BRAKE)
    _flywheel_motor_configs.motor_output.with_inverted(signals.InvertedValue.CLOCKWISE_POSITIVE)
    _flywheel_motor_configs.current_limits.with_stator_current_limit(80)
    _flywheel_motor_configs.current_limits.with_stator_current_limit_enable(True)
    _flywheel_motor_configs.slot0.with_k_s(0.11618)
    _flywheel_motor_configs.slot0.with_k_v(0.10922)
    _flywheel_motor_configs.slot0.with_k_p(0.125)
    _flywheel_motor_configs.slot0.with_k_i(0)
    _flywheel_motor_configs.slot0.with_k_d(0)

    # Flywheel Intake Motor Configs (KrakenX60 - TalonFX)
    _flywheel_intake_motor_configs = TalonFXConfiguration()
    _flywheel_intake_motor_configs.motor_output.with_neutral_mode(signals.NeutralModeValue.BRAKE)
    _flywheel_intake_motor_configs.motor_output.with_inverted(signals.InvertedValue.CLOCKWISE_POSITIVE)
    _flywheel_intake_motor_configs.current_limits.with_stator_current_limit(80)
    _flywheel_intake_motor_configs.current_limits.with_stator_current_limit_enable(True)
    _flywheel_intake_motor_configs.slot0.with_k_s(0.31895)
    _flywheel_intake_motor_configs.slot0.with_k_v(0.11374)
    _flywheel_intake_motor_configs.slot0.with_k_p(0.75)
    _flywheel_intake_motor_configs.slot0.with_k_i(0)
    _flywheel_intake_motor_configs.slot0.with_k_d(0)

    @classmethod
    def create_shooter(
        cls,
        get_current_swerve_state,
        get_robot_tilt,
        get_hub_center,
        launcher_offset_x: float,
        launcher_offset_y: float,
    ) -> Shooter:
        """
        Creates a Shooter subsystem instance using the configured constant values.

        :param cls: ShooterConstants class used as the source of the subsystem constants.
        :type cls: type[ShooterConstants]
        :param get_current_swerve_state: Function that returns the drivetrain state used by the shot solver.
        :type get_current_swerve_state: Callable[[], Any]
        :param get_robot_tilt: Function that returns the current robot pitch and roll in degrees.
        :type get_robot_tilt: Callable[[], tuple[float, float]]
        :param get_hub_center: Function that returns the current hub center translation for the active alliance.
        :type get_hub_center: Callable[[], wpimath.geometry.Translation2d]
        :param launcher_offset_x: Forward launcher offset from the robot reference point in meters.
        :type launcher_offset_x: float
        :param launcher_offset_y: Lateral launcher offset from the robot reference point in meters.
        :type launcher_offset_y: float
        :returns: Configured shooter subsystem.
        :rtype: subsystems.shooter.Shooter
        """

        return Shooter(
            cls._canbus,
            cls._flywheel_motor_id,
            cls._flywheel_intake_motor_id,
            cls._flywheel_motor_configs,
            cls._flywheel_intake_motor_configs,
            cls._num_config_attempts,
            get_current_swerve_state,
            get_robot_tilt,
            get_hub_center,
            launcher_offset_x,
            launcher_offset_y,
        )
