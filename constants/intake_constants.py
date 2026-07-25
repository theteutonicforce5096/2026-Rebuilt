from phoenix6 import CANBus, signals
from phoenix6.configs import TalonFXSConfiguration

from subsystems.intake import Intake


class IntakeConstants:
    """Hardware ID, motor config, and voltages for the intake wheel."""

    # CANBus instance
    _canbus = CANBus("Drivetrain")

    # CAN IDs
    _wheel_id = 40

    # Number of times to attempt to configure the device. Sized so retries span the several
    # seconds after code start when Phoenix license checks can reject configs on a pro bus.
    _num_config_attempts = 25

    # Intake wheel voltages for pulling balls in and spitting them back out
    _intake_volts = 12.0
    _eject_volts = -12.0

    # Intake Wheel Configs (brushed motor - TalonFXS)
    _wheel_configs = TalonFXSConfiguration()
    _wheel_configs.commutation.with_motor_arrangement(signals.MotorArrangementValue.BRUSHED_DC)
    _wheel_configs.commutation.with_advanced_hall_support(signals.AdvancedHallSupportValue.ENABLED)
    _wheel_configs.motor_output.with_neutral_mode(signals.NeutralModeValue.BRAKE)
    _wheel_configs.current_limits.with_stator_current_limit(25)
    _wheel_configs.current_limits.with_stator_current_limit_enable(True)
    # Supply cap bounds what a jammed wheel can pull from the battery
    _wheel_configs.current_limits.with_supply_current_limit(15)
    _wheel_configs.current_limits.with_supply_current_limit_enable(True)
    _wheel_configs.commutation.with_brushed_motor_wiring(
        signals.BrushedMotorWiringValue.LEADS_A_AND_C
    )

    @classmethod
    def create_intake(cls) -> Intake:
        """
        Create an Intake subsystem instance using the configured constant values.

        :returns: Configured intake subsystem.
        :rtype: subsystems.intake.Intake
        """
        return Intake(
            cls._canbus,
            cls._wheel_id,
            cls._wheel_configs,
            cls._num_config_attempts,
        )
