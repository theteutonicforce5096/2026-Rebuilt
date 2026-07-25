from commands2 import Subsystem
from phoenix6 import CANBus
from phoenix6.configs import TalonFXSConfiguration
from phoenix6.controls import VoltageOut
from phoenix6.hardware import TalonFXS
from wpilib import RobotBase

from subsystems.device_config import check_signal_status, configure_device


class Intake(Subsystem):
    """
    Drives the wheel that pulls balls up off the floor and spits them back out.

    The wheel is a brushed motor run open-loop in both directions. The pivoting arm that carries it
    is a separate subsystem, so a shot feeding through this wheel and an arm move can run at the
    same time without one cancelling the other.
    """

    def __init__(
        self,
        canbus: CANBus,
        wheel_id: int,
        wheel_configs: TalonFXSConfiguration,
        num_config_attempts: int,
    ):
        """
        Initialize the intake using the specified constants.

        :param canbus: CANBus instance that electronics are on
        :type canbus: phoenix6.CANBus
        :param wheel_id: CAN ID of the intake wheel
        :type wheel_id: int
        :param wheel_configs: Configs for the intake wheel
        :type wheel_configs: phoenix6.configs.TalonFXSConfiguration
        :param num_config_attempts: Number of times to attempt to configure the device
        :type num_config_attempts: int
        """
        Subsystem.__init__(self)

        # Create motor
        self.wheel = TalonFXS(wheel_id, canbus)

        # Apply motor configs
        configure_device(self.wheel, wheel_configs, num_config_attempts)

        if not RobotBase.isSimulation():
            # Nothing reads the wheel, so every signal stays trimmed to keep the bus clear for the
            # devices that are polled each loop.
            check_signal_status(
                self.wheel.optimize_bus_utilization(),
                "Intake wheel bus optimization",
            )

        # Create control request
        self.voltage_request = VoltageOut(output=0)

    def set_intake_speed(self, wheel_volts):
        """
        Apply the requested intake-wheel voltage.

        :param wheel_volts: Voltage to apply to the intake wheel motor.
        :type wheel_volts: float
        """
        self.wheel.set_control(self.voltage_request.with_output(wheel_volts))
