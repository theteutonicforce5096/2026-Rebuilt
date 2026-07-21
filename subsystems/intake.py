from commands2 import Subsystem
from phoenix6 import CANBus
from phoenix6.configs import CANcoderConfiguration, TalonFXSConfiguration, TalonFXConfiguration
from phoenix6.controls import PositionVoltage, VoltageOut
from phoenix6.hardware import CANcoder, TalonFXS, TalonFX
from wpilib import RobotBase, SmartDashboard

from subsystems.device_config import configure_device

class Intake(Subsystem):
    """
    Class for controlling intake.
    """

    def __init__(self, canbus: CANBus, intake_wheel_id: int, intake_arm_id: int, 
                 intake_arm_encoder_id: int, intake_wheel_configs: TalonFXSConfiguration, 
                 intake_arm_configs: TalonFXConfiguration, intake_arm_encoder_configs: CANcoderConfiguration,
                 num_config_attempts: int, intake_position: float, stowed_position: float, shooting_position: float):
        """
        Constructor for initializing intake using the specified constants.

        :param canbus: CANBus instance that electronics are on
        :type canbus: phoenix6.CANBus
        :param intake_wheel_id: CAN ID of the intake wheel
        :type intake_wheel_id: int
        :param intake_arm_id: CAN ID of the intake arm
        :type intake_arm_id: int
        :param intake_arm_encoder_id: CAN ID of the intake arm encoder
        :type intake_arm_encoder_id: int
        :param intake_wheel_configs: Configs for the intake wheel
        :type intake_wheel_configs: phoenix6.configs.TalonFXSConfiguration
        :param intake_arm_configs: Configs for the intake arm
        :type intake_arm_configs: phoenix6.configs.TalonFXConfiguration
        :param intake_arm_encoder_configs: Configs for the intake arm encoder
        :type intake_arm_encoder_configs: phoenix6.configs.CANcoderConfiguration
        :param num_config_attempts: Number of times to attempt to configure each device
        :type num_config_attempts: int
        :param intake_position: Encoder position where arm is down
        :type intake_position: float
        :param stowed_position: Encoder position where arm is up
        :type stowed_position: float
        :param shooting_position: Encoder position where the arm is at an intermediate position
        :type shooting_position: float
        """

        Subsystem.__init__(self)

        # Create motors
        self.intake_wheel = TalonFXS(intake_wheel_id, canbus)
        self.intake_arm = TalonFX(intake_arm_id, canbus)
        self.intake_arm_encoder = CANcoder(intake_arm_encoder_id, canbus)

        # Apply motor configs
        configure_device(self.intake_wheel, intake_wheel_configs, num_config_attempts)
        configure_device(self.intake_arm, intake_arm_configs, num_config_attempts)
        configure_device(self.intake_arm_encoder, intake_arm_encoder_configs, num_config_attempts)
        if not RobotBase.isSimulation():
            # Trim every signal to 0 Hz first, then raise only the encoder position we
            # depend on. Setting the frequency before optimizing would let the optimize
            # pass drop the encoder position back toward 0 Hz.
            self.intake_wheel.optimize_bus_utilization()
            self.intake_arm.optimize_bus_utilization()
            self.intake_arm_encoder.optimize_bus_utilization()
            self.intake_arm_encoder.get_position().set_update_frequency(1000.0)

        # Create PID control requests
        self.voltage_request = VoltageOut(output = 0)
        self.position_voltage_request = PositionVoltage(position = 0)

        # Arm positions
        self.intake_position = intake_position
        self.stowed_position = stowed_position
        self.shooting_position = shooting_position

    def periodic(self):
        """
        Publish the current intake wheel voltage for driver-station debugging.
        """
        intake_wheel_voltage = self.intake_wheel.get_motor_voltage().value_as_double
        SmartDashboard.putNumber("Intake Status", intake_wheel_voltage)

#Intake Wheel Functions
    def set_intake_speed(self, intake_wheel_volts):
        """
        Apply the requested intake-wheel voltage.

        :param intake_wheel_volts: Voltage to apply to the intake wheel motor.
        :type intake_wheel_volts: float
        """
        self.intake_wheel.set_control(
            self.voltage_request.with_output(intake_wheel_volts)
        )

    def set_setpoint(self, position):
        """
        Command the intake arm to the requested closed-loop position.

        :param position: Desired intake arm position in mechanism rotations.
        :type position: float
        """
        self.intake_arm.set_control(
            self.position_voltage_request.with_position(position)
        )

    def arm_down(self):
        """
        Move the intake arm to the intake position.
        """
        self.set_setpoint(self.intake_position)

    def arm_up(self):
        """
        Move the intake arm to the stowed position.
        """
        self.set_setpoint(self.stowed_position)

    def arm_down_intermediate(self):
        """
        Move the intake arm to the intermediate shooting position.
        """
        self.set_setpoint(self.shooting_position)
