from commands2 import PrintCommand, Subsystem
from phoenix6 import CANBus
from phoenix6.configs import CANcoderConfiguration, TalonFXSConfiguration
from phoenix6.controls import PositionVoltage, VoltageOut
from phoenix6.hardware import CANcoder, TalonFXS
from phoenix6.status_code import StatusCode
from wpilib import RobotBase

class Intake(Subsystem):
    """
    Class for controlling intake.
    """

    def __init__(self, canbus: CANBus, intake_wheel_id: int, intake_arm_id: int, 
                 intake_arm_encoder_id: int, intake_wheel_configs: TalonFXSConfiguration, 
                 intake_arm_configs: TalonFXSConfiguration, intake_arm_encoder_configs: CANcoderConfiguration,
                 num_config_attempts: int, intake_position: float, stowed_position: float):
        """
        Constructor for initializing shooter using the specified constants.

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
        :type intake_arm_configs: phoenix6.configs.TalonFXSConfiguration
        :param intake_arm_encoder_configs: Configs for the intake arm encoder
        :type intake_arm_encoder_configs: phoenix6.configs.CANcoderConfiguration
        :param num_config_attempts: Number of times to attempt to configure each device
        :type num_config_attempts: int
        :param intake_position: IT USED TO BE intake_arm_position. IM ASSUMING THIS IS THE ANGLE WHEN THE ARM IS ALL THE WAY DOWN
        :type intake_position: float
        :param stowed_position: IM ASSUMING THIS IS THE ANGLE WHEN ARM IS ALL THE WAY UP
        :type stowed_position: float
        """

        Subsystem.__init__(self) 
        
        # Create motors
        self.intake_wheel = TalonFXS(intake_wheel_id, canbus)
        self.intake_arm = TalonFXS(intake_arm_id, canbus)
        self.intake_arm_encoder = CANcoder(intake_arm_encoder_id, canbus)

        # Apply motor configs
        self._configure_device(self.intake_wheel, intake_wheel_configs, num_config_attempts)
        self._configure_device(self.intake_arm, intake_arm_configs, num_config_attempts)
        self._configure_device(self.intake_arm_encoder, intake_arm_encoder_configs, num_config_attempts)
        if RobotBase.isSimulation() == False:
            self.intake_arm_encoder.get_position().set_update_frequency(1000.0)
            self.intake_wheel.optimize_bus_utilization()
            self.intake_arm.optimize_bus_utilization()
            self.intake_arm_encoder.optimize_bus_utilization()

        # Create PID control requests
        self.voltage_request = VoltageOut(output = 0)
        self.position_voltage_request = PositionVoltage(position = 0)
        # Placeholder values, will need to be tuned

        # Arm Positions because apparently we need those
        self.intake_position = intake_position
        self.stowed_position = stowed_position

    def _configure_device(self, device: TalonFXS | CANcoder, 
                          configs: TalonFXSConfiguration | CANcoderConfiguration, 
                          num_attempts: int) -> None:
        """
        Configures a CTRE motor controller or CANcoder with the specified configs, 
        retrying up to num_attempts times if the configuration fails.
        
        :param device: The CTRE motor controller or CANcoder to configure
        :type device: phoenix6.hardware.TalonFXS | phoenix6.hardware.CANcoder
        :param configs: The configuration to apply to the device
        :type configs: phoenix6.configs.TalonFXSConfiguration | phoenix6.configs.CANcoderConfiguration
        :param num_attempts: Number of times to attempt to configure each device
        :type num_attempts: int
        """
        for _ in range(num_attempts):
            status_code: StatusCode = device.configurator.apply(configs)
            if status_code.is_ok():
                break
        if not status_code.is_ok():
            PrintCommand(f"Device with CAN ID {device.device_id} failed to config with error: {status_code.name}").schedule()

#Intake Wheel Functions 
    def set_intake_speed(self, intake_wheel_volts):
        self.intake_wheel.set_control(
            self.voltage_request.with_output(intake_wheel_volts)
        )

    def run_intake_wheel(self, intake_wheel_volts):
        return self.runOnce(
            lambda: self.set_intake_speed(intake_wheel_volts)
        )

    def set_setpoint(self, position):
        self.intake_arm.set_control(
            self.position_voltage_request.with_position(position)
        )
        print(position)

    def arm_down(self):
        self.set_setpoint(self.intake_position) 

    def move_arm(self, voltage):
        self.intake_arm.setVoltage(voltage)
        
    
    def arm_up(self):
        self.set_setpoint(self.stowed_position)  
      

        

    
            
