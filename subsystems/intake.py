'''
TODO: I am making the hopper and the intake two seperate things so it will make everything easier
'''
import wpilib
import phoenix6
import phoenix6.controls
import phoenix6.configs
import phoenix6.signals
import rev 
import commands2
from commands2 import Subsystem
from commands2.cmd import print_

from commands2 import PrintCommand, SequentialCommandGroup, DeferredCommand

from phoenix6 import CANBus
from phoenix6.configs import TalonFXConfiguration, TalonFXSConfiguration
from phoenix6.controls import VelocityVoltage, PositionVoltage
from phoenix6.hardware import TalonFX, TalonFXS
from phoenix6.status_code import StatusCode

from ntcore import NetworkTableInstance
from wpilib.shuffleboard import Shuffleboard


class Intake(Subsystem): # <-- Telling subsystem that its part of it too
    """
    Class for controlling intake.
    """

    def __init__(self, canbus: CANBus, intake_wheel_id: int, intake_arm_id: int, 
                 intake_wheel_configs: TalonFXSConfiguration, 
                 intake_arm_configs: TalonFXConfiguration,
                 num_config_attempts: int):
        """
        Constructor for initializing shooter using the specified constants.

        :param canbus: CANBus instance that electronics are on
        :type canbus: phoenix6.CANBus
        :param intake_wheel_id: CAN ID of the intake wheel
        :type intake_wheel_id: int
        :param intake_arm_id: CAN ID of the intake arm
        :type intake_arm_id: int
        :param intake_wheel_configs: Configs for the intake wheel
        :type intake_wheel_configs: phoenix6.configs.TalonFXSConfiguration
        :param intake_arm_configs: Configs for the intake arm
        :type intake_arm_configs: phoenix6.configs.TalonFXConfiguration
        :param num_config_attempts: Number of times to attempt to configure each device
        :type num_config_attempts: int
        """

        # Note to Riley:
        # The reason we need Subsystem.__init__ is because 
        # it needs to know the parent class's setup code and 
        # that commands 2 knows that it exists
        Subsystem.__init__(self) 

        # Objects that we need
        MOTOROUTPUT = phoenix6.configs.config_groups.MotorOutputConfigs()
        INVERTED = phoenix6.signals.spn_enums.InvertedValue(1)
        MOTOROUTPUT.inverted = INVERTED
        
        # Create motors
        self.intake_wheel = TalonFXS(intake_wheel_id, canbus)
        # self.wheel_config.motor_output = MOTOROUTPUT
        self.intake_arm = TalonFX(intake_arm_id, canbus)
        #TODO Add encoder for arm

        # Apply motor configs
        self._configure_device(self.intake_wheel, intake_wheel_configs, num_config_attempts)
        self._configure_device(self.intake_arm, intake_arm_configs, num_config_attempts)

        # Create PID control requests
        # self.velocity_voltage_request = VelocityVoltage(velocity = 0)
        # self.position_voltage_request = PositionVoltage(position = 0)

    def _configure_device(self, device: TalonFX | TalonFXS, 
                          configs: TalonFXConfiguration | TalonFXSConfiguration, 
                          num_attempts: int) -> None:
        """
        Configures a CTRE motor controller with the specified configs, 
        retrying up to num_attempts times if the configuration fails.
        
        :param device: The CTRE motor controller to configure
        :type device: phoenix6.hardware.TalonFX | phoenix6.hardware.TalonFXS
        :param configs: The configuration to apply to the device
        :type configs: phoenix6.configs.TalonFXConfiguration | phoenix6.configs.TalonFXSConfiguration
        :param num_attempts: Number of times to attempt to configure each device
        :type num_attempts: int
        """
        for _ in range(num_attempts):
            status_code: StatusCode = device.configurator.apply(configs)
            if status_code.is_ok():
                break
        if not status_code.is_ok():
            PrintCommand(f"Device with CAN ID {device.device_id} failed to config with error: {status_code.name}").schedule()
        
    # def intake(self, intake_speed):
    #     self.intake_wheel.set(intake_speed)

    def intake_running(self):
        self.intake_wheel.set(0.5) # Set to 50% power, can be changed later
            # print("Intake is running!!!")
        print("intake wheels are running")

    def intake_stopped(self):
        self.intake_wheel.set(0) # Stop the motor
        print("intake wheels are stopped")

    def arm_down(self):
        self.intake_arm.set(.5)
        print("arm is going to the floor")
    
    def arm_up(self):
        self.intake_arm.set(-.5)
        print("arm is going up")
    
    def arm_stopped(self):
        self.intake_arm.set(0)
        print("arm is stopped")

    
            

