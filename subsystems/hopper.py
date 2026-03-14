'''
TODO: I am making the hopper and the intake two seperate things so it will make everything easier
'''
import wpilib
import phoenix6
import phoenix6.controls
import phoenix6.configs
import phoenix6.signals
import commands2
from commands2 import Subsystem
from commands2.cmd import print_

from commands2 import PrintCommand

from phoenix6.configs import TalonFXConfiguration
from phoenix6.hardware import TalonFX
from phoenix6.status_code import StatusCode

from ntcore import NetworkTableInstance
from wpilib.shuffleboard import Shuffleboard

""" 
TODO: 
- It would be beneficial if we had an order to when which motors turn on so nothing gets stuck
1. Shooter Intake
2. Mechanim wheels
- We need two motors, both motors are talon FX, not sure there would need ot be very much tuning
"""

class Hopper(Subsystem): # <-- Telling subsystem that its part of it too
    """
    Class for controlling hopper.
    """

    def __init__(self, mechanim_wheel_id: int, agitator_wheel_id: int, 
                 mechanim_wheel_configs: TalonFXConfiguration, 
                 agitator_wheel_configs: TalonFXConfiguration,
                 num_config_attempts: int):
        """
        Constructor for initializing hopper using the specified constants.

        :param mechanim_wheel_id: CAN ID of the mechanim wheel
        :type mechanim_wheel_id: int
        :param agitator_wheel_id: CAN ID of the agitator wheel
        :type agitator_wheel_id: int
        :param mechanim_wheel_configs: Configs for the mechanim wheel
        :type mechanim_wheel_configs: phoenix6.configs.TalonFXConfiguration
        :param agitator_wheel_configs: Configs for the agitator wheel
        :type agitator_wheel_configs: phoenix6.configs.TalonFXConfiguration
        :param num_config_attempts: Number of times to attempt to configure each device
        :type num_config_attempts: int
        """

        Subsystem.__init__(self)

        # Jay is it okay if we make the canbus an object 
        # CONSTANTS
        CANBUS = phoenix6.CANBus.roborio()
        
        #TODO i think we added more motors or something
        # Create motors
        self.mechanim_wheel = TalonFX(mechanim_wheel_id, CANBUS) #the mechanim wheels
        self.agitator_wheel = TalonFX(agitator_wheel_id, CANBUS) #wheels in the hopper
        
        # Apply motor configs
        self._configure_device(self.mechanim_wheel, mechanim_wheel_configs, num_config_attempts)
        self._configure_device(self.agitator_wheel, agitator_wheel_configs, num_config_attempts)
        
    def _configure_device(self, device: TalonFX, 
                          configs: TalonFXConfiguration, 
                          num_attempts: int) -> None:
        """
        Configures a CTRE motor controller with the specified configs, 
        retrying up to num_attempts times if the configuration fails.
        
        :param device: The CTRE motor controller to configure
        :type device: phoenix6.hardware.TalonFX 
        :param configs: The configuration to apply to the device
        :type configs: phoenix6.configs.TalonFXConfiguration 
        :param num_attempts: Number of times to attempt to configure each device
        :type num_attempts: int
        """
        for _ in range(num_attempts):
            status_code: StatusCode = device.configurator.apply(configs)
            if status_code.is_ok():
                break
        if not status_code.is_ok():
            PrintCommand(f"Device with CAN ID {device.device_id} failed to config with error: {status_code.name}").schedule()
        
        
    def mechanim_on(self, speed):
        self.mechanim_wheel.set(speed)
        print("mechanim wheels are moving")
    
    def mechanim_off(self, speed):
        self.mechanim_wheel.set(speed)
        print("mechanim wheels are not moving")

    def agitator_on(self, speed):
        self.agitator_wheel.set(speed)
        print("agitator moving")
    
    def agitator_off(self, speed):
        self.agitator_wheel.set(speed)
        print("agitator is stopped")



