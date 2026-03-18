"""
TODO: I am making the hopper and the intake two seperate things so it will make everything easier
"""
from commands2 import PrintCommand, SequentialCommandGroup, Subsystem
from commands2.sysid import SysIdRoutine

from phoenix6 import CANBus, SignalLogger
from phoenix6.configs import TalonFXConfiguration
from phoenix6.controls import VelocityVoltage, VoltageOut
from phoenix6.hardware import TalonFX
from phoenix6.status_code import StatusCode

from wpilib.sysid import SysIdRoutineLog

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

    def __init__(self, canbus: CANBus, mechanim_wheel_id: int, agitator_wheel_id: int, 
                 mechanim_wheel_configs: TalonFXConfiguration, 
                 agitator_wheel_configs: TalonFXConfiguration,
                 num_config_attempts: int):
        """
        Constructor for initializing hopper using the specified constants.

        :param canbus: CANBus instance that electronics are on
        :type canbus: phoenix6.CANBus
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

        # Create motors
        self.mechanim_wheel = TalonFX(mechanim_wheel_id, canbus) #the mechanim wheels
        self.agitator_wheel = TalonFX(agitator_wheel_id, canbus) #wheels in the hopper
        
        # Apply motor configs
        self._configure_device(self.mechanim_wheel, mechanim_wheel_configs, num_config_attempts)
        self._configure_device(self.agitator_wheel, agitator_wheel_configs, num_config_attempts)

        # Create VelocityVoltage request
        self.velocity_pid_request = VelocityVoltage(velocity = 0)
        self.voltage_request = VoltageOut(output = 0)

        # Create SysId routine for characterizing the mechanim wheel motor.
        self.mechanim_motor_sys_id_routine = SysIdRoutine(
            SysIdRoutine.Config(
                rampRate = 0.5,
                stepVoltage = 9.0,
                timeout = 15.0,
                recordState = lambda state: SignalLogger.write_string(
                    "SysId_Hopper_Mechanim_Motor_State",
                    SysIdRoutineLog.stateEnumToString(state)
                )
            ),
            SysIdRoutine.Mechanism(
                lambda output: self.mechanim_wheel.set_control(
                    self.voltage_request.with_output(output)
                ),
                lambda log: None,
                self,
            ),
        )

        self.sys_id_routine_to_apply = self.mechanim_motor_sys_id_routine
        
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

    def set_sys_id_routine(self):
        """
        Set the SysId routine to run. Hopper currently exposes only the mechanim wheel routine.
        """
        self.sys_id_routine_to_apply = self.mechanim_motor_sys_id_routine

    def sys_id_quasistatic(self, direction: SysIdRoutine.Direction):
        """
        Runs the SysId Quasistatic test for the mechanim wheel motor.

        :param direction: Direction of the SysId Quasistatic test
        :type direction: SysIdRoutine.Direction
        """
        return self.sys_id_routine_to_apply.quasistatic(direction)

    def sys_id_dynamic(self, direction: SysIdRoutine.Direction):
        """
        Runs the SysId Dynamic test for the mechanim wheel motor.

        :param direction: Direction of the SysId Dynamic test
        :type direction: SysIdRoutine.Direction
        """
        return self.sys_id_routine_to_apply.dynamic(direction)

    def hop(self, mechanim_speed, agitator_speed):
        return SequentialCommandGroup(
            self.runOnce(
                lambda: self.mechanim_wheel.set_control(
                    self.velocity_pid_request.with_velocity(mechanim_speed)
                )
            ),
            self.runOnce(
                lambda: self.agitator_wheel.set_control(
                    self.velocity_pid_request.with_velocity(agitator_speed)
                )
            )
        )

    # def set_mecanum_speed(self, speed):
    #     self.mechanim_wheel.set(speed)

    # def set_agitator_speed(self, speed):
    #     self.agitator_wheel.set(speed)


