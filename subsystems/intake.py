from commands2 import PrintCommand, Subsystem, SequentialCommandGroup, WaitCommand, RepeatCommand, ParallelCommandGroup
from phoenix6 import CANBus
from phoenix6.configs import TalonFXSConfiguration
from phoenix6.controls import PositionVoltage, VoltageOut
from phoenix6.hardware import TalonFXS
from phoenix6.status_code import StatusCode
from wpilib import RobotBase
from wpilib import Timer
import wpilib


class Intake(Subsystem):
    """
    Class for controlling intake.
    """

    def __init__(self, canbus: CANBus, intake_arm_id: int, 
                 intake_arm_configs: TalonFXSConfiguration,
                 num_config_attempts: int, intake_position: float, stowed_position: float,
                 stall_current_threshold: float, stall_velocity_threshold: float,
                 stall_time_threshold: float):
        """
        Constructor for initializing shooter using the specified constants.

        :param canbus: CANBus instance that electronics are on
        :type canbus: phoenix6.CANBus
        :param intake_arm_id: CAN ID of the intake arm
        :type intake_arm_id: int
        :param intake_arm_configs: Configs for the intake arm
        :type intake_arm_configs: phoenix6.configs.TalonFXSConfiguration
        :param num_config_attempts: Number of times to attempt to configure each device
        :type num_config_attempts: int
        :param intake_position: Encoder position where arm is down
        :type intake_position: float
        :param stowed_position: Encoder position where arm is up
        :type stowed_position: float
        """

        Subsystem.__init__(self) 
        
        # Create motors
        self.intake_arm = TalonFXS(intake_arm_id, canbus)

        # Apply motor configs
        self._configure_device(self.intake_arm, intake_arm_configs, num_config_attempts)
        if RobotBase.isSimulation() == False:
            self.intake_arm.optimize_bus_utilization()

        # Create PID control requests
        self.voltage_request = VoltageOut(output = 0)
        self.arm_voltage_request = VoltageOut(output = 0)
        self.position_voltage_request = PositionVoltage(position = 0)
        # Placeholder values, will need to be tuned

        # Arm Positions because apparently we need those
        self.intake_position = intake_position
        self.stowed_position = stowed_position


        self.stall_current_threshold = stall_current_threshold
        self.stall_velocity_threshold = stall_velocity_threshold
        self.stall_time_threshold = stall_time_threshold
        self.stall_timer = Timer()
        self.is_stalled = False
        self.last_command_output = 0.0
        self.last_time = 0.0

    def periodic(self):
        """
        Current and velocity for stall detection
        """
        self.now = wpilib.getTime()
        self.dt = self.now - self.last_time
        self.last_time = self.now
        self.current = self.intake_arm.get_motor_stall_current()
        self.velocity = self.intake_arm.get_velocity()
        self.intake_arm_now = self.intake_arm.get_position()

        print(self.intake_arm_now)

    def _configure_device(self, device: TalonFXS , 
                          configs: TalonFXSConfiguration , 
                          num_attempts: int) -> None:
        """
        Configures a CTRE motor controller or CANcoder with the specified configs, 
        retrying up to num_attempts times if the configuration fails.
        
        :param device: The CTRE motor controller or CANcoder to configure
        :type device: phoenix6.hardware.phoenix6.hardware.TalonFXS 
        :param configs: The configuration to apply to the device
        :type configs: phoenix6.configs.phoenix6.configs.TalonFXSConfiguration 
        :param num_attempts: Number of times to attempt to configure each device
        :type num_attempts: int
        """
        for _ in range(num_attempts):
            status_code: StatusCode = device.configurator.apply(configs)
            if status_code.is_ok():
                break
        if not status_code.is_ok():
            PrintCommand(f"Device with CAN ID {device.device_id} failed to config with error: {status_code.name}").schedule()


    def set_setpoint(self, position):
        """
        Command the intake arm to the requested closed-loop position.

        :param position: Desired intake arm position in mechanism rotations.
        :type position: float
        """
        self.set_position = position

        self.intake_arm.set_control(
            self.position_voltage_request.with_position(position)
        )

        # print(position)

    
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
        

    def get_stall_detection(self):
        is_commanding_motion = abs(self.set_position - self.intake_arm_now) > .05 # Should be False


        stall_condition_met = (
            is_commanding_motion
            and self.current > self.stall_current_threshold
            and abs(self.velocity) < self.stall_velocity_threshold
        )

        if self.set_position is None:
                return

        if stall_condition_met:
            self.stall_timer.start()

        else:
            self.stall_timer.stop()
            self.stall_timer.reset()

        self.is_stalled = self.stall_timer.hasElapsed(self.stall_time_threshold) and stall_condition_met

        if self.is_stalled:
            self.set_setpoint(self.intake_arm_now)

             # set the setpoint to the current position to the position that it's at RIGHT NOW

        return self.is_stalled