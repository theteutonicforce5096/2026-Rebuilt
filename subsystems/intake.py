from commands2 import PrintCommand, Subsystem, SequentialCommandGroup, WaitCommand, RepeatCommand, ParallelCommandGroup
from phoenix6 import CANBus
from phoenix6.configs import CANcoderConfiguration, TalonFXSConfiguration, TalonFXConfiguration
from phoenix6.controls import PositionVoltage, VoltageOut
from phoenix6.hardware import CANcoder, TalonFXS, TalonFX
from phoenix6.status_code import StatusCode
from wpilib import RobotBase, SmartDashboard
from wpilib import Timer
import wpilib

class Intake(Subsystem):
    """
    Class for controlling intake.
    """

    def __init__(self, canbus: CANBus, intake_wheel_id: int, intake_arm_id: int, 
                 intake_arm_encoder_id: int, intake_wheel_configs: TalonFXSConfiguration, 
                 intake_arm_configs: TalonFXConfiguration, intake_arm_encoder_configs: CANcoderConfiguration,
                 num_config_attempts: int, intake_position: float, stowed_position: float, shooting_position: float, 
                 stall_current_threshold: float, stall_velocity_threshold: float,
                 stall_time_threshold: float,
                 arm_movement_pathway_low: float, arm_movement_pathway_high: float,
                 obstruction_current_threshold: float):
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
        :param stall_current_threshold: Minimum stator current required to detect a stall
        :type stall_current_threshold: float
        :param stall_velocity_threshold: Maximum arm velocity required to detect a stall
        :type stall_velocity_threshold: float
        :param stall_time_threshold: Time required for stall conditions to be met to determine a stall has occured
        :type stall_time_threshold: float
        :param arm_movement_pathway_low: Low position in the arm movement pathway
        :type arm_movement_pathway_low: float
        :param arm_movement_pathway_high: High position in the arm movement pathway
        :type arm_movement_pathway_high: float
        :param obstruction_current_threshold: Minimum stator current required to detect an obstruction
        :type obstruction_current_threshold: float
        """

        Subsystem.__init__(self) 
        
        # Create motors
        self.intake_wheel = TalonFXS(intake_wheel_id, canbus)
        self.intake_arm = TalonFX(intake_arm_id, canbus)
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
        self.shooting_position = shooting_position

        self.arm_movement_pathway_low = arm_movement_pathway_low
        self.arm_movement_pathway_high = arm_movement_pathway_high
        self.obstruction_current_threshold = obstruction_current_threshold

        self.set_position = None
        self.is_stalled = None

        self.stall_current_threshold = stall_current_threshold
        self.stall_velocity_threshold = stall_velocity_threshold
        self.stall_time_threshold = stall_time_threshold
        self.stall_timer = Timer()
        # self.is_stalled = False
        self.last_command_output = 0.0
        self.last_time = 0.0

    def periodic(self):
        """
        Current and velocity for stall detection
        """
        self.now = wpilib.getTime()
        self.dt = self.now - self.last_time
        self.last_time = self.now
        self.current = self.intake_arm.get_stator_current().value_as_double
        self.velocity = abs(self.intake_arm.get_velocity().value_as_double)
        self.intake_arm_now = self.intake_arm.get_position().value_as_double
        # """
        # Publish the current intake wheel voltage for driver-station debugging.
        # """
        # intake_wheel_voltage = self.intake_wheel.get_motor_voltage().value_as_double
        # SmartDashboard.putNumber("Intake Status", intake_wheel_voltage)

    def _configure_device(self, device: TalonFX | TalonFXS | CANcoder, 
                          configs: TalonFXConfiguration | TalonFXSConfiguration | CANcoderConfiguration, 
                          num_attempts: int) -> None:
        """
        Configures a CTRE motor controller or CANcoder with the specified configs, 
        retrying up to num_attempts times if the configuration fails.
        
        :param device: The CTRE motor controller or CANcoder to configure
        :type device: phoenix6.hardware.TalonFX | phoenix6.hardware.TalonFXS | phoenix6.hardware.CANcoder
        :param configs: The configuration to apply to the device
        :type configs: phoenix6.configs.TalonFXConfiguration | phoenix6.configs.TalonFXSConfiguration | 
        phoenix6.configs.CANcoderConfiguration
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
        """
        Apply the requested intake-wheel voltage.

        :param intake_wheel_volts: Voltage to apply to the intake wheel motor.
        :type intake_wheel_volts: float
        """
        self.intake_wheel.set_control(
            self.voltage_request.with_output(intake_wheel_volts)
        )

    def run_intake_wheel(self, intake_wheel_volts):
        """
        Build a one-shot command that applies intake wheel voltage.

        :param intake_wheel_volts: Voltage to apply to the intake wheel motor.
        :type intake_wheel_volts: float
        :returns: Command that applies intake wheel voltage once.
        :rtype: commands2.Command
        """
        return self.runOnce(
            lambda: self.set_intake_speed(intake_wheel_volts)
        )

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

        # print(f"set position: {position}")
        # print(f"current position: {self.intake_arm_now}")
        # print(f"voltage: {self.intake_arm.get_motor_voltage()}")
        # print(f"stator current: {self.intake_arm.get_stator_current()}")

    def detect_arm_movement_completion(self):
        return self.intake_arm.get_position().is_near(self.set_position, .009) or self.is_stalled

    def set_arm_voltage(self, voltage):
        self.intake_arm.set_control(
            self.voltage_request.with_output(voltage)
        )

        # print("voltage request sent")
        # print(f"current position: {self.intake_arm_now}")
        # print(f"voltage: {self.intake_arm.get_motor_voltage()}")
        # print(f"stator current: {self.intake_arm.get_stator_current()}")

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

    def get_stall_detection(self):
        is_commanding_motion = abs(self.set_position - self.intake_arm_now) > .009 # Should be False
        
        if .3 < self.intake_arm_now < .35 or .37 < self.intake_arm_now < .5:
            if self.current > 6.7:
                self.set_arm_voltage(0)
                self.is_stalled = True
                print("OBSTRUCTION !!!@1!!!1111")
                return self.is_stalled

        stall_condition_met = (
            is_commanding_motion
            and self.current > self.stall_current_threshold 
            and abs(self.velocity) < self.stall_velocity_threshold
        )

        if self.set_position is None:
                return

        if stall_condition_met == True:
            self.stall_timer.start()
            # print("stall condition met")

        else:
            self.stall_timer.stop()
            self.stall_timer.reset()

        self.is_stalled = self.stall_timer.hasElapsed(self.stall_time_threshold) and stall_condition_met

        if self.is_stalled == True:
            self.set_arm_voltage(0)
            # self.set_setpoint(self.intake_arm_now)
            print("arm stopped")

             # set the setpoint to the current position to the position that it's at RIGHT NOW

        return self.is_stalled
            
