from commands2 import Subsystem
from commands2 import ParallelCommandGroup, SequentialCommandGroup, WaitCommand, PrintCommand, WaitUntilCommand

from phoenix6 import CANBus
from phoenix6.configs import TalonFXConfiguration, TalonFXSConfiguration, CANcoderConfiguration
from phoenix6.hardware import TalonFX, TalonFXS, CANcoder
from phoenix6.controls import VelocityVoltage, VoltageOut
from phoenix6.status_code import StatusCode

from ntcore import NetworkTableInstance

class Shooter(Subsystem):
    """
    Class for controlling shooter.
    """

    def __init__(self, canbus: CANBus, flywheel_motor_id: int, flywheel_intake_motor_id: int, 
                 flywheel_encoder_id: int, flywheel_motor_configs: TalonFXSConfiguration, 
                 flywheel_intake_motor_configs: TalonFXConfiguration,
                 flywheel_encoder_configs: CANcoderConfiguration,
                 num_config_attempts: int, flywheel_encoder_vel_update_frequency: float):
        """
        Constructor for initializing shooter using the specified constants.

        :param canbus: CANBus instance that electronics are on
        :type canbus: phoenix6.CANBus
        :param flywheel_motor_id: CAN ID of the flywheel motor
        :type flywheel_motor_id: int
        :param flywheel_intake_motor_id: CAN ID of the flywheel intake motor
        :type flywheel_intake_motor_id: int
        :param flywheel_encoder_id: CAN ID of the flywheel encoder
        :type flywheel_encoder_id: int
        :param flywheel_motor_configs: Configs for the flywheel motor
        :type flywheel_motor_configs: phoenix6.configs.TalonFXSConfiguration
        :param flywheel_intake_motor_configs: Configs for the flywheel intake motor
        :type flywheel_intake_motor_configs: phoenix6.configs.TalonFXConfiguration
        :param flywheel_encoder_configs: Configs for the flywheel encoder
        :type flywheel_encoder_configs: phoenix6.configs.CANcoderConfiguration
        :param num_config_attempts: Number of times to attempt to configure each device
        :type num_config_attempts: int
        :param flywheel_encoder_vel_update_frequency: Update frequency in hertz for the flywheel encoder's 
        velocity measurement; Minimum is 4 hertz
        :type flywheel_encoder_vel_update_frequency: float
        """

        # Initialize parent classes
        Subsystem.__init__(self)

        # Create motors and encoder
        self.flywheel_motor = TalonFXS(flywheel_motor_id, canbus)
        self.flywheel_intake_motor = TalonFX(flywheel_intake_motor_id, canbus)
        self.flywheel_encoder = CANcoder(flywheel_encoder_id, canbus)
        
        # Apply motor and encoder configs
        self._configure_device(self.flywheel_motor, flywheel_motor_configs, num_config_attempts)
        self._configure_device(self.flywheel_intake_motor, flywheel_intake_motor_configs, num_config_attempts)
        self._configure_device(self.flywheel_encoder, flywheel_encoder_configs, num_config_attempts)

        # Increase encoder update frequency to 1000 hertz to improve velocity PID on flywheel motor
        self.flywheel_encoder.get_velocity().set_update_frequency(flywheel_encoder_vel_update_frequency)
        
        # Create VelocityVoltage request
        self.velocity_pid_request = VelocityVoltage(velocity = 0)

        self.voltage_request = VoltageOut(output = 0, use_timesync = True).with_update_freq_hz(0.0)
        
        # What to publish over networktables for shooter
        self._network_table_instance = NetworkTableInstance.getDefault()
        
        # Shooter state
        self._shooter_table = self._network_table_instance.getTable("Shooter State")
        self.desired_ball_speed = self._shooter_table.getFloatTopic("Desired Ball Speed in Rotations per Second").publish() 
        self.desired_ball_speed_sub = self._shooter_table.getFloatTopic("Desired Ball Speed in Rotations per Second").subscribe(.5)
        self.desired_ball_speed_sub.get()
        self.desired_flywheel_intake_speed = self._shooter_table.getFloatTopic("Desired Flywheel Intake Speed in Rotations per Second").publish()
        self.desired_flywheel_intake_speed_sub = self._shooter_table.getFloatTopic("Desired Flywheel Intake Speed in Rotations per Second").subscribe(.25)
        self.desired_flywheel_intake_speed_sub.get()
        
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
        
        #TODO is_near is not working well. Likely because of oscilation (fix PID tuning)
    def shoot(self, flywheel_target_velocity, intake_motor_velocity):
        return SequentialCommandGroup(
            self.runOnce(
                lambda: self.flywheel_motor.set_control(
                    self.velocity_pid_request.with_velocity(flywheel_target_velocity)
                )
            ),
            WaitUntilCommand(
                lambda: self.flywheel_motor.get_velocity().is_near(flywheel_target_velocity, 0.25)
            ), 
            self.runOnce(
                lambda: self.set_flywheel_velocities(flywheel_target_velocity, intake_motor_velocity)
            )
        )
    
    def set_flywheel_velocities(self,flywheel_target_velocity, intake_motor_velocity):
        self.flywheel_motor.set_control(
            self.velocity_pid_request.with_velocity(flywheel_target_velocity)
        )
        self.flywheel_intake_motor.set_control(
            self.velocity_pid_request.with_velocity(intake_motor_velocity)
        )
    
    def set_voltage(self, motor: TalonFX | TalonFXS, voltage, duration):
        return self.run(
            lambda: motor.set_control(
                self.voltage_request.with_output(voltage)
            )
        ).withTimeout(duration)
        
    def stop(self):
        SequentialCommandGroup(
            # self.runOnce(lambda: self.flywheel_motor.set_control(
            #     self.velocity_pid_request.with_velocity(target_velocity * .75))),
            # self.runOnce(lambda: self.flywheel_intake_motor.set_control(
            #     self.velocity_pid_request.with_velocity(flywheel_intake_velocity_rps * .75))),
            # WaitCommand(.25),
            # self.runOnce(lambda: self.flywheel_motor.set_control(
            #     self.velocity_pid_request.with_velocity(target_velocity * .5))),
            # self.runOnce(lambda: self.flywheel_intake_motor.set_control(
            #     self.velocity_pid_request.with_velocity(flywheel_intake_velocity_rps * .5))),
            # WaitCommand(.25),
            # self.runOnce(lambda: self.flywheel_motor.set_control(
            #     self.velocity_pid_request.with_velocity(target_velocity * .25))),
            # self.runOnce(lambda: self.flywheel_intake_motor.set_control(
            #     self.velocity_pid_request.with_velocity(flywheel_intake_velocity_rps * .25))),
            # WaitCommand(.25),
            self.runOnce(lambda: self.flywheel_motor.set_control(
                self.velocity_pid_request.with_velocity(0))),
            self.runOnce(lambda: self.flywheel_intake_motor.set_control(
                self.velocity_pid_request.with_velocity(0)))
        ).schedule()

    # def stop_networktable(self):
    #     SequentialCommandGroup(
    #         self.runOnce(lambda: self.flywheel_motor.set_control(
    #             self.velocity_pid_request.with_velocity(self.desired_ball_speed_sub.get() * 113 * .75))),
    #         self.runOnce(lambda: self.flywheel_intake_motor.set_control(
    #             self.velocity_pid_request.with_velocity(self.desired_flywheel_intake_speed_sub.get() * 106 * .75))),
    #         WaitCommand(.25),
    #         self.runOnce(lambda: self.flywheel_motor.set_control(
    #             self.velocity_pid_request.with_velocity(self.desired_ball_speed_sub.get() * 113 * .5))),
    #         self.runOnce(lambda: self.flywheel_intake_motor.set_control(
    #             self.velocity_pid_request.with_velocity(self.desired_flywheel_intake_speed_sub.get() * 106 * .5))),
    #         WaitCommand(.25),
    #         self.runOnce(lambda: self.flywheel_motor.set_control(
    #             self.velocity_pid_request.with_velocity(self.desired_ball_speed_sub.get() * 113 * .25))),
    #         self.runOnce(lambda: self.flywheel_intake_motor.set_control(
    #             self.velocity_pid_request.with_velocity(self.desired_flywheel_intake_speed_sub.get() * 106 * .25))),
    #         WaitCommand(.25),
    #         self.runOnce(lambda: self.flywheel_motor.set_control(
    #             self.velocity_pid_request.with_velocity(0))),
    #         self.runOnce(lambda: self.flywheel_intake_motor.set_control(
    #             self.velocity_pid_request.with_velocity(0)))
    #     ).schedule()
        
