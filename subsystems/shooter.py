from commands2 import Subsystem
from commands2 import ParallelCommandGroup, SequentialCommandGroup, WaitCommand, PrintCommand, WaitUntilCommand

from phoenix6 import CANBus, SignalLogger
from phoenix6.configs import TalonFXConfiguration, TalonFXSConfiguration, CANcoderConfiguration
from phoenix6.hardware import TalonFX, TalonFXS, CANcoder
from phoenix6.controls import VelocityVoltage, VoltageOut
from phoenix6.status_code import StatusCode

from commands2.sysid import SysIdRoutine 
from wpilib.sysid import SysIdRoutineLog
from wpilib import SendableChooser

from ntcore import NetworkTableInstance
from wpilib.shuffleboard import Shuffleboard

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

        self.voltage_request = VoltageOut(output = 0)
        
        # What to publish over networktables for shooter
        self._network_table_instance = NetworkTableInstance.getDefault()

        #Create SysId routine for characterizing flywheel motor
        self.flywheel_motor_sys_id_routine = SysIdRoutine(
            SysIdRoutine.Config(
                rampRate = 0.5,
                stepVoltage = 9.0,
                timeout = 30.0,
                recordState = lambda state: SignalLogger.write_string(
                    "SysId_Flywheel_Motor_State", SysIdRoutineLog.stateEnumToString(state)
                )
            ),
            SysIdRoutine.Mechanism(
                lambda output: self.flywheel_motor.set_control(self.voltage_request.with_output(output)),
                lambda log: None,
                self,
            ),
        )

        #Create SysId routine for characterizing flywheel intake motor
        self.flywheel_intake_motor_sys_id_routine = SysIdRoutine(
            SysIdRoutine.Config(
                rampRate = 0.5,
                stepVoltage = 9.0,
                timeout = 30.0,
                recordState = lambda state: SignalLogger.write_string(
                    "SysId_Flywheel_Intake_Motor_State", SysIdRoutineLog.stateEnumToString(state)
                )
            ),
            SysIdRoutine.Mechanism(
                lambda output: self.flywheel_intake_motor.set_control(self.voltage_request.with_output(output)),
                lambda log: None,
                self,
            ),
        )

        # Create widget for selecting SysId routine and set default value
        self.sys_id_routine_to_apply = self.flywheel_motor_sys_id_routine
        self.sys_id_routines = SendableChooser()
        self.sys_id_routines.setDefaultOption("Flywheel Motor Routine", self.flywheel_motor_sys_id_routine)
        self.sys_id_routines.addOption("Flywheel Intake Motor Routine", self.flywheel_intake_motor_sys_id_routine)

        # Send widget to Shuffleboard 
        Shuffleboard.getTab("SysId").add(f"Routines", self.sys_id_routines).withSize(2, 1)

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
        
    def set_sys_id_routine(self):
        """
        Set the SysId Routine to run based off of the routine chosen in Shuffleboard.
        """
        self.sys_id_routine_to_apply = self.sys_id_routines.getSelected()

    def sys_id_quasistatic(self, direction: SysIdRoutine.Direction):
        """
        Runs the SysId Quasistatic test in the given direction for the routine specified by self.sys_id_routine_to_apply.

        :param direction: Direction of the SysId Quasistatic test
        :type direction: SysIdRoutine.Direction
        """
        return self.sys_id_routine_to_apply.quasistatic(direction)

    def sys_id_dynamic(self, direction: SysIdRoutine.Direction):
        """
        Runs the SysId Dynamic test in the given direction for the routine specified by self.sys_id_routine_to_apply.

        :param direction: Direction of the SysId Dynamic test
        :type direction: SysIdRoutine.Direction
        """
        return self.sys_id_routine_to_apply.dynamic(direction)
    
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
    #             self.velocity_pid_request.with_velocity(self.desired_ball_speed_sub.get() * .75))),
    #         self.runOnce(lambda: self.flywheel_intake_motor.set_control(
    #             self.velocity_pid_request.with_velocity(self.desired_flywheel_intake_speed_sub.get() * .75))),
    #         WaitCommand(.25),
    #         self.runOnce(lambda: self.flywheel_motor.set_control(
    #             self.velocity_pid_request.with_velocity(self.desired_ball_speed_sub.get() * .5))),
    #         self.runOnce(lambda: self.flywheel_intake_motor.set_control(
    #             self.velocity_pid_request.with_velocity(self.desired_flywheel_intake_speed_sub.get() * .5))),
    #         WaitCommand(.25),
    #         self.runOnce(lambda: self.flywheel_motor.set_control(
    #             self.velocity_pid_request.with_velocity(self.desired_ball_speed_sub.get() * .25))),
    #         self.runOnce(lambda: self.flywheel_intake_motor.set_control(
    #             self.velocity_pid_request.with_velocity(self.desired_flywheel_intake_speed_sub.get() * .25))),
    #         WaitCommand(.25),
    #         self.runOnce(lambda: self.flywheel_motor.set_control(
    #             self.velocity_pid_request.with_velocity(0))),
    #         self.runOnce(lambda: self.flywheel_intake_motor.set_control(
    #             self.velocity_pid_request.with_velocity(0)))
    #     ).schedule()
        
