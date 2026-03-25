from typing import Any, Callable, Final

from commands2 import Subsystem
from commands2 import DeferredCommand, PrintCommand, SequentialCommandGroup, WaitUntilCommand

from phoenix6 import CANBus, SignalLogger
from phoenix6.controls import VelocityVoltage, VoltageOut
from phoenix6.configs import CANcoderConfiguration, TalonFXConfiguration, TalonFXSConfiguration
from phoenix6.hardware import CANcoder, TalonFX, TalonFXS
from phoenix6.status_code import StatusCode

from commands2.sysid import SysIdRoutine
from ntcore import NetworkTableInstance
from wpilib import RobotBase, SendableChooser, Timer
from wpilib.shuffleboard import Shuffleboard
from wpilib.sysid import SysIdRoutineLog
from wpimath.geometry import Translation2d
from wpimath.kinematics import ChassisSpeeds

from constants.shot_calculator import ShotCalculator
from constants.shot_calculator_constants import (
    Config as ShotCalculatorConfig,
    LaunchParameters,
    ShotInputs,
    calc_shot_profile,
)

class Shooter(Subsystem):
    """
    Class for controlling shooter.
    """

    DEFAULT_FLYWHEEL_INTAKE_TARGET_RPS: Final[float] = 26.5
    CALCULATED_FEED_CONFIDENCE_THRESHOLD: Final[float] = 50.0

    def __init__(self, canbus: CANBus, flywheel_motor_id: int, flywheel_intake_motor_id: int, 
                 flywheel_encoder_id: int, flywheel_motor_configs: TalonFXSConfiguration, 
                 flywheel_intake_motor_configs: TalonFXConfiguration,
                 flywheel_encoder_configs: CANcoderConfiguration,
                 num_config_attempts: int, flywheel_encoder_vel_update_frequency: float,
                 get_current_swerve_state: Callable[[], Any],
                 get_robot_tilt: Callable[[], tuple[float, float]],
                 get_hub_center: Callable[[], Translation2d],
                 launcher_offset_x: float, launcher_offset_y: float):
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
       
        if RobotBase.isSimulation() == False:
            self.flywheel_motor.optimize_bus_utilization()
            self.flywheel_intake_motor.optimize_bus_utilization()
            self.flywheel_encoder.optimize_bus_utilization()

        # Set encoder update frequency to improve velocity PID on flywheel motor
        self.flywheel_encoder.get_velocity().set_update_frequency(flywheel_encoder_vel_update_frequency)
        
        # Create VelocityVoltage request
        self.velocity_pid_request = VelocityVoltage(velocity = 0)
        self.intake_velocity_pid_request = VelocityVoltage(velocity = 0) 
        # Created a separate PID request because ts never works ^^^

        self.voltage_request = VoltageOut(output = 0)

        # Empty detection variable       
        self.empty_time = None
        self.has_surged = False

        # What to publish over networktables for shooter
        self._network_table_instance = NetworkTableInstance.getDefault()

        #Create SysId routine for characterizing flywheel motor
        self.flywheel_motor_sys_id_routine = SysIdRoutine(
            SysIdRoutine.Config(
                rampRate = 0.5,
                stepVoltage = 9.0,
                timeout = 15.0,
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
                timeout = 15.0,
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
        self.desired_ball_speed_sub = self._shooter_table.getFloatTopic("Desired Ball Speed in Rotations per Second").subscribe(60)
        self.desired_ball_speed_sub.get()
        self.desired_flywheel_intake_speed = self._shooter_table.getFloatTopic("Desired Flywheel Intake Speed in Rotations per Second").publish()
        self.desired_flywheel_intake_speed_sub = self._shooter_table.getFloatTopic("Desired Flywheel Intake Speed in Rotations per Second").subscribe(40)
        self.desired_flywheel_intake_speed_sub.get()

        self._get_current_swerve_state = get_current_swerve_state
        self._get_robot_tilt = get_robot_tilt
        self._get_hub_center = get_hub_center

        self.shot_calculator = ShotCalculator(
            ShotCalculatorConfig(
                launcher_offset_x = launcher_offset_x,
                launcher_offset_y = launcher_offset_y,
            )
        )
        self._latest_calculated_shot: LaunchParameters | None = None

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

    def _build_shot_inputs(self) -> ShotInputs:
        current_state = self._get_current_swerve_state()
        pitch_deg, roll_deg = self._get_robot_tilt()

        return ShotInputs(
            robot_pose = current_state.pose,
            field_velocity = ChassisSpeeds.fromRobotRelativeSpeeds(
                current_state.speeds,
                current_state.pose.rotation()
            ),
            robot_velocity = current_state.speeds,
            hub_center = self._get_hub_center(),
            hub_forward = Translation2d(1.0, 0.0),
            vision_confidence = 1.0,
            pitch_deg = pitch_deg,
            roll_deg = roll_deg,
        )

    def _calculate_launch_parameters(self) -> LaunchParameters | None:
        return self.shot_calculator.calculate(self._build_shot_inputs())

    def _apply_calculated_shot(self):
        self._latest_calculated_shot = self._calculate_launch_parameters()

        flywheel_target_velocity = (
            self._latest_calculated_shot.flywheel_rps
            if self._latest_calculated_shot is not None
            else 0.0
        )
        intake_motor_velocity = (
            self.DEFAULT_FLYWHEEL_INTAKE_TARGET_RPS
            if self._latest_calculated_shot is not None
            else 0.0
        )

        self.set_flywheel_velocities(flywheel_target_velocity, intake_motor_velocity)


    @staticmethod
    def should_feed_calculated_shot(launch_parameters: LaunchParameters | None) -> bool:
        return (
            launch_parameters is not None
            and launch_parameters.confidence >= Shooter.CALCULATED_FEED_CONFIDENCE_THRESHOLD
        )

    def reset_calculated_shot_state(self):
        self.shot_calculator.reset_warm_start()
        self._latest_calculated_shot = None

    def get_latest_calculated_shot(self) -> LaunchParameters | None:
        return self._latest_calculated_shot

    def create_manual_shoot_command(self):
        return DeferredCommand(
            lambda: self.shoot(
                self.desired_ball_speed_sub.get(),
                self.desired_flywheel_intake_speed_sub.get()
            ),
            self
        )

    def get_fixed_distance_shot_targets(self, distance_m: float) -> tuple[float, float]:
        flywheel_target_velocity, _ = calc_shot_profile(distance_m)
        return flywheel_target_velocity, self.DEFAULT_FLYWHEEL_INTAKE_TARGET_RPS

    def create_fixed_distance_shoot_command(self, distance_m: float):
        return self.run(
            lambda: self.set_flywheel_velocities(*self.get_fixed_distance_shot_targets(distance_m))
        ).until(
            lambda: self.flywheel_motor.get_closed_loop_error().is_near(0, 1)
        )

    def create_calculated_shoot_command(self):
        return self.runOnce(
            lambda: self._apply_calculated_shot()
        )

    def create_calculated_feed_command(self, hopper):
        return DeferredCommand(
            lambda: (
                hopper.create_feed_cycle_command()
                if self.should_feed_calculated_shot(self._latest_calculated_shot)
                else hopper.create_stop_command()
            ),
            hopper
        )

    def create_stop_command(self):
        return self.runOnce(
            lambda: self.set_flywheel_velocities(0, 0)
        )
    
#Shoot functions
    def shoot(self, flywheel_target_velocity, intake_motor_velocity):
        return self.runOnce(
            lambda: self.set_flywheel_velocities(flywheel_target_velocity, intake_motor_velocity)
        )
    
    def set_flywheel_velocities(self, flywheel_target_velocity, intake_motor_velocity):
        self.flywheel_motor.set_control(
            self.velocity_pid_request.with_velocity(flywheel_target_velocity)
        )
        self.flywheel_intake_motor.set_control(
            self.intake_velocity_pid_request.with_velocity(intake_motor_velocity)
        )
        # Okay changed this to the seperate PID request crossing finguers
    
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

#Detect amp surge functions (to determine if the hopper is empty)
    def detect_empty(self):
        if self.flywheel_intake_motor.get_closed_loop_error().is_near(0, 1) == True:
            self.surging = False

        elif self.flywheel_intake_motor.get_closed_loop_error().is_near(0, 1) == False:
            self.surging = True

        if self.surging == True:
            self.empty_time = Timer.getFPGATimestamp()
            # every time there is an amp surge, the empty_time timestamp resets
            # when no surge occurs, empty_time stops updating
            # So, empty_time = timestamp of last surge
            # We detect that the hopper is empty when its been over 3 seconds since the last surge
    
        return (
            (Timer.getFPGATimestamp() - self.empty_time) >= 3
        )
    
    def reset_empty_time(self):
        self.empty_time = Timer.getFPGATimestamp()
        self.surging = True

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
        
