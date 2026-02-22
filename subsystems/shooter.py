from commands2 import Subsystem
from commands2 import SequentialCommandGroup, WaitUntilCommand, WaitCommand

from phoenix6 import CANBus
from phoenix6.configs import TalonFXConfiguration, TalonFXSConfiguration, CANcoderConfiguration
from phoenix6.hardware import TalonFX, TalonFXS, CANcoder
from phoenix6.controls import VelocityVoltage

from ntcore import NetworkTableInstance

class Shooter(Subsystem):
    """
    Class for controlling shooter.
    """

    def __init__(self, canbus: CANBus, flywheel_motor_id: int, flywheel_intake_motor_id: int, 
                 flywheel_encoder_id: int, flywheel_motor_configs: TalonFXSConfiguration, 
                 flywheel_intake_motor_configs: TalonFXConfiguration,
                 flywheel_encoder_configs: CANcoderConfiguration):
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
        """

        # Initialize parent classes
        Subsystem.__init__(self)

        # Create motors and encoder
        self.flywheel_motor = TalonFXS(flywheel_motor_id, canbus)
        self.flywheel_intake_motor = TalonFX(flywheel_intake_motor_id, canbus)
        self.flywheel_encoder = CANcoder(flywheel_encoder_id, canbus)
        
        # Apply motor and encoder configs
        self.flywheel_motor.configurator.apply(flywheel_motor_configs)
        self.flywheel_intake_motor.configurator.apply(flywheel_intake_motor_configs)
        self.flywheel_encoder.configurator.apply(flywheel_encoder_configs)
        
        # Create VelocityVoltage request
        self.velocity_pid_request = VelocityVoltage(velocity = 0)
        
        #TODO Network Table Stuffs
        
        # What to publish over networktables for shooter
        self._network_table_instance = NetworkTableInstance.getDefault()
        
        # Shooter state
        self._shooter_table = self._network_table_instance.getTable("Shooter State")
        self.desired_ball_speed = self._shooter_table.getFloatTopic("Desired Ball Speed (percent)").publish() 
        self.desired_ball_speed_sub = self._shooter_table.getFloatTopic("Desired Ball Speed (percent)").subscribe(.5)
        self.desired_ball_speed_sub.get()
        self.desired_flywheel_intake_speed = self._shooter_table.getFloatTopic("Desired Flywheel Intake Speed (percent)").publish()
        self.desired_flywheel_intake_speed_sub = self._shooter_table.getFloatTopic("Desired Flywheel Intake Speed (percent)").subscribe(.5)
        self.desired_flywheel_intake_speed_sub.get()
        
#TODO How should the flywheel intake motor be run? (value in physics file)
    def shoot(self, target_velocity, flywheel_intake_velocity_rps):
        SequentialCommandGroup(
            self.runOnce(lambda: self.flywheel_motor.set_control(
                self.velocity_pid_request.with_velocity(target_velocity)
            )),
            WaitUntilCommand(lambda: self.flywheel_motor.get_velocity().is_near(target_velocity, 0.25)),
            self.runOnce(lambda: self.flywheel_intake_motor.set_control(
                self.velocity_pid_request.with_velocity(flywheel_intake_velocity_rps)
            ))
        ).schedule()
        
    def stop(self, target_velocity, flywheel_intake_velocity_rps):
        SequentialCommandGroup(
            self.runOnce(lambda: self.flywheel_motor.set_control(
                self.velocity_pid_request.with_velocity(target_velocity * .75))),
            self.runOnce(lambda: self.flywheel_intake_motor.set_control(
                self.velocity_pid_request.with_velocity(flywheel_intake_velocity_rps * .75))),
            WaitCommand(.25),
            self.runOnce(lambda: self.flywheel_motor.set_control(
                self.velocity_pid_request.with_velocity(target_velocity * .5))),
            self.runOnce(lambda: self.flywheel_intake_motor.set_control(
                self.velocity_pid_request.with_velocity(flywheel_intake_velocity_rps * .5))),
            WaitCommand(.25),
            self.runOnce(lambda: self.flywheel_motor.set_control(
                self.velocity_pid_request.with_velocity(target_velocity * .25))),
            self.runOnce(lambda: self.flywheel_intake_motor.set_control(
                self.velocity_pid_request.with_velocity(flywheel_intake_velocity_rps * .25))),
            WaitCommand(.25),
            self.runOnce(lambda: self.flywheel_motor.set_control(
                self.velocity_pid_request.with_velocity(0))),
            self.runOnce(lambda: self.flywheel_intake_motor.set_control(
                self.velocity_pid_request.with_velocity(0)))
        ).schedule()
        
        # self.flywheel_motor.set(0)
        # self.flywheel_intake_motor.set(0)


        
