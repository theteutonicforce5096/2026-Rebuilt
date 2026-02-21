from commands2 import Subsystem

from phoenix6.configs import TalonFXConfiguration, TalonFXSConfiguration, CANcoderConfiguration
from phoenix6.hardware import TalonFX, TalonFXS, CANcoder
from phoenix6.controls import VelocityTorqueCurrentFOC

class Shooter(Subsystem):
    """
    Class for controlling shooter.
    """

    def __init__(self, flywheel_motor_id: int, flywheel_intake_motor_id: int, flywheel_encoder_id: int,
                 flywheel_motor_configs: TalonFXSConfiguration, 
                 flywheel_intake_motor_configs: TalonFXConfiguration,
                 flywheel_encoder_configs: CANcoderConfiguration):
        """
        Constructor for initializing shooter using the specified constants.

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
        self.flywheel_motor = TalonFXS(flywheel_motor_id)
        self.flywheel_intake_motor = TalonFX(flywheel_intake_motor_id)
        self.flywheel_encoder = CANcoder(flywheel_encoder_id)
        
        # Apply motor and encoder configs
        self.flywheel_motor.configurator.apply(flywheel_motor_configs)
        self.flywheel_intake_motor.configurator.apply(flywheel_intake_motor_configs)
        self.flywheel_encoder.configurator.apply(flywheel_encoder_configs)
        
        # Create Velocity TorqueCurrentFOC request
        self.velocity_pid_request = VelocityTorqueCurrentFOC(velocity = 0)
        
#testing
    def run_intake(self):
        self.flywheel_motor.set(.1)
        self.flywheel_intake_motor.set(.5)
        
    def stop_intake(self):
        self.flywheel_motor.set(0)
        self.flywheel_intake_motor.set(0)
        
#TODO How should the flywheel intake motor be run? (value in physics file)
    def shoot(self, initial_velocity_rps, flywheel_intake_velocity_rps):
        self.flywheel_motor.set_control(
            self.velocity_pid_request.with_velocity(initial_velocity_rps)
        )
        self.flywheel_intake_motor.set_control(
            self.velocity_pid_request.with_velocity(flywheel_intake_velocity_rps)
        )

        
    def stop(self):
        self.flywheel_motor.set_control(self.flywheel_velocity_torque.with_velocity(0))
        self.flywheel_intake_motor.set_control(self.flywheel_intake_velocity_torque.with_velocity(0))
        # self.flywheel_motor.set(0)
        # self.flywheel_intake_motor.set(0)


        
