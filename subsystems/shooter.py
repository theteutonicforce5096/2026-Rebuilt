from commands2 import Subsystem

from constants.shooter_constants import ShooterConstants

from phoenix6.hardware import TalonFXS, CANcoder
from phoenix6.hardware import TalonFX
from phoenix6.controls import VelocityTorqueCurrentFOC

class Shooter(Subsystem):
    def __init__(self):
        Subsystem.__init__(self)
        
        self.flywheel_motor = TalonFXS(ShooterConstants.FLYWHEEL_CAN_ID)
        self.flywheel_intake_motor = TalonFX(ShooterConstants.FLYWHEEL_INTAKE_CAN_ID)
        #self.shoot_motor_encoder = CANcoder()
        
    #Apply configurations from Shooter Constants
        self.flywheel_motor.configurator.apply(ShooterConstants.talonfxs_configs)
        self.flywheel_intake_motor.configurator.apply(ShooterConstants.talonfx_configs)
        
    #Velocity PID Control
        self.flywheel_velocity_torque = VelocityTorqueCurrentFOC(0).with_slot(0)
        
        # self.controller = self.motor.getClosedLoopController()
        # self.controller.setSetpoint()
    
        
        # self.motor_config.closedLoop.P()
        
#testing
    def run_intake(self):
        self.flywheel_motor.set(.1)
        self.flywheel_intake_motor.set(.5)
        
    def stop_intake(self):
        self.flywheel_motor.set(0)
        self.flywheel_intake_motor.set(0)
        
        
#TODO How should the flywheel intake motor be run?
    def calculated_shot(self, initial_velocity):
        self.flywheel_motor.set_control(self.flywheel_velocity_torque.with_velocity(initial_velocity))
        
    def test_shot(self, test_velocity):
        self.flywheel_motor.set_control(self.flywheel_velocity_torque.with_velocity(test_velocity))
        
    def stop(self):
        self.flywheel_intake_motor.set(0)
        self.flywheel_motor.set_control(self.flywheel_velocity_torque.with_velocity(0))

        
