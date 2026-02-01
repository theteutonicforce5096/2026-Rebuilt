from commands2 import Subsystem

from phoenix6.hardware import TalonFXS, CANcoder

import rev

class Shooter(Subsystem):
    def __init__(self, CAN_ID):
        Subsystem.__init__(self)
        
        #self.motor = TalonFXS() #CAN ID later
        self.encoder = CANcoder() #CAN ID later
        
        
    # #On-Board PID test 
    #     self.motor = rev.SparkMax(CAN_ID, rev.SparkLowLevel.MotorType.kBrushless)
    #     self.motor_config = rev.SparkMaxConfig()
    #     controller = self.motor.getClosedLoopController()
    #     controller.setSetpoint()
        
    #     self.motor_config.closedLoop.P()
        
    #     rev.SparkClosedLoopController().setSetpoint
        

        
