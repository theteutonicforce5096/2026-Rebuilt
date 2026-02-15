from phoenix6.configs import TalonFXConfiguration, TalonFXSConfiguration
from phoenix6.signals import NeutralModeValue


class ShooterConstants:
    
    #TODO fix CAN ID's after finalizing flywheel 
    FLYWHEEL_CAN_ID = 50
    FLYWHEEL_INTAKE_CAN_ID = 61
    
    #TODO Tune. ADD CURRENT LIMITS. Check if any need to be inverted, and check if coast or brake. 
    #TODO: For current limits: Torque current? Stator & supply? 
    
    #Shoot Motor Configs (TalonFXS)
    talonfxs_configs = TalonFXSConfiguration()
    # talonfxs_configs.slot0.k_s = 2.5
    # talonfxs_configs.slot0.k_v = 0
    # talonfxs_configs.slot0.k_p = 5
    # talonfxs_configs.slot0.k_i = 0
    # talonfxs_configs.slot0.k_d = 0
    talonfxs_configs.motor_output.neutral_mode = NeutralModeValue.COAST
    
    #FWIntake Motor Configs (TalonFX)
    talonfx_configs = TalonFXConfiguration()
    # talonfx_configs.slot0.k_s = .1
    # talonfx_configs.slot0.k_v = .12
    # talonfx_configs.slot0.k_p = .11
    # talonfx_configs.slot0.k_i = 0
    # talonfx_configs.slot0.k_d = 0
    talonfx_configs.motor_output.neutral_mode = NeutralModeValue.BRAKE