from phoenix6.configs import TalonFXConfiguration, TalonFXSConfiguration
from phoenix6.signals import NeutralModeValue


class ShooterConstants:
    
    #TODO fix CAN ID's after finalizing flywheel 
    FLYWHEEL_CAN_ID = 50
    FLYWHEEL_INTAKE_CAN_ID = 61
    
    #TODO Tune. Check current limits. Check if any need to be inverted, and check if coast or brake. 
    
    #Shoot Motor Configs (TalonFXS)
    talonfxs_configs = TalonFXSConfiguration()
    # talonfxs_configs.slot0.k_s = 2.5
    # talonfxs_configs.slot0.k_v = 0
    # talonfxs_configs.slot0.k_p = 5
    # talonfxs_configs.slot0.k_i = 0
    # talonfxs_configs.slot0.k_d = 0
    talonfxs_configs.motor_output.neutral_mode = NeutralModeValue.COAST
    talonfxs_configs.current_limits.stator_current_limit = 120
    talonfxs_configs.current_limits.stator_current_limit_enable = True
    
    #FWIntake Motor Configs (TalonFX)
    talonfx_configs = TalonFXConfiguration()
    # talonfx_configs.slot1.k_s = .1
    # talonfx_configs.slot1.k_v = 0
    # talonfx_configs.slot1.k_p = .11
    # talonfx_configs.slot1.k_i = 0
    # talonfx_configs.slot1.k_d = 0
    talonfx_configs.motor_output.neutral_mode = NeutralModeValue.BRAKE
    talonfx_configs.current_limits.stator_current_limit = 120
    talonfxs_configs.current_limits.stator_current_limit_enable = True