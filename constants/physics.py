from math import tan, cos, radians, sqrt, pi
from wpimath.geometry import Pose2d
from wpimath.units import inchesToMeters

default_y_dis = inchesToMeters(12.5) #TODO confirm height of robot
default_r = inchesToMeters(7) #flywheel radius in meters
default_g = -9.8 #m/s^2
default_θ = 67.5 #degrees

default_a = 1 #TODO constant from regression model
default_b = 0 #TODO constant from regression model


def calc_x_dis():
    #Positions from the perspective looking out from the player stations, the back left corner is (0,0)
        #Q. Does it need to be absolute values?
        #Q. Will switching sides change anything?
    hub_x_pos = inchesToMeters(158.84)
    hub_y_pos = inchesToMeters(182.11)
    x_dis = sqrt(abs(hub_x_pos - Pose2d.X) + abs(hub_y_pos - Pose2d.Y)) 
    return x_dis

def calc_velocity(x_dis: float, y_dis: float = default_y_dis,
                        θ: float = default_θ, g: float = default_g,
                        a: float = default_a, b: float = default_b):
    """
        Fuction for getting ideal velocity.
        :param x_dis: horizontal distance in meters from target to shooter
        :type x_dis: float
        :param y_dis: vertical distance in meters from target to shooter
        :type y_dis: float
        :param θ: angle in degrees from horizontal that ball shoots out at
        :type θ: float
        :param g: gravitational acceleration in meters per second squared
        :type g: float
        :param a: multiplicative coefficient from regression model
        :type a: float
        :param b: constant from regression model
        :type b: float
    """
    
    ideal_velocity_mps = sqrt(
        (g * (x_dis ** 2)) / 
        (2 * ((cos(radians(θ))) ** 2)) * (y_dis - (x_dis - tan(radians(θ))))
    )
    
    #Calibrated velocity for ball to actually shoot out at (in meters per second)
    cal_initial_velocity_mps = (a * ideal_velocity_mps) + b
    return cal_initial_velocity_mps

#puts calibrated velocity in rotations per second to use
#for testing, set a = 1, b = 0
def shoot_speed(cal_initial_velocity_mps, r: float = default_r):
    """
        Function to get shoot velocity in rotations per second (rps)
        :param r: flywheel radius in meters
        :type r: float
    """
    initial_velocity_rps = (cal_initial_velocity_mps / 
                        (2 * pi * r))
    return initial_velocity_rps

#TODO Get a good intake velocity 
def flywheel_intake_speed():
    flywheel_intake_velocity_rps = 2 #rps
    return flywheel_intake_velocity_rps
#67