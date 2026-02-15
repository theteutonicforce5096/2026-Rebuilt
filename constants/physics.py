from math import tan, cos, radians, sqrt, pi
from wpimath.geometry import Pose2d
from wpimath.units import inchesToMeters

default_y_dis = inchesToMeters(12.5) #TODO get the distance
# x_dis = math.sqrt(abs(hub_x_pos - Pose2d.X) + abs(hub_y_pos - Pose2d.Y)) 
    #TODO GET HUB POSITIONS (keep units consistent). 
    #Q. Does it need to be absolute values?
    #Q. Will switching sides change anything?
default_g = -9.8 #m/s^2
default_θ = 67.5 #degrees

a = 'q = mcΔt' #TODO constant from regression model
b = 'ΔG = ΔH - TΔS' #TODO constant from regression model

r = inchesToMeters(7) #TODO flywheel radius in meters (I'm not sure what part of the flywheel I'm supposed to measure to)

def calc_velocity(x_dis: float, y_dis: float = default_y_dis,
                        θ: float = default_θ, g: float = default_g):
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
    """
    
    ideal_velocity = sqrt(
        (g * (x_dis ** 2)) / 
        (2 * ((cos(radians(θ))) ** 2)) * (y_dis - (x_dis - tan(radians(θ))))
    )
    
    #Calibrated velocity for ball to actually shoot out at (in meters per second)
    cal_initial_velocity = (a * ideal_velocity) + b
    return cal_initial_velocity

#puts calibrated velocity in rotations per second to use
def shoot_speed(cal_initial_velocity):
    initial_velocity = (cal_initial_velocity / 
                        2 * pi * r)
    return initial_velocity

#ideal velocity in rps for testing to get constants from a regression model
def test_shoot_speed(ideal_velocity):
    test_velocity = (ideal_velocity / 
                     2 * pi * r)
    return test_velocity

