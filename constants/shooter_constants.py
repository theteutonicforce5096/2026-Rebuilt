from math import tan, cos, radians, sqrt
from wpimath.geometry import Pose2d
from wpimath.units import inchesToMeters

#TODO Maybe add a new shooter physics file

default_y_dis = inchesToMeters(12.5) #TODO get the distance
# x_dis = math.sqrt(abs(hub_x_pos - Pose2d.X) + abs(hub_y_pos - Pose2d.Y)) 
    #TODO GET HUB POSITIONS (keep units consistent). 
    #Q. Does it need to be absolute values?
    #Q. Will switching sides change anything?
default_g = -9.8 #m/s^2
default_θ = 67.5 #degrees

a = 'q = mcΔt' #TODO constant from regression model
b = 'ΔG = ΔH - TΔS' #TODO constant from regression model

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
    
    #Calibrated velocity for ball to actually shoot out at
    cal_initial_velocity = (a * ideal_velocity) + b
    return cal_initial_velocity