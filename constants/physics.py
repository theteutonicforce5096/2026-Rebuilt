from math import tan, cos, radians, sqrt, pi
from wpimath.geometry import Rotation2d, Transform2d, Translation2d
from wpimath.units import inchesToMeters
from wpilib import DriverStation

default_y_dis = inchesToMeters(99.11) #120.36 in. (hub height) - 21.25 in. (flywheel height)
default_r = inchesToMeters(2) #flywheel radius in meters
default_g = -9.8 #m/s^2
default_θ = 67.5 #degrees

def _calc_profile_velocity_mps(x_dis: float, y_dis: float = default_y_dis,
                               θ: float = default_θ, g: float = default_g):
    try:
        return abs(calc_velocity(x_dis, y_dis, θ, g))
    except ValueError:
        velocity_term = (
            (g * (x_dis ** 2))
            / (2 * ((cos(radians(θ))) ** 2))
            * (y_dis - (x_dis - tan(radians(θ))))
        )

        # Preserve the existing model shape while keeping the shot-profile
        # baseline finite across the scoring range.
        ideal_velocity_mps = sqrt(abs(velocity_term))
        return ideal_velocity_mps

def calc_x_dis(x_pose, y_pose): 
    # Create shooter position variable
    # X: -7.78 inches (behind center)
    # Y: -7.95 inches (right of center)
    shooter_offset = Transform2d(
        Translation2d(inchesToMeters(-7.78), inchesToMeters(-7.95)),
        Rotation2d(0) # 0 means it always faces the same way as the drivebase
    )

    #Bottom left of field map (blue on left) is (0,0). Values are for AndyMark Field
    alliance_color = DriverStation.getAlliance()
    if alliance_color is not None:
        if alliance_color == DriverStation.Alliance.kBlue:
            hub_x_pos = inchesToMeters(181.56)
            hub_y_pos = inchesToMeters(158.32)
        else:
            hub_x_pos = inchesToMeters(468.56)
            hub_y_pos = inchesToMeters(158.32)
            
    #Shooter is 7.95 inches to the right of the pigeon
    #Shooter is 7.78 inches behind the pigeon
    x_dis = sqrt(
        (hub_x_pos - (x_pose + inchesToMeters(7.78)))**2 + 
        (hub_y_pos - (y_pose - inchesToMeters(7.95)))**2)
    return x_dis

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
    
    ideal_velocity_mps = sqrt(
        (g * (x_dis ** 2)) / 
        (2 * ((cos(radians(θ))) ** 2)) * (y_dis - (x_dis - tan(radians(θ))))
    )
    return ideal_velocity_mps

#Calibration Notes:
#these were kinda inconsistent
#TODO this info is useless cuz PID is cooked
#50 rps for 58 inches (Hub front to robot front) 7.5ft tp back
#60 rps for 11 ft (Hub front to robot back)
#robot with bumpers ~34 in. long

#target velocity puts ideal velocity in rotations per second
def shoot_speed(ideal_velocity_mps, r: float = default_r):
    """
        Function to get shoot velocity in rotations per second (rps)

        :param r: flywheel radius in meters
        :type r: float
    """
    target_velocity = (ideal_velocity_mps / 
                        (2 * pi * r))
    return target_velocity


def calc_time_of_flight(x_dis: float, y_dis: float = default_y_dis,
                        θ: float = default_θ, g: float = default_g):
    """
        Function to get shot time of flight in seconds from the same baseline
        model used for launch velocity.
    """

    ideal_velocity_mps = _calc_profile_velocity_mps(x_dis, y_dis, θ, g)
    horizontal_velocity_mps = ideal_velocity_mps * cos(radians(θ))
    if abs(horizontal_velocity_mps) < 1e-9:
        raise ValueError("Horizontal launch velocity must be non-zero")

    return x_dis / horizontal_velocity_mps


def calc_shot_profile(x_dis: float, y_dis: float = default_y_dis,
                      θ: float = default_θ, g: float = default_g,
                      r: float = default_r):
    """
        Function to get the baseline shot profile for a given distance.

        :returns: Tuple of target flywheel speed in rotations per second and
        time of flight in seconds.
    """

    ideal_velocity_mps = _calc_profile_velocity_mps(x_dis, y_dis, θ, g)
    return (
        shoot_speed(ideal_velocity_mps, r),
        calc_time_of_flight(x_dis, y_dis, θ, g),
    )

#TODO find better value
#If you are converting a percent, multiply by 106
def flywheel_intake_speed():
    flywheel_intake_velocity_rps = 26.5 #rps
    return flywheel_intake_velocity_rps
