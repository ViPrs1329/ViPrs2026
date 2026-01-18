import numpy
from wpimath.geometry import Translation2d, Rotation2d
from pathplannerlib.config import PIDConstants
from math import pi

# wpilib has us covered for simple conversions
# class convert:
#     def in2m(inches):
#         return 0.0254 * inches
    
#     def rev2rad(rev):
#         return rev * 2 * numpy.pi

#     def rad2rev(radians):
#         return radians / (2 * numpy.pi)

class CANIDs:
    #TODO update these IDs to match the robot
    pass

class Input:

    class Consts:
        pass

    class States:
        pass

class Limelight:

    class Consts:
        #TODO update these names to match the limelights on the robot
        tableNames: list[str] = ["limelight-lside", "limelight-rside"]

        # the measured standard deviation of a single limelight at a known reference area
        standardDeviationRef: tuple[float, float, float] = (0.05, 0.05, 0.01) # meters, meters, radians

        # the reference target area at which the standard deviation was measured
        targetAreaRef: float = 0.05 # percentage

    class States:
        pass

class Climber:
    class Consts:
        motorId: int = 100  # Example motor ID for the climbing mechanism

    class States:
        pass

class Shooter:
    class Consts:
        turretId: int = 100
        anglingId: int = 100
        shootingId: int = 100

    class States:
        pass

class FNS:

    class Consts:
        # acceptable tolerances between the odometry and vision system
        #TODO tune these values
        visionWheelTranslationPoseTolerance: float = 0.1  # in meters
        visionWheelRotationPoseTolerance: float = 0.1  # in radians

    class States:
        pass
 
