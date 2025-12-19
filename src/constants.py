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

class Input:

    class Consts:
        pass

    class States:
        pass

class Limelight:

    class Consts:
        #TODO update these names to match the limelights on the robot
        tableNames: list[str] = ["limelight-lside", "limelight-rside"]

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