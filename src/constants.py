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
    # Swerve CAN IDs
    #TODO update these IDs to match the robot

    flDrive: int = 1
    flRotation: int = 2
    flEncoder: int = 3

    frDrive: int = 4
    frRotation: int = 5
    frEncoder: int = 6

    blDrive: int = 7
    blRotation: int = 8
    blEncoder: int = 9

    brDrive: int = 10
    brRotation: int = 11
    brEncoder: int = 12

    pigeon: int = 100

class Input:

    class Consts:
        pass

    class States:
        pass

class Drive:

    class Consts:
        
        #TODO update these offsets after characterizing the robot
        flModuleOffset: Translation2d = Translation2d()
        frModuleOffset: Translation2d = Translation2d()
        blModuleOffset: Translation2d = Translation2d()
        brModuleOffset: Translation2d = Translation2d()

        # this is for pathplanner
        # see below for actual pid values for the motors themselves
        #TODO tune these pathplanner values
        translationConstants: PIDConstants = PIDConstants(0.1, 0, 0, 0)
        rotationConstants: PIDConstants = PIDConstants(0.1, 0, 0, 0)

        driveCurrentLimit: int = 30
        rotCurrentLimit: int = 30

        driveGearRatio: float = 6.75
        rotGearRatio: float = 12.8
        wheelRadius: float = 0.0508  # 2 inches in meters

        #TODO tune this value
        rampRate: float = 0.5

        # PID values for the motors themselves
        #TODO tune these values
        rotP: int = 1
        rotI: int = 0
        rotD: int = 0

        driveP: int = 1
        driveI: int = 0
        driveD: int = 0

        maxSpeed: float = 1.0 # in m/s
        maxAngularSpeed: float = 0.5
        inputDeadzone: float = 0.1
        maxModuleSpeed: float = maxSpeed * 1.414

        #TODO tune the wheel feed forward value
        #TODO implement this in the code
        wheelFF: float = 0.05

        visionMeasurementStdDevs: tuple[float, float, float] = (0.1, 0.1, 0.1)  # in meters and radians

    class States:
        # modes for the drive system
        # autonomous mode uses pathplanner and any other 
        # code that drives the robot automatically

        # teleop mode uses driver inputs to control the robot
        autonomous: int = 0
        teleop: int = 1

class Sensor:

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