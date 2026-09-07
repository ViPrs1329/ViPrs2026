import numpy
from wpimath.geometry import Translation2d, Rotation2d
from pathplannerlib.config import PIDConstants
from math import pi


# class Limelight:

#     class Consts:
#         #TODO update these names to match the limelights on the robot
#         tableNames: list[str] = ["limelight-lside", "limelight-rside"]

#         # the measured standard deviation of a single limelight at a known reference area
#         standardDeviationRef: tuple[float, float, float] = (0.05, 0.05, 0.01) # meters, meters, radians

#         # the reference target area at which the standard deviation was measured
#         targetAreaRef: float = 0.05 # percentage

#     class States:
#         pass

class Feeder:
    class CANids:
        leftConveyor: int = 41
        rightConveyor: int = 42
        backConveyor: int = 43
        kicker: int = 45

    class Consts:
        backConveyorDutyCycle: float = -0.3
        leftConveyorDutyCycle: float = -0.3
        rightConveyorDutyCycle: float = 0.3

class Intake:
    class CANids:
        leftIntakeMotor: int = 22
        rightIntakeMotor: int = 23

    class Consts:
        intakeDutyCycle: float = 0.35