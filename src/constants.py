import numpy
from wpimath.geometry import Translation2d, Rotation2d
from pathplannerlib.config import PIDConstants
from math import pi


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

