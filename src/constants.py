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
        kickerDutyCycle: float = 0.4

class Intake:
    class CANids:
        leftIntakeMotor: int = 22
        rightIntakeMotor: int = 23

    class Consts:
        intakeDutyCycle: float = 0.35

class Shooter:
    class CANids:
        shooter: int = 53

class Turret:
    class CANids:
        turretMotor: int = 51
        turretEncoder: int = 52

    class Consts:
        # forward = 0 revs, left = 0.25 revs, right = -0.25 revs, back = 0.5 revs
        minRotation: float = -0.25
        maxRotation: float = 0.75

class Slapdown:
    class CANids:
        slapdownMotor: int = 21  # TODO: confirm actual CAN id

    class Consts:
        gearRatio: float = 43.75
        extendDegrees: float = 130.0

        # Vertical (the arm's unstable top-of-stroke balance point, where the arm
        # briefly needs zero gravity feedforward) is 35 degrees into the stroke from
        # stowed. Horizontal, where the TalonFX's Arm_Cosine gravity type reads its
        # angle as zero, is another 90 degrees past that.
        stowedDegreesFromHorizontal: float = -(35.0 + 90.0)

        # Positions are in mechanism (output shaft) revolutions -- the motor is
        # configured with SensorToMechanismRatio = gearRatio, so 1.0 here is one full
        # revolution of the arm, not the rotor -- offset so 0 lines up with horizontal.
        # The rotor itself turns gearRatio * extendDegrees / 360 (~15.8) revolutions
        # over the stroke.
        stowedPosition: float = stowedDegreesFromHorizontal / 360.0
        extendedPosition: float = stowedPosition + extendDegrees / 360.0
