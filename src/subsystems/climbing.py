from commands2 import Subsystem
from phoenix6 import hardware

from constants import Climber

class ClimbingSubsystem(Subsystem):
    def __init__(self):
        """
        Subsystem responsible for managing the climbing mechanism of the robot.

        This subsystem controls the extension and retraction of the climbing arms,
        as well as monitoring the status of the climbing process.
        """
        # Initialize climbing mechanism components here
        
        super().__init__()

        climbingMotor: hardware.TalonFX = hardware.TalonFX(Climber.Consts.motorId)
        