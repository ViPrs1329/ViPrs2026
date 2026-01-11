from commands2 import Subsystem
import math
from phoenix6 import hardware, controls, configs, StatusCode
from constants import Intake

class IntakeSubsystem(Subsystem):
    def __init__(self) -> None:
        super().__init__()

        intakeMotor: hardware.TalonFX = hardware.TalonFX(Intake.Consts.intakeCANId)
        armMotor: hardware.TalonFX = hardware.TalonFX(Intake.Consts.armCANId)
