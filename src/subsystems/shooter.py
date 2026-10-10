from commands2 import Subsystem
from phoenix6 import hardware, SignalLogger, swerve, units, utils


class Shooter(Subsystem):
    def __init__(self):
        super().__init__()

        self.shooterMotor = hardware.TalonFX(53)


    def periodic(self):
        self.shooterMotor.setVoltage(3)