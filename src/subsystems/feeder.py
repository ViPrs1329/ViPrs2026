from commands2 import Subsystem
from phoenix6 import hardware, SignalLogger, swerve, units, utils

class Feeder (Subsystem):
    def __init__(self):
        super().__init__()


        self.feederMotor = hardware.TalonFX(45)


    def periodic(self):
        self.feederMotor.setVoltage(6)