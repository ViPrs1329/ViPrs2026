from commands2 import Subsystem
from constants import Shooter
from tuning.tunable import TunableDouble
from wpilib import Servo

class HoodSubsystem(Subsystem):
    def __init__(self):
        super().__init__()

        self.leftHood = Servo(0)
        self.rightHood = Servo(1)
        for actuator in (self.leftHood, self.rightHood):
            actuator.setBounds(2000, 1500, 1500, 1500, 1000)

        self.tunableHood = TunableDouble("Hood Tunable", 0, "Hood")

    def periodic(self):   
        setpoint = self.tunableHood.get() 
        self.leftHood.set(setpoint)
        self.rightHood.set(setpoint)