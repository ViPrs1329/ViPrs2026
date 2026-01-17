from commands2 import Subsystem
from wpilib import SmartDashboard

from phoenix6.hardware import TalonFX
from phoenix6.controls import DutyCycleOut, VelocityVoltage
from phoenix6.configs import TalonFXConfiguration

from constants import Shooter

class shooterSubsystem(Subsystem):
    def __init__(self):
        super().__init__()

        self.turretMotor: TalonFX = TalonFX(Shooter.Consts.turretId)
        self.anglingMotor: TalonFX = TalonFX(Shooter.Consts.anglingId)
        self.shootingMotor: TalonFX = TalonFX(Shooter.Consts.shootingId)

        self.turretOut = DutyCycleOut(0)
        self.angleOut = DutyCycleOut(0)
        self.shooterVelocity = VelocityVoltage(0)
        def turnTurret(self, speed: float):
            self.turretMotor.set_control(self.turretOut.with_output(speed))

        def stopTurret(self):
            self.turnTurret(0)

        def angleShooter(self, speed: float):
            self.anglingMotor.set_control(self.angleOut.with_output(speed))
        def stopAngle(self):
            self.angleShooter(0)
        def shoot(self, rpm: float):
            self.shootingMotor.set_control(self.shooterVelocity.with_velocity(rpm))
        def stopShooter(self):
            self.shootingMotor.set_control(self.shooterVelocity.with_velocity(0))
        def periodic(self):
            SmartDashboard.putNumber(
            "Shooter Velocity",
            self.shootingMotor.get_velocity().value
            )           

