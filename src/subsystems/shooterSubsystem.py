from commands2 import Subsystem
from wpilib import SmartDashboard

from phoenix6.hardware import TalonFX
from phoenix6.controls import PositionTorqueCurrentFOC, VelocityTorqueCurrentFOC
from phoenix6.configs import TalonFXConfiguration, CurrentLimitsConfigs

from ntcore import NetworkTableInstance, NetworkTable, FloatPublisher

from constants import Shooter

class ShooterSubsystem(Subsystem):
    def __init__(self):
        super().__init__()

        self.turretMotor: TalonFX = TalonFX(Shooter.Consts.turretId)
        self.anglingMotor: TalonFX = TalonFX(Shooter.Consts.anglingId)
        self.shootingMotor: TalonFX = TalonFX(Shooter.Consts.shootingId)

        turretConfiguration: TalonFXConfiguration = TalonFXConfiguration()
        turretConfiguration.with_current_limits(
            CurrentLimitsConfigs()
            .with_stator_current_limit(40)
            .with_supply_current_limit(20)
        )
        turretConfiguration.slot0.with_k_p(1).with_k_i(0).with_k_d(0).with_k_s(0)
        self.turretMotor.configurator.apply(turretConfiguration)

        anglingConfiguration: TalonFXConfiguration = TalonFXConfiguration()
        anglingConfiguration.with_current_limits(
            CurrentLimitsConfigs()
            .with_stator_current_limit(60)
            .with_supply_current_limit(30)
        )
        anglingConfiguration.slot0.with_k_p(1).with_k_i(0).with_k_d(0).with_k_s(0)
        self.anglingMotor.configurator.apply(anglingConfiguration)

        shootingConfiguration: TalonFXConfiguration = TalonFXConfiguration()
        shootingConfiguration.with_current_limits(
            CurrentLimitsConfigs()
            .with_stator_current_limit(80)
            .with_supply_current_limit(40)
        )
        shootingConfiguration.slot0.with_k_p(1).with_k_i(0).with_k_d(0).with_k_s(0)
        self.shootingMotor.configurator.apply(shootingConfiguration)

        self.turretOut = PositionTorqueCurrentFOC(0)
        self.angleOut = PositionTorqueCurrentFOC(0)
        self.shooterOut = VelocityTorqueCurrentFOC(0)

        self.turretMotor.set_position(0)
        self.anglingMotor.set_position(0)

        inst = NetworkTableInstance.getDefault()
        self.shooterTable: NetworkTable = inst.getTable("ShooterTable")
        self.rpmPub: FloatPublisher = self.shooterTable.getFloatTopic("X").publish()
        self.rpmPub.set(0)

    def turnTurret(self, position: float):
        self.turretOut.with_position(position)
        self.turretMotor.set_control(self.turretOut)

    def angleShooter(self, position: float):
        self.angleOut.with_position(position)
        self.anglingMotor.set_control(self.angleOut)

    def setRpm(self, rpm: float):
        self.shooterOut.with_velocity(rpm / 60)
        self.shootingMotor.set_control(self.shooterOut)

    def periodic(self):
        self.rpmPub.set(self.shootingMotor.get_rotor_velocity().value)
