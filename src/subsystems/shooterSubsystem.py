from commands2 import Subsystem
from constants import Shooter
from phoenix6 import hardware, controls, configs, signals, StatusCode
from ntcore import NetworkTableInstance, NetworkTable, FloatPublisher, StructPublisher
from tuning.tunable import TunableDouble
from wpilib import SmartDashboard, Timer, DriverStation



class ShooterSubsystem(Subsystem):
    def __init__(self):
        super().__init__()

        self.shooter = hardware.TalonFX(Shooter.CANids.shooter)
        shooterConfig = configs.TalonFXConfiguration()
        shooterConfig.with_current_limits(
            configs.CurrentLimitsConfigs()
            .with_stator_current_limit(80)
            .with_supply_current_limit(40)
        )
        shooterConfig.slot0.with_k_p(5).with_k_i(0).with_k_d(0).with_k_s(8).with_k_v(0.5).with_k_a(0)
        self.shooter.configurator.apply(shooterConfig)

        inst = NetworkTableInstance.getDefault()
        self.shooterTable: NetworkTable = inst.getTable("ShooterTable")
        self.rpmPub: FloatPublisher = self.shooterTable.getFloatTopic("RPM").publish()
        self.rpmPub.set(0)
        self.targetRPMPub: FloatPublisher = self.shooterTable.getFloatTopic("TargetRPM").publish()
        self.targetRPMPub.set(0)
        self.rpmTunable: TunableDouble = TunableDouble("Tunable RPM", 0, "Shooter")

        self._last_publish_time = Timer.getFPGATimestamp()

    def shootFromTunable(self):
        self.shooter.set_control(controls.VelocityTorqueCurrentFOC(self.rpmTunable.get() / 60).with_slot(0))

    def setRPM(self, rpm: float) -> None:
        self.shooter.set_control(controls.VelocityTorqueCurrentFOC(rpm / 60).with_slot(0))
        self.targetRPMPub.set(rpm)

    def stopShooter(self):
        self.shooter.stopMotor()

    def periodic(self) -> None:


        if Timer.getFPGATimestamp() - self._last_publish_time >= 0.25:
            self._last_publish_time = Timer.getFPGATimestamp()
            self.rpmPub.set(self.shooter.get_rotor_velocity().value * 60)
            