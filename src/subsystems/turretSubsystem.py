from commands2 import Subsystem
from constants import Turret
from phoenix6 import hardware, controls, configs, signals, StatusCode
from ntcore import NetworkTableInstance, NetworkTable, FloatPublisher, StructPublisher
from tuning.tunable import TunableDouble
from wpilib import SmartDashboard, Timer, DriverStation

class TurretSubsystem(Subsystem):
    def __init__(self):
        super().__init__()
        self.turretMotor = hardware.TalonFX(Turret.CANids.turretMotor)
        turretConfig = configs.TalonFXConfiguration()
        turretConfig.with_current_limits(
            configs.CurrentLimitsConfigs()
            .with_stator_current_limit(40)
            .with_supply_current_limit(20)
        )
        turretConfig.with_motor_output(
            configs.MotorOutputConfigs()
            .with_inverted(signals.InvertedValue.CLOCKWISE_POSITIVE)
            .with_peak_forward_duty_cycle(0.1)
            .with_peak_reverse_duty_cycle(-0.1)
        )
        turretConfig.with_motion_magic(
            configs.MotionMagicConfigs()
            .with_motion_magic_acceleration(200)
            .with_motion_magic_cruise_velocity(80)
            .with_motion_magic_jerk(500)
        )
        turretConfig.slot0.with_k_p(10).with_k_i(0).with_k_d(0).with_k_s(1.5).with_k_v(0).with_k_a(0).with_static_feedforward_sign(signals.StaticFeedforwardSignValue.USE_CLOSED_LOOP_SIGN)
        self.turretMotor.configurator.apply(turretConfig)

        self.turretEncoder = hardware.CANcoder(Turret.CANids.turretEncoder)

        inst = NetworkTableInstance.getDefault()
        self.turretTable: NetworkTable = inst.getTable("TurretTable")
        self.posPub: FloatPublisher = self.turretTable.getFloatTopic("Position").publish()
        self.posPub.set(0)
        self.targetPub: FloatPublisher = self.turretTable.getFloatTopic("TargetPos").publish()
        self.targetPub.set(0)
        self.posTunable: TunableDouble = TunableDouble("Tunable Pos", 0, "Turret")

        self._last_publish_time = Timer.getFPGATimestamp()

    def rotateTo(self, rotation):
        self.turretMotor.set_control(controls.MotionMagicTorqueCurrentFOC(rotation * 81))

    def syncMotorWithEncoder(self):
        turretPosition = self.turretEncoder.get_position().value
        self.turretMotor.set_position(turretPosition * 81)

    def periodic(self):
        if Timer.getFPGATimestamp() - self._last_publish_time >= 0.25:
            self._last_publish_time = Timer.getFPGATimestamp()
            self.posPub.set(self.turretMotor.get_position().value / 81)
            self.rotateTo(0.25)