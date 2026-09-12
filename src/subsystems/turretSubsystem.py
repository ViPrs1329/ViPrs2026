from commands2 import Subsystem
from constants import Turret
from phoenix6 import hardware, controls, configs, signals, StatusCode
from ntcore import NetworkTableInstance, NetworkTable, FloatPublisher, StructPublisher
from tuning.tunable import TunableDouble
from wpilib import SmartDashboard, Timer, DriverStation
from wpimath.geometry import Pose2d, Rotation2d
import math

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
            .with_motion_magic_acceleration(1000)
            .with_motion_magic_cruise_velocity(80)
            .with_motion_magic_jerk(5000)
        )
        turretConfig.slot0.with_k_p(2).with_k_i(0).with_k_d(0).with_k_s(0.2).with_k_v(0).with_k_a(0).with_static_feedforward_sign(signals.StaticFeedforwardSignValue.USE_CLOSED_LOOP_SIGN)
        self.turretMotor.configurator.apply(turretConfig)

        self.turretEncoder = hardware.CANcoder(Turret.CANids.turretEncoder)

        inst = NetworkTableInstance.getDefault()
        self.turretTable: NetworkTable = inst.getTable("TurretTable")
        self.posPub: FloatPublisher = self.turretTable.getFloatTopic("Position").publish()
        self.posPub.set(0)
        self.targetPub: FloatPublisher = self.turretTable.getFloatTopic("TargetPos").publish()
        self.targetPub.set(0)
        self.posTunable: TunableDouble = TunableDouble("Tunable Pos", 0, "Turret")
        self.targetPose: StructPublisher = self.turretTable.getStructTopic("Target Pose", Pose2d).publish()
        self.robotPose: Pose2d = Pose2d(0, 0, 0)
        self.targetDistance: float = 0

        self._last_publish_time = Timer.getFPGATimestamp()

        self.alliance = DriverStation.getAlliance()

    def setRobotPose(self, pose: Pose2d):
        self.robotPose = pose

    def rotateTo(self, rotation):
        self.turretMotor.set_control(controls.MotionMagicVoltage(rotation * 81))

    def getTargetPos(self, position: Pose2d):
        if self.alliance == DriverStation.Alliance.kRed:
            if position.X() > 11.915:
                # on blue size
                return Pose2d(11.915, 4, 0)
            else:
                # passing
                if position.Y() < 4:
                    return Pose2d(16.54 - 2, 2, 0)
                else:
                    return Pose2d(16.54 - 2, 6, 0)

        elif self.alliance == DriverStation.Alliance.kBlue:
            if position.X() < 4.625:
                # on red side
                return Pose2d(4.625, 4, 0)
            else:
                # passing
                if position.Y() < 4:
                    return Pose2d(2, 2, 0)
                else:
                    return Pose2d(2, 6, 0)


    @staticmethod
    def calculateTurretRotation(robotPose: Pose2d, targetPose: Pose2d) -> float:
        """Calculates the turret rotation (in revolutions) needed to face targetPose
        from robotPose, in the turret's zeroed frame: forward = 0, left = 0.25,
        right = -0.25, back = 0.5. Result is wrapped into the turret's bounds of
        [-0.25, 0.75] revs.
        """
        relativeTranslation = targetPose.relativeTo(robotPose).translation()
        bearing = Rotation2d(relativeTranslation.X(), relativeTranslation.Y())
        turretRotation = bearing.radians() / math.tau

        if turretRotation < Turret.Consts.minRotation:
            turretRotation += 1.0

        return turretRotation

    def syncMotorWithEncoder(self):
        turretPosition = self.turretEncoder.get_position().value
        self.turretMotor.set_position(turretPosition * 81)

    def periodic(self):
        targetPosition = self.getTargetPos(self.robotPose)
        if targetPosition is not None:
            self.targetDistance = math.sqrt((self.robotPose.X() - targetPosition.X()) ** 2 + (self.robotPose.Y() - targetPosition.Y()) ** 2)
            self.rotateTo(self.calculateTurretRotation(self.robotPose, targetPosition))
        if Timer.getFPGATimestamp() - self._last_publish_time >= 0.25:
            self._last_publish_time = Timer.getFPGATimestamp()
            self.posPub.set(self.turretMotor.get_position().value / 81)
            # print(self.alliance)
            self.targetPose.set(self.getTargetPos(self.robotPose))
            self.alliance = DriverStation.getAlliance()