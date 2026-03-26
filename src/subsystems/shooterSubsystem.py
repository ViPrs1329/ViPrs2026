from commands2 import Subsystem
from wpilib import SmartDashboard, Timer

from phoenix6.hardware import TalonFX, CANcoder
from phoenix6.controls import PositionTorqueCurrentFOC, VelocityTorqueCurrentFOC, DutyCycleOut, PositionVoltage
from phoenix6.configs import TalonFXConfiguration, CurrentLimitsConfigs, MotionMagicConfigs, TorqueCurrentConfigs, ClosedLoopGeneralConfigs, SoftwareLimitSwitchConfigs, FeedbackConfigs
from phoenix6.signals import GravityTypeValue, StaticFeedforwardSignValue, FeedbackSensorSourceValue

from ntcore import NetworkTableInstance, NetworkTable, FloatPublisher

from constants import Shooter

from tuning.tunable import TunableDouble

import csv


class ShooterSubsystem(Subsystem):
    def __init__(self) -> None:
        super().__init__()

        self.turretMotor: TalonFX = TalonFX(Shooter.Consts.turretId)
        self.hoodMotor: TalonFX = TalonFX(Shooter.Consts.hoodId)
        self.shootingMotor: TalonFX = TalonFX(Shooter.Consts.shootingId)
        self.turretEncoder: CANcoder = CANcoder(Shooter.Consts.turretEncoderId)
        self.towerConveyor: TalonFX = TalonFX(Shooter.Consts.towerConveyor)
        self.frontConveyor: TalonFX = TalonFX(Shooter.Consts.frontConveyor)
        self.backConveyor: TalonFX = TalonFX(Shooter.Consts.backConveyor)

        towerConveyorConfiguration: TalonFXConfiguration = TalonFXConfiguration()
        towerConveyorConfiguration.with_current_limits(
            CurrentLimitsConfigs()
            .with_stator_current_limit(40)
            .with_supply_current_limit(20)
        )
        self.towerConveyor.configurator.apply(towerConveyorConfiguration)


        turretConfiguration: TalonFXConfiguration = TalonFXConfiguration()
        turretConfiguration.with_current_limits(
            CurrentLimitsConfigs()
            .with_stator_current_limit(40)
            .with_supply_current_limit(20)
        ).with_feedback(
            FeedbackConfigs()
            .with_feedback_sensor_source(FeedbackSensorSourceValue.FUSED_CANCODER)
            .with_feedback_remote_sensor_id(53)
        )
        turretConfiguration.slot0.with_k_p(50).with_k_i(0).with_k_d(0).with_k_s(10)
        self.turretMotor.configurator.apply(turretConfiguration)

        hoodConfiguration: TalonFXConfiguration = TalonFXConfiguration()
        hoodConfiguration.with_current_limits(
            CurrentLimitsConfigs()
            .with_stator_current_limit(60)
            .with_supply_current_limit(30)
        )
        hoodConfiguration.slot0.with_k_p(10).with_k_i(0).with_k_d(0.1).with_k_s(0.23).with_k_g(0.06).with_gravity_type(GravityTypeValue.ELEVATOR_STATIC).with_static_feedforward_sign(StaticFeedforwardSignValue.USE_CLOSED_LOOP_SIGN)
        self.hoodMotor.configurator.apply(hoodConfiguration)

        shootingConfiguration: TalonFXConfiguration = TalonFXConfiguration()
        shootingConfiguration.with_current_limits(
            CurrentLimitsConfigs()
            .with_stator_current_limit(80)
            .with_supply_current_limit(40)
            .with_stator_current_limit_enable(True)
            .with_supply_current_limit_enable(True)
        )
        shootingConfiguration.slot0.with_k_p(5).with_k_i(0).with_k_d(0).with_k_s(2).with_k_v(0.23).with_k_a(0).with_static_feedforward_sign(StaticFeedforwardSignValue.USE_CLOSED_LOOP_SIGN)
        self.shootingMotor.configurator.apply(shootingConfiguration)

        self.turretOut = PositionTorqueCurrentFOC(0).with_slot(0)
        self.angleOut = PositionVoltage(0).with_slot(0)
        # self.shooterOut = VelocityTorqueCurrentFOC(0).with_slot(0)

        self.turretMotor.set_position(0)
        self.hoodMotor.set_position(0)

        inst = NetworkTableInstance.getDefault()
        self.shooterTable: NetworkTable = inst.getTable("ShooterTable")
        self.rpmPub: FloatPublisher = self.shooterTable.getFloatTopic("RPM").publish()
        self.rpmPub.set(0)
        self.hoodPub: FloatPublisher = self.shooterTable.getFloatTopic("HoodAngle").publish()
        self.hoodPub.set(0)
        self.turretPub: FloatPublisher = self.shooterTable.getFloatTopic("TurretAngle").publish()
        self.turretPub.set(0)
        self.targetRPMPub: FloatPublisher = self.shooterTable.getFloatTopic("TargetRPM").publish()
        self.targetRPMPub.set(0)
        self.targetHoodPub: FloatPublisher = self.shooterTable.getFloatTopic("TargetHoodAngle").publish()
        self.targetHoodPub.set(0)
        self.targetTurretPub: FloatPublisher = self.shooterTable.getFloatTopic("TargetTurretAngle").publish()
        self.targetTurretPub.set(0)
        self.rpmTunable: TunableDouble = TunableDouble("Tunable RPM", 0, "Shooter")
        self.hoodTunable: TunableDouble = TunableDouble("Tunable Hood", 0, "Shooter")
        self.distTunable: TunableDouble = TunableDouble("Distance Tunable", 0, "Shooter")
        self.turretTunable: TunableDouble = TunableDouble("Target Turret", 0, "Shooter")

        self._last_publish_time = Timer.getFPGATimestamp()

        self.shooterCalibrationData: list[dict[str, float]] = self.loadCalibrationData("/home/lvuser/py/tuning/shooterTable.csv")
        self.distances = [i['distance'] for i in self.shooterCalibrationData]
        self.columns = [k for k in self.shooterCalibrationData[0].keys() if k != 'distance']
            
        # self.startConveyor()
        # self.angleTurret(1)

    def angleTurret(self, position: float) -> None:
        self.turretOut.with_position(position)
        self.turretMotor.set_control(self.turretOut)
        self.targetTurretPub.set(position)

    def angleHood(self, position: float) -> None:
        self.angleOut = self.angleOut.with_position(position)
        self.hoodMotor.set_control(self.angleOut)
        self.targetHoodPub.set(position)

    def setRPM(self, rpm: float) -> None:
        # output = self.shooterOut.with_velocity(100)
        self.shootingMotor.set_control(VelocityTorqueCurrentFOC(rpm / 60).with_slot(0))
        self.targetRPMPub.set(rpm)

    def startConveyor(self):
        self.towerConveyor.set_control(DutyCycleOut(0.7, True))
        self.frontConveyor.set_control(DutyCycleOut(0.6, True))
        self.backConveyor.set_control(DutyCycleOut(-0.6, True))

    def stopConveyor(self):
        self.towerConveyor.stopMotor()
        self.frontConveyor.stopMotor()
        self.backConveyor.stopMotor()

    def reverseConveyor(self):
        self.towerConveyor.set_control(DutyCycleOut(-0.7, True))
        self.frontConveyor.set_control(DutyCycleOut(-0.6, True))
        self.backConveyor.set_control(DutyCycleOut(0.6, True))

    def loadCalibrationData(self, filePath) -> list[dict[str, float]]:
        data: list[dict[str, float]] = []
        with open(filePath, mode='r') as f:
            # DictReader automatically uses the first row as keys
            reader = csv.DictReader(f)
            for row in reader:
                # strip() removes spaces; float() converts strings to numbers
                clean_row = {key.strip(): float(val.strip()) for key, val in row.items()}
                data.append(clean_row)
                
        # Crucial: Ensure the list is sorted by distance for the search algorithm
        data.sort(key=lambda x: x['distance'])
        return data
    
    def lookupCalibration(self, distance: float) -> dict[str, float]:
        # 1. Handle Lower Bound Clamping
        if distance <= self.shooterCalibrationData[0]['distance']:
            return self.shooterCalibrationData[0].copy()

        # 2. Linear Search for the 'Upper' bounding row
        upper_idx = -1
        for i in range(len(self.shooterCalibrationData)):
            if self.shooterCalibrationData[i]['distance'] > distance:
                upper_idx = i
                break
        
        # 3. Handle Upper Bound Clamping (if no distance was greater)
        if upper_idx == -1:
            return self.shooterCalibrationData[-1].copy()

        # 4. Identify the two rows to interpolate between
        lower = self.shooterCalibrationData[upper_idx - 1]
        upper = self.shooterCalibrationData[upper_idx]

        # 5. Calculate Interpolation Factor (t)
        t = (distance - lower['distance']) / (upper['distance'] - lower['distance'])

        # 6. Build the results
        results = {'distance': distance}
        for col in self.columns:
            y0 = lower[col]
            y1 = upper[col]
            results[col] = y0 + t * (y1 - y0)
            
        return results

    def updateDistance(self, distance: float) -> None:
        calibration = self.lookupCalibration(distance)
        # self.setRPM(calibration['targetRPM'])
        self.setRPM(self.rpmTunable.get())
        # self.angleHood(calibration['hoodAngle'])
        self.angleHood(self.hoodTunable.get())
        # self.angleTurret(self.turretTunable.get())

    def periodic(self) -> None:
        self.updateDistance(self.distTunable.get())


        if Timer.getFPGATimestamp() - self._last_publish_time >= 0.25:
            self._last_publish_time = Timer.getFPGATimestamp()
            self.rpmPub.set(self.shootingMotor.get_rotor_velocity().value * 60)
            self.hoodPub.set(self.hoodMotor.get_position().value)
            self.turretPub.set(self.turretMotor.get_position().value)
