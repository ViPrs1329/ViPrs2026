from commands2 import Subsystem
from wpilib import SmartDashboard

from phoenix6.hardware import TalonFX, CANcoder
from phoenix6.controls import PositionTorqueCurrentFOC, VelocityTorqueCurrentFOC, DutyCycleOut
from phoenix6.configs import TalonFXConfiguration, CurrentLimitsConfigs, MotionMagicConfigs, TorqueCurrentConfigs, ClosedLoopGeneralConfigs, SoftwareLimitSwitchConfigs

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
        )
        turretConfiguration.slot0.with_k_p(0).with_k_i(0).with_k_d(0).with_k_s(0)
        self.turretMotor.configurator.apply(turretConfiguration)

        hoodConfiguration: TalonFXConfiguration = TalonFXConfiguration()
        hoodConfiguration.with_current_limits(
            CurrentLimitsConfigs()
            .with_stator_current_limit(60)
            .with_supply_current_limit(30)
        )
        hoodConfiguration.slot0.with_k_p(0.1).with_k_i(0).with_k_d(0).with_k_s(5)
        self.hoodMotor.configurator.apply(hoodConfiguration)

        shootingConfiguration: TalonFXConfiguration = TalonFXConfiguration()
        shootingConfiguration.with_current_limits(
            CurrentLimitsConfigs()
            .with_stator_current_limit(80)
            .with_supply_current_limit(40)
            .with_stator_current_limit_enable(True)
            .with_supply_current_limit_enable(True)
        )
        shootingConfiguration.with_torque_current(
            TorqueCurrentConfigs()
            .with_peak_forward_torque_current(100)
            .with_peak_reverse_torque_current(-100)
            .with_torque_neutral_deadband(0)
        )
        shootingConfiguration.with_closed_loop_general(
            ClosedLoopGeneralConfigs()
            .with_gain_sched_error_threshold(0)
        )
        shootingConfiguration.with_software_limit_switch(
            SoftwareLimitSwitchConfigs()
            .with_forward_soft_limit_enable(False)
            .with_reverse_soft_limit_enable(False)
        )
        shootingConfiguration.slot0.with_k_p(5).with_k_i(0).with_k_d(0).with_k_s(8.12636).with_k_v(0).with_k_a(0)
        self.shootingMotor.configurator.apply(shootingConfiguration)

        self.turretOut = PositionTorqueCurrentFOC(0).with_slot(0)
        self.angleOut = PositionTorqueCurrentFOC(0).with_slot(0)
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

        self.shooterCalibrationData: list[dict[str, float]] = self.loadCalibrationData("/home/lvuser/py/tuning/shooterTable.csv")
        self.distances = [i['distance'] for i in self.shooterCalibrationData]
        self.columns = [k for k in self.shooterCalibrationData[0].keys() if k != 'distance']
            
        self.startConveyor()

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
        self.towerConveyor.set_control(DutyCycleOut(0.8, True))

    def stopConveyor(self):
        self.towerConveyor.stopMotor()

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

    def periodic(self) -> None:
        self.updateDistance(self.distTunable.get())
        self.rpmPub.set(self.shootingMotor.get_rotor_velocity().value * 60)
        self.hoodPub.set(self.hoodMotor.get_position().value)
        self.turretPub.set(self.turretMotor.get_position().value)