from commands2 import Subsystem
from wpilib import SmartDashboard

from phoenix6.hardware import TalonFX, CANcoder
from phoenix6.controls import MotionMagicTorqueCurrentFOC, MotionMagicVelocityTorqueCurrentFOC
from phoenix6.configs import TalonFXConfiguration, CurrentLimitsConfigs, MotionMagicConfigs

from ntcore import NetworkTableInstance, NetworkTable, FloatPublisher

from constants import Shooter

import csv
import bisect

class ShooterSubsystem(Subsystem):
    def __init__(self) -> None:
        super().__init__()

        self.turretMotor: TalonFX = TalonFX(Shooter.Consts.turretId)
        self.hoodMotor: TalonFX = TalonFX(Shooter.Consts.hoodId)
        self.shootingMotor: TalonFX = TalonFX(Shooter.Consts.shootingId)
        self.turretEncoder: CANcoder = CANcoder(Shooter.Consts.turretEncoderId)

        turretConfiguration: TalonFXConfiguration = TalonFXConfiguration()
        turretConfiguration.with_current_limits(
            CurrentLimitsConfigs()
            .with_stator_current_limit(40)
            .with_supply_current_limit(20)
        )
        turretConfiguration.slot0.with_k_p(1).with_k_i(0).with_k_d(0).with_k_s(0)
        self.turretMotor.configurator.apply(turretConfiguration)

        hoodConfiguration: TalonFXConfiguration = TalonFXConfiguration()
        hoodConfiguration.with_current_limits(
            CurrentLimitsConfigs()
            .with_stator_current_limit(60)
            .with_supply_current_limit(30)
        ).with_motion_magic(
            MotionMagicConfigs()
            .with_motion_magic_cruise_velocity(50)
            .with_motion_magic_acceleration(40)
            .with_motion_magic_jerk(100)
        )
        hoodConfiguration.slot0.with_k_p(1).with_k_i(0).with_k_d(0).with_k_s(0)
        self.hoodMotor.configurator.apply(hoodConfiguration)

        shootingConfiguration: TalonFXConfiguration = TalonFXConfiguration()
        shootingConfiguration.with_current_limits(
            CurrentLimitsConfigs()
            .with_stator_current_limit(80)
            .with_supply_current_limit(40)
        )
        shootingConfiguration.slot0.with_k_p(1).with_k_i(0).with_k_d(0).with_k_s(0)
        self.shootingMotor.configurator.apply(shootingConfiguration)

        self.turretOut = MotionMagicTorqueCurrentFOC(0)
        self.angleOut = MotionMagicTorqueCurrentFOC(0)
        self.shooterOut = MotionMagicVelocityTorqueCurrentFOC(0)

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

        self.shooterCalibrationData = self.loadCalibrationData("src/tuning/shooterTable.csv")
        self.distances = [i['distance'] for i in self.shooterCalibrationData]
        self.columns = [k for k in self.shooterCalibrationData[0].keys() if k != 'distance']
            
    def angleTurret(self, position: float) -> None:
        self.turretOut.with_position(position)
        self.turretMotor.set_control(self.turretOut)
        self.targetTurretPub.set(position)

    def angleHood(self, position: float) -> None:
        self.angleOut.with_position(position)
        self.hoodMotor.set_control(self.angleOut)
        self.targetHoodPub.set(position)

    def setRPM(self, rpm: float) -> None:
        self.shooterOut.with_velocity(rpm / 60)
        self.shootingMotor.set_control(self.shooterOut)
        self.targetRPMPub.set(rpm)

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
        self.setRPM(calibration['targetRPM'])
        self.angleHood(calibration['hoodAngle'])

    def periodic(self) -> None:
        self.rpmPub.set(self.shootingMotor.get_rotor_velocity().value)
        self.hoodPub.set(self.hoodMotor.get_position().value)
        self.turretPub.set(self.turretMotor.get_position().value)