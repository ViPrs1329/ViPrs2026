# DriveSubsystem.py
#
# 

from pathplannerlib.auto import AutoBuilder
from pathplannerlib.config import RobotConfig
from pathplannerlib.util import DriveFeedforwards
from pathplannerlib.controller import PPHolonomicDriveController

from wpimath.geometry import Pose2d
from wpimath.geometry import Rotation2d
from wpimath.geometry import Translation2d
from wpimath.kinematics import ChassisSpeeds
from wpimath.kinematics import SwerveDrive4Kinematics
from wpimath.kinematics import SwerveDrive4Odometry
from wpimath.kinematics import SwerveModulePosition
from wpimath.kinematics import SwerveModuleState
from wpilib import DriverStation
from wpilib import Field2d
from wpilib import SmartDashboard

from phoenix6.hardware import Pigeon2
from phoenix6.hardware import CANcoder
from phoenix6.configs import CANcoderConfiguration
from phoenix6.configs import MagnetSensorConfigs

from rev import SparkMax
from rev import SparkBaseConfig
from rev import SparkBase
from rev import SparkClosedLoopController
from rev import ClosedLoopConfig
from rev import ClosedLoopSlot

from commands2 import InstantCommand
from commands2 import ParallelCommandGroup
from commands2 import SequentialCommandGroup
from commands2 import WaitCommand

from commands2 import Subsystem
from constants import CANIDs
from constants import Drive
from constants import Input

from math import pi

from typing import Callable

class SwerveModule:

    def __init__(self, driveMotorID: int, rotMotorID: int, rotEncoderID: int) -> None:
        self.currentPosition: SwerveModulePosition = SwerveModulePosition()
        self.currentState: SwerveModuleState = SwerveModuleState()

        # create the motors and encoder
        self.driveMotor: SparkMax = SparkMax(driveMotorID, SparkMax.MotorType.kBrushless)
        self.rotMotor: SparkMax = SparkMax(rotMotorID, SparkMax.MotorType.kBrushless)
        self.encoder: CANcoder = CANcoder(rotEncoderID)

        # configure the motors
        self.slot: ClosedLoopSlot = ClosedLoopSlot(0)

        self.driveController: SparkClosedLoopController = self.driveMotor.getClosedLoopController()
        self.driveController.setReference(0, SparkBase.ControlType.kVelocity, self.slot)

        driveConfig: SparkBaseConfig = SparkBaseConfig()
        driveConfig.setIdleMode(SparkBaseConfig.IdleMode.kBrake)
        driveConfig.smartCurrentLimit(Drive.Consts.driveCurrentLimit)
        driveConfig.closedLoopRampRate(Drive.Consts.rampRate)

        driveConfig.closedLoop.pid(Drive.Consts.driveP, Drive.Consts.driveI, Drive.Consts.driveD, self.slot)
        driveConfig.closedLoop.setFeedbackSensor(ClosedLoopConfig.FeedbackSensor.kPrimaryEncoder)
        driveConfig.closedLoop.positionWrappingEnabled(False)
        driveConfig.encoder.positionConversionFactor(2 * pi * Drive.Consts.wheelRadius / Drive.Consts.driveGearRatio)  # rot → meters
        driveConfig.encoder.velocityConversionFactor(2 * pi * Drive.Consts.wheelRadius / (Drive.Consts.driveGearRatio * 60))  # rot/m → meters/s

        self.driveMotor.configure(driveConfig, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters)

        self.rotController: SparkClosedLoopController = self.rotMotor.getClosedLoopController()
        self.rotController.setReference(0, SparkBase.ControlType.kPosition, self.slot)
        
        rotConfig: SparkBaseConfig = SparkBaseConfig()
        rotConfig.setIdleMode(SparkBaseConfig.IdleMode.kBrake)
        rotConfig.smartCurrentLimit(Drive.Consts.rotCurrentLimit)
        rotConfig.encoder.positionConversionFactor(2 * pi / Drive.Consts.rotGearRatio)  # rot → rad
        rotConfig.encoder.velocityConversionFactor((2 * pi) / (Drive.Consts.rotGearRatio * 60))  # RPM → rad/s

        rotConfig.closedLoop.pid(Drive.Consts.rotP, Drive.Consts.rotI, Drive.Consts.rotD, self.slot)
        rotConfig.closedLoop.setFeedbackSensor(ClosedLoopConfig.FeedbackSensor.kPrimaryEncoder)
        rotConfig.closedLoop.positionWrappingEnabled(True)
        rotConfig.closedLoop.positionWrappingInputRange(-pi, pi)
        self.rotMotor.configure(rotConfig, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters)
    
    def getPosition(self) -> SwerveModulePosition:
        return self.currentPosition
    
    def getState(self) -> SwerveModuleState:
        return self.currentState
    
    def setTargetState(self, targetState: SwerveModuleState) -> None:
        # optimize the state
        targetState.optimize(self.currentState.angle)
        self.currentState = targetState

        # 0.02 is 50hz = rate at which main control loop runs
        self.currentPosition = SwerveModulePosition(
            self.currentPosition.distance + (self.currentState.speed * 0.02),
            self.currentState.angle
        )
    
    def update(self):
        # update the motor speeds
        self.rotController.setReference(self.currentState.angle.radians(), SparkBase.ControlType.kPosition, self.slot)
        self.driveController.setReference(self.currentState.speed, SparkBase.ControlType.kVelocity, self.slot)

        # update the encoder position
        if abs(self.rotMotor.getEncoder().getVelocity()) < 0.1:
            self.rotMotor.getEncoder().setPosition(2 * pi * self.encoder.get_position().value_as_double)

        elif abs(self.rotMotor.getEncoder().getPosition() - 2 * pi * self.encoder.get_position().value_as_double) > 0.1:
            self.rotMotor.getEncoder().setPosition(2 * pi * self.encoder.get_position().value_as_double)

class DriveSubsystem(Subsystem):

    def __init__(self, combinedPoseSupplier: Callable[[], Pose2d]) -> None:
        super().__init__()

        self.combinedPoseSupplier: Callable[[], Pose2d] = combinedPoseSupplier

        self.gyro: Pigeon2
        self.field: Field2d
        self.modules: list[SwerveModule]
        self.kinematics: SwerveDrive4Kinematics
        self.odometry: SwerveDrive4Odometry
        self.config: RobotConfig
        self.driveState: int
        self.scoreCoralCommand: SequentialCommandGroup
        self.scoreAlgaeCommand: SequentialCommandGroup

        # Initialize swerve modules and other things...

        self.gyro = Pigeon2(CANIDs.pigeon)
        self.field = Field2d()
        self.modules = [
            SwerveModule(CANIDs.flDrive, CANIDs.flRotation, CANIDs.flEncoder),
            SwerveModule(CANIDs.frDrive, CANIDs.frRotation, CANIDs.frEncoder),
            SwerveModule(CANIDs.blDrive, CANIDs.blRotation, CANIDs.blEncoder),
            SwerveModule(CANIDs.brDrive, CANIDs.brRotation, CANIDs.brEncoder)
        ]
        self.kinematics = SwerveDrive4Kinematics(
            Drive.Consts.flModuleOffset,
            Drive.Consts.frModuleOffset,
            Drive.Consts.blModuleOffset,
            Drive.Consts.brModuleOffset
        )
        self.odometry = SwerveDrive4Odometry(
            self.kinematics,
            self.gyro.getRotation2d(),
            self.getPositions()
        )
        self.configureAutoBuilder()

        self.driveState = Drive.States.autonomous

    #TODO add other methods as needed

    def switchToTeleop(self) -> None:
        """
        Switch the drive state to teleop.
        This is called when the robot is in teleop mode.
        """
        self.driveState = Drive.States.teleop

    def switchToAutonomous(self) -> None:
        """
        Switch the drive state to autonomous.
        This is called when the robot is in autonomous mode.
        """
        self.driveState = Drive.States.autonomous

    def configureAutoBuilder(self) -> None:
        """Configure the PathPlanner AutoBuilder for autonomous path following."""
        try:
            self.config = RobotConfig.fromGUISettings()
            AutoBuilder.configure(
                self.combinedPoseSupplier,
                self.resetPose,
                self.getSpeeds,
                self.driveRobotRelative,
                PPHolonomicDriveController(
                    Drive.Consts.translationConstants,
                    Drive.Consts.rotationConstants
                ),
                self.config,
                self.shouldFlipPath,
                self
            )
        except Exception as inst:
            print("Failed to load PathPlanner config and configure AutoBuilder", inst)
        
        SmartDashboard.putData("Field", self.field)

    def periodic(self):
        self.updateSpeeds()
        self.odometry.update(self.gyro.getRotation2d(), self.getPositions())
        self.field.setRobotPose(self.getPose())

    def getPose(self) -> Pose2d:
        """Get the current pose of the robot."""
        return self.odometry.getPose()
    
    def resetPose(self, pose: Pose2d) -> None:
        """Reset the robot's pose to the specified pose."""
        print(pose)
        self.gyro.set_yaw(pose.rotation().degrees())
        self.field.setRobotPose(pose)
        self.odometry.resetPosition(
            self.gyro.getRotation2d(),
            self.getPositions(),
            pose
        )

    def rezeroGyro(self) -> None:
        """Re-zero the gyro to 0 degrees."""
        self.gyro.set_yaw(0)

    def getSpeeds(self) -> ChassisSpeeds:
        """Get the current chassis speeds of the robot."""
        return self.kinematics.toChassisSpeeds(self.getModuleStates())
    
    def driveFieldRelative(self, fieldRelativeSpeeds) -> None:
        """Drive the robot using field-relative speeds."""
        self.driveRobotRelative(
            ChassisSpeeds.fromFieldRelativeSpeeds(
                fieldRelativeSpeeds,
                self.getPose().rotation()
            )
        )

    def scalingFunction(self, x: float) -> float:
        """
        Applies a scaling function to the input to adjust sensitivity.
        This makes the controls more sensitive at low speeds and less sensitive at high speeds.
        """
        # Applies a scaling function to the input
        # to make it more sensitive at low speeds
        # and less sensitive at high speeds
        if abs(x) < Drive.Consts.inputDeadzone:
            return 0
        else:
            if x > 0:
                return x * x
            else:
                return -x * x

    def controllerDrive(self, vx, vy, omega) -> None:
        """Drive the robot using controller inputs."""
        if self.driveState == Drive.States.teleop:
            self.driveFieldRelative(
                ChassisSpeeds(
                    self.scalingFunction(vx) * Drive.Consts.maxSpeed,
                    self.scalingFunction(vy) * Drive.Consts.maxSpeed,
                    omega * Drive.Consts.maxAngularSpeed
                )
            )

    def stopDrive(self) -> None:
        """Stop all robot motion."""
        self.driveRobotRelative(
            ChassisSpeeds(0, 0, 0)
        )

    def driveRobotRelative(self, robotRelativeSpeeds: ChassisSpeeds, ff: DriveFeedforwards | None = None) -> None:
        """Drive the robot using robot-relative speeds."""
        targetSpeeds: ChassisSpeeds = ChassisSpeeds.discretize(robotRelativeSpeeds, 0.02)
        targetStates: tuple[SwerveModuleState, SwerveModuleState, SwerveModuleState, SwerveModuleState] = self.kinematics.toSwerveModuleStates(targetSpeeds)
        self.setStates(targetStates)

    def setStates(self, targetStates: tuple[SwerveModuleState, SwerveModuleState, SwerveModuleState, SwerveModuleState]) -> None:
        """Set the target states for all swerve modules."""
        targetStates = SwerveDrive4Kinematics.desaturateWheelSpeeds(targetStates, Drive.Consts.maxModuleSpeed)
        for i in range(len(self.modules)):
            self.modules[i].setTargetState(targetStates[i])

    def getModuleStates(self) -> tuple[SwerveModuleState, SwerveModuleState, SwerveModuleState, SwerveModuleState]:
        """Get the current states of all swerve modules."""
        states = []
        for i in range(len(self.modules)):
            states.append(self.modules[i].getState())

        return tuple(states)
    
    def getPositions(self) -> tuple[SwerveModulePosition, SwerveModulePosition, SwerveModulePosition, SwerveModulePosition]:
        """Get the current positions of all swerve modules."""
        positions = []
        for i in range(len(self.modules)):
            positions.append(self.modules[i])

        return tuple(positions)
    
    def shouldFlipPath(self) -> bool:
        """Determine if the autonomous path should be flipped based on alliance color."""
        alliance = DriverStation.getAlliance()
        if alliance == DriverStation.Alliance.kRed:
            return True
        else:
            return False
        
    # method that sets the speeds for the motors
    def updateSpeeds(self) -> None:
        """ Update the speeds of all swerve modules. """
        for module in self.modules:
            module.update()
            
    # def updateHardware(self):
    #     # This method gets called periodically to update hardware state
    #     pass

    # def cacheSensors(self):
    #     # This is called periodically to cache sensor data
    #     # so that we don't clog up the CAN network
    #     pass