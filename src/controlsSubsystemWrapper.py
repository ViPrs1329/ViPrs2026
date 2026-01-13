from subsystems.krakenDriveSubsystem import CommandSwerveDrivetrain
from subsystems.LimelightSubsystem import LimelightSubsystem
from subsystems.IntakeSubsystem import IntakeSubsystem
import constants as Consts
from commands2 import InstantCommand
from commands2 import PrintCommand
from commands2 import SequentialCommandGroup
from commands2 import ParallelCommandGroup
from commands2 import Subsystem
from ntcore import NetworkTableInstance
from ntcore import NetworkTable
from ntcore import DoublePublisher
from ntcore import StringPublisher
from ntcore import BooleanPublisher
from ntcore import StructPublisher

from wpimath.geometry import Pose2d
from wpilib import Timer

class SubsystemWrapper(Subsystem):
    def __init__(self, drivetrain: CommandSwerveDrivetrain, limelight: LimelightSubsystem, intake: IntakeSubsystem):
        """
        Wrapper class that coordinates multiple subsystems to perform complex robot actions.

        This class provides a simplified interface for common robot operations by combining
        movements from multiple subsystems into single method calls.
        """        

        #TODO add other subsystems as needed
        self.drivetrain: CommandSwerveDrivetrain
        self.limelight: LimelightSubsystem
        self.intake: IntakeSubsystem
        self.resetBeforeTeleopCommand: SequentialCommandGroup
        self.resetSubsystemsCommand: SequentialCommandGroup
        self.nt: NetworkTableInstance
        self.logTable: NetworkTable
        self.driveRotationPub: StructPublisher
        self.targetVisiblePub: BooleanPublisher

        self.drivetrain = drivetrain
        self.limelight = limelight
        self.intake = intake

        self.resetBeforeTeleopCommand = SequentialCommandGroup(
            # Safety first - stop all motion
            # Move mechanisms to safe starting positions
            PrintCommand("Resetting subsystems for teleop...")
        )

        self.resetBeforeAutonomousCommand = SequentialCommandGroup(
            # Safety first - stop all motion
            # Move mechanisms to autonomous starting positions
            PrintCommand("Resetting subsystems for autonomous...")
        )

        self.resetSubsystemsCommand = SequentialCommandGroup(
            # Safety first - stop all motion
            # Move mechanisms to default positions
            PrintCommand("Resetting all subsystems...")
        )

        # Initialize NetworkTables logging
        self.nt = NetworkTableInstance.getDefault()
        self.logTable = self.nt.getTable("SubsystemWrapper")
        #TODO create tables for other subsystems as needed
        
        # Create persistent publishers
        self.driveRotationPub = self.logTable.getStructTopic("drive/odometry", Pose2d).publish()
        self.targetVisiblePub = self.logTable.getBooleanTopic("limelight/targetVisible").publish()
        #TODO add other publishers as needed
        
    def updateNetworkTables(self) -> None:
        """Update NetworkTables with current subsystem states"""
        
        # Drivetrain logging
        self.driveRotationPub.set(self.drivetrain.get_state().pose)
        
        # Limelight logging
        self.targetVisiblePub.set(self.limelight.canSeeTarget())
        
        # Log command states
        commandsTable = self.logTable.getSubTable("commands")
        commandsTable.putBoolean("resetCommand", self.resetSubsystemsCommand.isScheduled())

        #TODO add other subsystem logging as needed

    def periodic(self) -> None:
        """Called periodically, use for updating NetworkTables"""
        self.updateNetworkTables()

        visionRobotPose: Pose2d | None = self.limelight.getRobotPositionFieldRelative()
        if visionRobotPose is not None:
            self.drivetrain.add_vision_measurement(visionRobotPose, Timer.getFPGATimestamp(), Consts.Drive.Consts.visionMeasurementStdDevs)

    def resetSubsystems(self) -> None:
        """
        Reset all subsystems to their default states.
        This includes stopping all motion, zeroing encoders, and moving to default positions.
        """
        self.resetSubsystemsCommand.schedule()

    def resetBeforeTeleop(self) -> None:
        """
        Prepare robot for teleop operation.
        Moves to safe positions without zeroing sensors.
        """
        self.resetBeforeTeleopCommand.schedule()

    def resetBeforeAutonomous(self) -> None:
        """
        Prepare robot for autonomous operation.
        Moves to safe positions without zeroing sensors.
        """
        self.resetBeforeAutonomousCommand.schedule()
