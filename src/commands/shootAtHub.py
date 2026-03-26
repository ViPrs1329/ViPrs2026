from commands2 import Command, Subsystem
from ntcore import StructPublisher
from wpimath.geometry import Pose2d

from subsystems.shooterSubsystem import ShooterSubsystem
from subsystems.krakenDriveSubsystem import CommandSwerveDrivetrain
from phoenix6.swerve.requests import FieldCentric

class boilerCommand(Command):
    def __init__(self, shooter: ShooterSubsystem, drivetrain: CommandSwerveDrivetrain, driveRequest: FieldCentric):
        super().__init__()
        super().addRequirements(shooter)
        self.shooter: ShooterSubsystem = shooter
        self.drivetrain: CommandSwerveDrivetrain = drivetrain
        self.driveRequest: FieldCentric = driveRequest

        # self.ghostTarget: Pose2d = Pose2d()
        # self.ghostTargetPub: StructPublisher = self.shooter.shooterTable.getStructTopic("Ghost Target", Pose2d).publish()

    def initialize(self):
        # Code to initialize the command
        pass

    def execute(self):
        # Code to execute the command
        pass

    def isFinished(self):
        # Code to check if the command is finished
        return True

    def end(self, interrupted):
        # Code to end the command
        pass