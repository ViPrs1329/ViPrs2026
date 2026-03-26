from commands2 import Command, Subsystem
from subsystems import shooterSubsystem, krakenDriveSubsystem
from phoenix6.swerve.requests import FieldCentric

class boilerCommand(Command):
    def __init__(self, shooter: shooterSubsystem.ShooterSubsystem, drivetrain: krakenDriveSubsystem.CommandSwerveDrivetrain, driveRequest: FieldCentric):
        super().__init__()
        super().addRequirements(shooter)
        self.shooter: shooterSubsystem.ShooterSubsystem = shooter
        self.drivetrain: krakenDriveSubsystem.CommandSwerveDrivetrain = drivetrain
        self.driveRequest: FieldCentric = driveRequest

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