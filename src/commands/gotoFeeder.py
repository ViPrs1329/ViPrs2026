from commands2 import Command, Subsystem
from pathplannerlib.auto import AutoBuilder
from pathplannerlib.path import PathPlannerPath, PathConstraints

from subsystems.krakenDriveSubsystem import CommandSwerveDrivetrain

class GoToFeeder(Command):
    def __init__(self, subsystem: CommandSwerveDrivetrain):
        super().__init__()
        super().addRequirements(subsystem)
        self.subsystem: CommandSwerveDrivetrain = subsystem

    def initialize(self):
        # Code to initialize the command
        self.path: PathPlannerPath = PathPlannerPath.fromPathFile("Feeder")
        self.constraints: PathConstraints = PathConstraints(4.0, 3.0, 1.0, 0.1)
        self.followPathCommand: Command = AutoBuilder.pathfindThenFollowPath(self.path, self.constraints)
        self.followPathCommand.schedule()

    def execute(self):
        # Code to execute the command
        pass

    def isFinished(self):
        # Code to check if the command is finished
        return True

    def end(self, interrupted):
        # Code to end the command
        pass