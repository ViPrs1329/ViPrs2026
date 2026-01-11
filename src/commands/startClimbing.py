from commands2 import Command, Subsystem

class StartClimbing(Command):
    def __init__(self, subsystem: Subsystem):
        super().__init__()
        super().addRequirements(subsystem)
        self.subsystem: Subsystem = subsystem

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