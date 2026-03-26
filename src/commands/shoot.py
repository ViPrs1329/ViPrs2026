from commands2 import Command, Subsystem
from ntcore import StructPublisher
from wpimath.geometry import Pose2d
from wpimath.kinematics import ChassisSpeeds
from wpimath.controller import PIDController
from wpimath.filter import LinearFilter

from subsystems.shooterSubsystem import ShooterSubsystem
from subsystems.krakenDriveSubsystem import CommandSwerveDrivetrain
from phoenix6.swerve.requests import FieldCentric

class Shoot(Command):
    def __init__(self, shooter: ShooterSubsystem, drivetrain: CommandSwerveDrivetrain):
        super().__init__()
        super().addRequirements(shooter)
        self.shooter: ShooterSubsystem = shooter
        self.drivetrain: CommandSwerveDrivetrain = drivetrain
        # self.driveRequest: FieldCentric = driveRequest

        # self.ghostTarget: Pose2d = Pose2d()
        # self.ghostTargetPub: StructPublisher = self.shooter.shooterTable.getStructTopic("Ghost Target", Pose2d).publish()

    def initialize(self):
        # Code to initialize the command
        self.pid = PIDController(0.1, 0, 0)
        self.filter = LinearFilter.singlePoleIIR(0.1, 0.02)
        self.shooter.startConveyor()

    def execute(self):
        # Code to execute the command
        self.shooter.updateDistance(self.drivetrain.get_state().pose, self.drivetrain.get_state().speeds, True)

        # self.drivetrain.apply_request(
        #     lambda: (
        #         self.driveRequest.with_velocity_x(
        #             self.speedsReporter().vx
        #         ) # Drive forward with negative Y (forward)
        #         .with_velocity_y(
        #             self.speedsReporter().vy
        #         ) # DRive left with negative X (left)
        #         .with_rotational_rate(
        #             self.pid.calculate(self.drivetrain.get_state().pose.rotation().radians)
        #         ) # Drive counterclockwise with negative X (left)
        #     )
        # )

    def isFinished(self):
        # Code to check if the command is finished
        return False

    def end(self, interrupted):
        # Code to end the command
        self.shooter.angleHood(0)
        self.shooter.stopConveyor()