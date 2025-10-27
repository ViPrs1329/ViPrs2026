import commands2.button
from commands2 import SequentialCommandGroup
from commands2.button import CommandXboxController
from commands2.button import CommandJoystick
from commands2.sysid import SysIdRoutine

from pathplannerlib.auto import AutoBuilder
from pathplannerlib.auto import NamedCommands
from pathplannerlib.auto import PathPlannerAuto
from pathplannerlib.events import EventTrigger
from pathplannerlib.path import GoalEndState
from pathplannerlib.path import PathConstraints
from pathplannerlib.path import PathPlannerPath
from pathplannerlib.path import Waypoint

from wpimath.geometry import Pose2d
from wpimath.geometry import Rotation2d
from wpimath.geometry import Translation2d
from wpimath.kinematics import ChassisSpeeds
from wpimath.units import rotationsToRadians
from wpimath import units
from wpilib import SendableChooser
from wpilib import SmartDashboard
from wpilib import DriverStation
from wpilib import XboxController

from commands2 import Command
from commands2 import PrintCommand
from commands2 import InstantCommand
from commands2 import RunCommand
from commands2 import CommandScheduler
from commands2.button import CommandXboxController
from commands2.button import Trigger

from phoenix6 import swerve

from subsystems.DriveSubsystem import DriveSubsystem
from subsystems.krakenDriveSubsystem import CommandSwerveDrivetrain
from subsystems.LimelightSubsystem import LimelightSubsystem

from controlsSubsystemWrapper import SubsystemWrapper

from constants import Drive

from generated.tuner_constants import TunerConstants
from telemetry import Telemetry

class RobotContainer:
    """
    This class is where the bulk of the robot's resources 
    are declared. Here, subsystems are instantiated and commands 
    and button bindings are configured.
    """
    def __init__(self):
        #TODO add other subsystems as needed
        self.autoChooser: SendableChooser
        self.drivetrain: CommandSwerveDrivetrain
        self.limelight: LimelightSubsystem
        self.subsystemWrapper: SubsystemWrapper
        self.drivingController: CommandXboxController
        self.operatorController: CommandJoystick

        self.initSubsystems()
        self.initControls()
        self.initAutoChooser()
        self.initCommands()
        self.configureButtonBindings()

        self.maxSpeed = (
            TunerConstants.speed_at_12_volts
        )  # speed_at_12_volts desired top speed
        self.maxAngularRate = rotationsToRadians(
            0.75
        )  # 3/4 of a rotation per second max angular velocity

        self.drive = (
            swerve.requests.FieldCentric()
            .with_deadband(self.maxSpeed * 0.1)
            .with_rotational_deadband(
                self.maxAngularRate * 0.1
            )  # Add a 10% deadband
            .with_drive_request_type(
                swerve.SwerveModule.DriveRequestType.OPEN_LOOP_VOLTAGE
            )  # Use open-loop control for drive motors
        )
        self.brake = swerve.requests.SwerveDriveBrake()
        self.point = swerve.requests.PointWheelsAt()
        self.forwardStraight = (
            swerve.requests.RobotCentric()
            .with_drive_request_type(
                swerve.SwerveModule.DriveRequestType.OPEN_LOOP_VOLTAGE
            )
        )

        self._logger = Telemetry(self.maxSpeed)
        
    def initAutoChooser(self):
        self.autoChooser = AutoBuilder.buildAutoChooser("Autos")
        SmartDashboard.putData("Auto Chooser", self.autoChooser)

    def initSubsystems(self):
        """Instantiate the robot's subsystems."""
        
        # create subsystems
        # self.drivetrain = DriveSubsystem(self.fns.getOdometry)
        self.drivetrain = TunerConstants.create_drivetrain()
        self.limelight = LimelightSubsystem()
        
        #TODO add other subsystems as needed

        # create a wrapper for the subsystems
        self.subsystemWrapper = SubsystemWrapper(
            self.drivetrain, 
            self.limelight
            #TODO add other subsystems as needed
        )

        # register subsystems with the command scheduler
        #TODO register other subsystems as needed
        commands2.CommandScheduler.getInstance().registerSubsystem(self.drivetrain)
        commands2.CommandScheduler.getInstance().registerSubsystem(self.limelight)
        commands2.CommandScheduler.getInstance().registerSubsystem(self.subsystemWrapper)

    def initControls(self):
        """Instantiate the robot's control objects"""
        
        self.drivingController = CommandXboxController(0)
        self.operatorController = CommandJoystick(1)
        #TODO verify that these are the correct ports
        #TODO verify that we will need a xbox controller and a button board


    def initCommands(self):
        """Instantiate the robot's commands."""
        
        NamedCommands.registerCommand("marker1", PrintCommand("marker1"))
        NamedCommands.registerCommand("marker2", PrintCommand("marker2"))
        NamedCommands.registerCommand("Hello", PrintCommand("Hello"))
        #TODO add other commands as needed

    def configureButtonBindings(self):
        """Configure the button bindings for user input."""
               
        self.drivetrain.setDefaultCommand(
            self.drivetrain.apply_request(
                lambda: (
                    self.drive.with_velocity_x(
                        -self.drivingController.getLeftY() * self.maxSpeed
                    ) # Drive forward with negative Y (forward)
                    .with_velocity_y(
                        -self.drivingController.getLeftX() * self.maxSpeed
                    ) # DRive left with negative X (left)
                    .with_rotational_rate(
                        -self.drivingController.getRightX() * self.maxAngularRate
                    ) # Drive counterclockwise with negative X (left)
                )
            )
        )

        # Idle while the robot is diabled. This ensures the configured
        # neutral mode is applied to the drive motors while diabled.
        idle = swerve.requests.Idle()
        Trigger(DriverStation.isDisabled).whileTrue(
            self.drivetrain.apply_request(lambda: idle).ignoringDisable(True)
        )

        self.drivingController.a().whileTrue(self.drivetrain.apply_request(lambda: self.brake))
        self.drivingController.b().whileTrue(
            self.drivetrain.apply_request(
                lambda: self.point.with_module_direction(
                    Rotation2d(-self.drivingController.getLeftY(), -self.drivingController.getLeftX())
                )
            )
        )

        # Bind the reset gyro command to the X button on the controller
        self.drivingController.x().onTrue(
            self.drivetrain.runOnce(lambda: self.drivetrain.seed_field_centric())
        )

        self.drivetrain.register_telemetry(
            lambda state: self._logger.telemeterize(state)
        )

        #TODO add other button bindings as needed

    def getAutonomousCommand(self) -> Command:
        return self.autoChooser.getSelected()

    # not needed since commands2.Subsystem automatically 
    # calls the update function inside the subsystem 
    # each robot iteration

    # def updateHardware(self):
    #     """Call the update methods of each subsystem."""
    #     pass
