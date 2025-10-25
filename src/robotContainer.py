import commands2.button
from commands2 import SequentialCommandGroup
from commands2.button import CommandXboxController
from commands2.button import CommandJoystick

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

from subsystems.DriveSubsystem import DriveSubsystem
from subsystems.LimelightSubsystem import LimelightSubsystem
from subsystems.fnsSubsystem import FnsSubsystem

from controlsSubsystemWrapper import SubsystemWrapper

from constants import Drive

class RobotContainer:
    """
    This class is where the bulk of the robot's resources 
    are declared. Here, subsystems are instantiated and commands 
    and button bindings are configured.
    """
    def __init__(self):
        #TODO add other subsystems as needed
        self.autoChooser: SendableChooser
        self.drivetrain: DriveSubsystem
        self.limelight: LimelightSubsystem
        self.fns: FnsSubsystem
        self.subsystemWrapper: SubsystemWrapper
        self.drivingController: CommandXboxController
        self.operatorController: CommandJoystick

        self.initSubsystems()
        self.initControls()
        self.initAutoChooser()
        self.initCommands()
        self.configureButtonBindings()
        
    def initAutoChooser(self):
        self.autoChooser = AutoBuilder.buildAutoChooser("Autos")
        SmartDashboard.putData("Auto Chooser", self.autoChooser)

    def initSubsystems(self):
        """Instantiate the robot's subsystems."""
        
        # create subsystems
        self.drivetrain = DriveSubsystem(self.fns.getOdometry)
        self.limelight = LimelightSubsystem()
        self.fns = FnsSubsystem(self.drivetrain.getPose, self.limelight.getRobotPositionFieldRelative, self.drivetrain.resetPose)
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
            RunCommand(
                lambda: self.drivetrain.controllerDrive(
                    -self.drivingController.getLeftY(),
                    -self.drivingController.getLeftX(),
                    -self.drivingController.getRightX()
                ),
                self.drivetrain
            )
        )

        # Bind the reset gyro command to the X button on the controller
        self.drivingController.x().onTrue(
            InstantCommand(
                lambda: self.drivetrain.rezeroGyro(),
                self.drivetrain
            )
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
