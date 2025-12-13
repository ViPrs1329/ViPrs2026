import wpilib.simulation
from pyfrc.physics.core import PhysicsEngine
from pyfrc.physics.core import PhysicsInterface

from wpimath.geometry import Pose2d
from wpilib import Field2d

from robot import MyRobot

class Physics(PhysicsEngine):
    def __init__(self, physicsController, robot):
        self.physicsController: PhysicsInterface = physicsController
        self.robot: MyRobot = robot

        # set up field visualization
        self.field: Field2d = Field2d()
        wpilib.SmartDashboard.putData("Field", self.field)

    def update_sim(self, now: float, deltaT: float) -> None:
        """
        Updates the 2D field visualization
        The actual motor physics are handled inside krakenDriveSubsystem.py
        """
        # pull the pose from the drivetrain
        pose: Pose2d = self.robot.robotContainer.drivetrain.get_state().pose

        self.field.setRobotPose(pose)