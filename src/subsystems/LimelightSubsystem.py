from commands2 import Subsystem
from ntcore import NetworkTableInstance
from ntcore import NetworkTable

from wpimath.geometry import Pose2d
from wpimath.geometry import Rotation2d
from wpimath.geometry import Transform2d

from constants import Limelight

class LimelightSubsystem(Subsystem):
    def __init__(self) -> None:
        super().__init__()
        self.tables: list[NetworkTable]
        
        self.tables = []
        for name in Limelight.Consts.tableNames:
            table: NetworkTable = NetworkTableInstance.getDefault().getTable(name)
            self.tables.append(table)

    def canSeeTarget(self) -> bool:
        """
        Check if the limelight can see a target.
        """
        return any(table.getNumber("tv", 0) == 1 for table in self.tables)
    
    def getRobotPositionFieldRelative(self) -> Pose2d | None:
        """
        Get the robot's position relative to the field.

        This method averages the robot's position across all limelight tables using a weighted average biased towards bigger tag area.
        
        If no targets are visible, it returns None.
        """
        if not self.canSeeTarget():
            return None
        
        robotPose: Pose2d = Pose2d(0, 0, Rotation2d(0))

        totalWeight: float = 0
        # Iterate through all tables to accumulate the robot's position
        for table in self.tables:
            
            # botpose_orb uses MT2 compared to botpose which uses MT1
            botPose = table.getNumberArray("botpose_orb", [0, 0, 0, 0, 0, 0]) 
            if table.getNumber("tv", 0) == 1:
                weight: float = table.getNumber("ta", 0)
                totalWeight += weight
                x = botPose[0]
                y = botPose[1]
                rotation = Rotation2d.fromDegrees(botPose[5]) * weight
                robotPose += Transform2d(x, y, rotation) * weight # yes, this is a Transform2d, not a Pose2d
        
        if totalWeight > 0:
            robotPose /= totalWeight
            return robotPose
        else:
            return None
    
    def getRobotPositionRelativeToTarget(self, target: int) -> Pose2d | None:
        """
        Get the robot's position relative to the specified targets.
        This method averages the robot's position across all limelight tables for the specified target using a weighted average biased towards bigger tag area.
        If no targets are visible, it returns None.
        
        :param targets: List of target indices to consider.
        :return: Pose2d representing the robot's position relative to the targets.
        """
        if not self.canSeeTarget():
            return Pose2d(0, 0, Rotation2d(0))
        
        robotPose: Pose2d = Pose2d(0, 0, Rotation2d(0))

        totalWeight: float = 0

        # Iterate through all tables to accumulate the robot's position
        for table in self.tables:
            if table.getNumber("tv", 0) == 1 and table.getNumber("tid", -1) == target:
                weight = table.getNumber("ta", 0)
                totalWeight += weight
                botPose = table.getNumberArray("botpose_targetspace", [0, 0, 0, 0, 0, 0])
                x = botPose[0]
                z = botPose[2]
                rotation = Rotation2d.fromDegrees(botPose[5]) * weight
                robotPose += Transform2d(x, z, rotation) * weight

        if totalWeight > 0:
            robotPose /= totalWeight
            return robotPose
        else:
            return None
        