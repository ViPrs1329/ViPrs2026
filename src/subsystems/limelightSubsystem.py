from typing import NamedTuple, Optional

from commands2 import Subsystem
from ntcore import DoubleArraySubscriber, NetworkTableInstance
from wpilib import Timer
from wpimath.geometry import Pose2d, Rotation2d

from constants import Limelight


class LimelightPoseEstimate(NamedTuple):
    pose: Pose2d
    timestampSeconds: float
    tagCount: int


class LimelightSubsystem(Subsystem):
    def __init__(self, tableName: str = Limelight.Consts.tableName) -> None:
        super().__init__()

        table = NetworkTableInstance.getDefault().getTable(tableName)
        # botpose_wpiblue: [x, y, z, roll, pitch, yaw, latency(ms), tagCount, tagSpan, avgTagDist, avgTagArea]
        # in meters/degrees, field-relative with the blue alliance wall as the origin.
        self.botposeSubscriber: DoubleArraySubscriber = table.getDoubleArrayTopic(
            "botpose_wpiblue"
        ).subscribe([])

        self.latestEstimate: Optional[LimelightPoseEstimate] = None

    def periodic(self) -> None:
        self.latestEstimate = self._readPose()

    def _readPose(self) -> Optional[LimelightPoseEstimate]:
        botpose = self.botposeSubscriber.get()
        if len(botpose) < 8:
            return None

        tagCount = int(botpose[7])
        if tagCount == 0:
            return None

        latencySeconds = botpose[6] / 1000.0
        pose = Pose2d(botpose[0], botpose[1], Rotation2d.fromDegrees(botpose[5]))
        timestampSeconds = Timer.getFPGATimestamp() - latencySeconds

        return LimelightPoseEstimate(pose=pose, timestampSeconds=timestampSeconds, tagCount=tagCount)

    def getEstimatedPose(self) -> Optional[LimelightPoseEstimate]:
        """Latest robot pose in the blue-origin field frame, or None if no tags are visible."""
        return self.latestEstimate
