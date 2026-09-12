from dataclasses import dataclass
from typing import Callable, Optional

import limelight
from commands2 import Subsystem
from wpilib import Timer
from wpimath.geometry import Pose2d, Rotation2d

from constants import Limelight as LimelightConstants


@dataclass(frozen=True)
class LimelightPoseEstimate:
    """A MegaTag2 robot pose estimate, ready to fuse into swerve odometry."""
    pose: Pose2d
    timestampSeconds: float
    tagCount: int
    avgTagDistanceMeters: float


class LimelightSubsystem(Subsystem):
    """Wraps a Limelight 4 (ethernet, limelightlib-python) running MegaTag2.

    MegaTag2 needs the robot's current heading fed in every loop, so the
    drivetrain's heading (and ideally its angular rate) must be supplied.
    """

    def __init__(
        self,
        headingSupplier: Callable[[], Rotation2d],
        headingRateDegreesPerSecondSupplier: Callable[[], float] = lambda: 0.0,
        hostname: str = LimelightConstants.Consts.hostname,
    ) -> None:
        super().__init__()
        self.headingSupplier = headingSupplier
        self.headingRateDegreesPerSecondSupplier = headingRateDegreesPerSecondSupplier

        self.camera = limelight.Limelight(hostname)
        self.camera.enable_websocket()

        self.latestEstimate: Optional[LimelightPoseEstimate] = None

    def periodic(self) -> None:
        heading = self.headingSupplier()
        self.camera.update_robot_orientation([
            heading.degrees(),
            self.headingRateDegreesPerSecondSupplier(),
            0.0, 0.0, 0.0, 0.0,
        ])
        self.latestEstimate = self._readMegaTag2Pose()

    def _readMegaTag2Pose(self) -> Optional[LimelightPoseEstimate]:
        rawResults = self.camera.get_latest_results()
        if rawResults is None:
            return None

        # limelightresults.GeneralResult doesn't expose MegaTag2, so this is read
        # straight from the raw payload: botpose_orb_wpiblue is
        # [x, y, z, roll, pitch, yaw, latencyMs, tagCount, tagSpan, avgTagDist, avgTagArea]
        # in meters/degrees, in the blue-alliance-origin field frame.
        botpose = rawResults.get("botpose_orb_wpiblue")
        if not botpose or len(botpose) < 10:
            return None

        tagCount = int(botpose[7])
        if tagCount == 0:
            return None

        latencySeconds = botpose[6] / 1000.0
        pose = Pose2d(botpose[0], botpose[1], Rotation2d.fromDegrees(botpose[5]))

        return LimelightPoseEstimate(
            pose=pose,
            timestampSeconds=Timer.getFPGATimestamp() - latencySeconds,
            tagCount=tagCount,
            avgTagDistanceMeters=botpose[9],
        )

    def getEstimatedPose(self) -> Optional[LimelightPoseEstimate]:
        """Latest MegaTag2 pose estimate, or None if no tags are currently visible.

        Fuse into swerve odometry with, e.g.:
            estimate = self.limelight.getEstimatedPose()
            if estimate is not None:
                self.drivetrain.add_vision_measurement(estimate.pose, estimate.timestampSeconds)
        """
        return self.latestEstimate
