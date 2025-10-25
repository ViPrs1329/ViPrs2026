from typing import Callable

from commands2 import Subsystem

from wpimath.geometry import Pose2d, Rotation2d, Translation2d

from constants import FNS

class FnsSubsystem(Subsystem):
    def __init__(self, odometrySupplier: Callable[[], Pose2d], cameraSupplier: Callable[[], Pose2d | None], odometryReset: Callable[[Pose2d], None]) -> None:
        super().__init__()
        
        self.odometrySupplier: Callable[[], Pose2d] = odometrySupplier
        self.cameraSupplier: Callable[[], Pose2d | None] = cameraSupplier
        self.odometryReset: Callable[[Pose2d], None] = odometryReset

    def resetOdometry(self, pose: Pose2d = Pose2d(Translation2d(0, 0), Rotation2d(0))) -> None:
        """Reset the robot's odometry to a specified pose."""
        self.odometryReset(pose)

    def getWheelOdometry(self) -> Pose2d:
        """ Get the odometry based on wheel encoders and gyro. """
        return self.odometrySupplier()

    def getCameraOdometry(self) -> Pose2d | None:
        """ Get the odometry based on vision camera. """
        return self.cameraSupplier()

    def getOdometry(self) -> Pose2d:
        """ Get the combined odometry from both wheel and camera sources.
        If both sources are available, it averages them. If only one is available, it returns that one.
        If neither is available, it returns None.
        """
        wheelOdometry: Pose2d = self.odometrySupplier()
        cameraOdometry: Pose2d | None = self.cameraSupplier()

        if wheelOdometry is None and cameraOdometry is None:
            return None
        if wheelOdometry is None:
            return cameraOdometry
        if cameraOdometry is None:
            return wheelOdometry
        
        if wheelOdometry.translation().distance(cameraOdometry.translation()) < FNS.Consts.visionWheelTranslationPoseTolerance:
            avgTranslation: Translation2d = (wheelOdometry.translation() + cameraOdometry.translation()) / 2
            if wheelOdometry.rotation().radians() - cameraOdometry.rotation().radians() < FNS.Consts.visionWheelRotationPoseTolerance:
                avgRotation: Rotation2d = Rotation2d((wheelOdometry.rotation().radians() + cameraOdometry.rotation().radians()) / 2)
                return Pose2d(avgTranslation, avgRotation)
            
            else:
                self.resetOdometry(cameraOdometry)
                return cameraOdometry
            
        else:
            self.resetOdometry(cameraOdometry)
            return cameraOdometry