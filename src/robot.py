import time
import wpilib
from wpilib import Timer
import commands2
from robotContainer import RobotContainer

class MyRobot(commands2.TimedCommandRobot):

    def robotInit(self) -> None:
        """
        This function is called upon program startup and
        should be used for any initialization code.
        """

        self.robotContainer: RobotContainer
        self.autonomousCommand: commands2.Command | None

        try:
            self.robotContainer = RobotContainer()
            self.autonomousCommand = None
        except Exception as e:
            raise RuntimeError(f"Failed to initialize RobotContainer:\n{e}")
        self.lasttime = 0

    def robotPeriodic(self):
        commands2.CommandScheduler.getInstance().run()
        print(time.perf_counter() - self.lasttime)
        self.lasttime = time.perf_counter()

        
    def autonomousInit(self):
        """This function is run once each time the robot enters autonomous mode."""

        self.autonomousCommand = self.robotContainer.getAutonomousCommand()
        if self.autonomousCommand is not None:
            self.autonomousCommand.schedule()

    def autonomousPeriodic(self):
        """This function is called periodically during autonomous."""
        pass

    def autonomousExit(self) -> None:
        if self.autonomousCommand is not None:
            self.autonomousCommand.cancel()

        self.autonomousCommand = None

    def disabledInit(self):
        """This function is called initially when disabledd"""

    def disabledPeriodic(self):
        pass

    def teleopInit(self): 
        """This function is called once each time the robot enters teleoperated mode."""
        if self.autonomousCommand is not None:
            self.autonomousCommand.cancel()
            
        self.autonomousCommand = None
        
    def teleopPeriodic(self):
        """This function is called periodically during teleoperated mode."""
        
    def testInit(self): 
        """This function is called once each time the robot enters test mode."""
        
    def testPeriodic(self): 
        """This function is called periodically during test mode."""

    def simulationInit(self):
        pass

    def simulationPeriodic(self):
        """"This function is called periodically during the simulation mode"""
        


if __name__ == "__main__":
    wpilib.run(MyRobot)