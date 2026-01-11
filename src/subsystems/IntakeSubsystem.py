from commands2 import Subsystem, Command, InstantCommand
import math
from wpimath import units
from phoenix6 import hardware, controls, configs, StatusCode
from constants import Intake

class IntakeSubsystem(Subsystem):
    def __init__(self) -> None:
        super().__init__()

        intakeMotor: hardware.TalonFX = hardware.TalonFX(Intake.Consts.intakeCANId)
        armMotor: hardware.TalonFX = hardware.TalonFX(Intake.Consts.armCANId)
        intakeMotorSpeed: controls.VelocityVoltage = controls.VelocityVoltage(0).with_enable_foc(True)
        armMotorPositioning: controls.PositionVoltage = controls.PositionVoltage(0).with_enable_foc(True)
        intakeMotor.set_control(intakeMotorSpeed)
        armMotor.set_control(armMotorPositioning)

        self.armDown: Command = InstantCommand(
            lambda:(armMotorPositioning.position = -0.25)
        )

        self.armUp: Command = InstantCommand(
            lambda:(armMotorPositioning.position = 0.25)
        )

        self.intakeFuel: Command = InstantCommand(
            lambda:(intakeMotorSpeed.velocity = 2.0)
        )

        """help me im going insane"""
        """how do i close parentheses"""