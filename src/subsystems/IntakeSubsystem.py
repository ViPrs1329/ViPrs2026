from commands2 import Subsystem, Command, InstantCommand
import math
from wpimath import units
from phoenix6 import hardware, controls, configs, signals, StatusCode
from constants import Intake
from tuning.tunable import TunableDouble

class IntakeSubsystem(Subsystem):
    def __init__(self) -> None:
        super().__init__()

        intakeConfig = configs.TalonFXConfiguration()
        intakeConfig.with_current_limits(
            configs.CurrentLimitsConfigs()
            .with_stator_current_limit(60)
            .with_supply_current_limit(30)
        )

        deployConfig = configs.TalonFXConfiguration()
        deployConfig.slot0.with_k_p(15).with_k_i(0).with_k_d(0).with_k_g(13).with_k_s(3).with_gravity_type(signals.GravityTypeValue.ELEVATOR_STATIC).with_static_feedforward_sign(signals.StaticFeedforwardSignValue.USE_CLOSED_LOOP_SIGN) # the arm position offset will be calculated by the design team
        deployConfig.with_current_limits(
            configs.CurrentLimitsConfigs()
            .with_stator_current_limit(60)
            .with_supply_current_limit(30)
        )
        self.intakeMotor: hardware.TalonFX = hardware.TalonFX(Intake.Consts.intakeCANId)
        self.deployMotor: hardware.TalonFX = hardware.TalonFX(Intake.Consts.armCANId)
        self.intakeMotorSpeed: controls.VelocityTorqueCurrentFOC = controls.VelocityTorqueCurrentFOC(0).with_slot(0)
        self.deployMotorPositioning: controls.PositionTorqueCurrentFOC = controls.PositionTorqueCurrentFOC(0).with_slot(0)

        self.deployMotor.setNeutralMode(signals.NeutralModeValue.BRAKE)
        self.intakeMotor.setNeutralMode(signals.NeutralModeValue.COAST)

        self.intakeMotor.configurator.apply(intakeConfig)
        self.deployMotor.configurator.apply(deployConfig)

        self.deployMotor.set_position(0) # tell the motor that this is the upright position

        self.intakeOutCommand: Command = InstantCommand(
            lambda: self.setIntakePosition(1) # this is in rotations, not radians, so design will tell us the gear ratio
        )

        self.intakeInCommand: Command = InstantCommand(
            lambda: self.setIntakePosition(0) # 0 because the arm starts up so it is zeroed there
        )

        self.intakeFuel: Command = InstantCommand(
            lambda: self.setIntakeSpeed(0.6) # rotations per second
        )

        self.stopIntakeFuel: Command = InstantCommand(
            lambda: self.setIntakeSpeed(0) # rotations per second
        )

        self.intakePosTunable = TunableDouble("Intake Position", 7.6, "Intake")
        self.intakePosPub = TunableDouble("Intake Report", 0, "Intake")

        self.counter = 0

    def extendIntake(self) -> None:
        self.setIntakePosition(7.6)

    def retractIntake(self) -> None:
        self.deployMotor.stopMotor()

    def startIntake(self):
        self.setIntakeSpeed(0.6)

    def reverseIntake(self):
        self.setIntakeSpeed(-0.6)

    def stopIntake(self):
        self.intakeMotor.stopMotor()

    def setIntakePosition(self, pos: float) -> None:
        self.deployMotor.set_control(self.deployMotorPositioning.with_position(pos))

    def periodic(self):
        # self.intakeMotor.set_control(self.intakeMotorSpeed)
        # self.setIntakePosition(self.intakePosTunable.get())

        self.counter += 1
        if self.counter == 50:
            self.intakePosPub.set(self.deployMotor.get_position().value)
            self.counter = 0

    def setIntakeSpeed(self, speed: float):
        self.intakeMotor.set_control(controls.DutyCycleOut(speed, True))
