from commands2 import Subsystem
from constants import Intake
from phoenix6 import hardware, controls, configs, signals, StatusCode

class IntakeSubsystem(Subsystem):
    def __init__(self):
        super().__init__()

        intakeConfig = configs.TalonFXConfiguration()
        intakeConfig.with_current_limits(
            configs.CurrentLimitsConfigs()
            .with_stator_current_limit(20)
            .with_supply_current_limit(10)
        )
        intakeConfig.slot0.with_k_p(0).with_k_i(0).with_k_d(0).with_k_s(0).with_k_v(0).with_k_a(0)

        self.leftIntakeMotor = hardware.TalonFX(Intake.CANids.leftIntakeMotor)
        self.rightIntakeMotor = hardware.TalonFX(Intake.CANids.rightIntakeMotor)

        self.leftIntakeMotor.configurator.apply(intakeConfig)
        self.rightIntakeMotor.configurator.apply(intakeConfig)

    def startIntake(self) -> None:
        self._setIntakeVelocity(Intake.Consts.intakeVelocity)

    def stopIntake(self) -> None:
        self.leftIntakeMotor.stopMotor()
        self.rightIntakeMotor.stopMotor()

    def reverseIntake(self) -> None:
        self._setIntakeVelocity(-Intake.Consts.intakeVelocity)

    def _setIntakeVelocity(self, velocity: float) -> None:
        self.leftIntakeMotor.set_control(controls.VelocityTorqueCurrentFOC(velocity).with_slot(0))
        self.rightIntakeMotor.set_control(controls.VelocityTorqueCurrentFOC(-velocity).with_slot(0))