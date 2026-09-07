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

        self.leftIntakeMotor = hardware.TalonFX(Intake.CANids.leftIntakeMotor)
        self.rightIntakeMotor = hardware.TalonFX(Intake.CANids.rightIntakeMotor)

        self.leftIntakeMotor.configurator.apply(intakeConfig)
        self.rightIntakeMotor.configurator.apply(intakeConfig)

    def startIntake(self):
        self._setIntakeSpeed(Intake.Consts.intakeDutyCycle)

    def stopIntake(self):
        self._setIntakeSpeed(0)

    def reverseIntake(self):
        self._setIntakeSpeed(-Intake.Consts.intakeDutyCycle)

    def _setIntakeSpeed(self, dutyCycle: float):
        self.leftIntakeMotor.set_control(controls.DutyCycleOut(dutyCycle, enable_foc=True))
        self.rightIntakeMotor.set_control(controls.DutyCycleOut(-dutyCycle, enable_foc=True))