from commands2 import Subsystem, Command, InstantCommand
import math
from wpimath import units
from phoenix6 import hardware, controls, configs, signals, StatusCode
from constants import Intake

class IntakeSubsystem(Subsystem):
    def __init__(self) -> None:
        super().__init__()

        intakeConfig = configs.TalonFXConfiguration()
        intakeConfig.slot0.with_k_p(1).with_k_i(0).with_k_d(0).with_k_s(0)
        intakeConfig.with_current_limits(
            configs.CurrentLimitsConfigs()
            .with_stator_current_limit(60)
            .with_supply_current_limit(30)
        )

        armConfig = configs.TalonFXConfiguration()
        armConfig.slot0.with_k_p(1).with_k_i(0).with_k_d(0).with_gravity_type(signals.GravityTypeValue.ARM_COSINE).with_gravity_arm_position_offset(0) # the arm position offset will be calculated by the design team
        armConfig.with_current_limits(
            configs.CurrentLimitsConfigs()
            .with_stator_current_limit(60)
            .with_supply_current_limit(30)
        )
        self.intakeMotor: hardware.TalonFX = hardware.TalonFX(Intake.Consts.intakeCANId)
        self.armMotor: hardware.TalonFX = hardware.TalonFX(Intake.Consts.armCANId)
        self.intakeMotorSpeed: controls.VelocityTorqueCurrentFOC = controls.VelocityTorqueCurrentFOC(0).with_slot(0)
        self.armMotorPositioning: controls.PositionVoltage = controls.PositionVoltage(0).with_enable_foc(True).with_slot(0)

        self.armMotor.setNeutralMode(signals.NeutralModeValue.BRAKE)
        self.intakeMotor.setNeutralMode(signals.NeutralModeValue.COAST)

        self.intakeMotor.configurator.apply(intakeConfig)
        self.armMotor.configurator.apply(armConfig)

        self.armMotor.set_position(0) # tell the motor that this is the upright position

        self.armDownCommand: Command = InstantCommand(
            lambda: self.setArmPosition(1) # this is in rotations, not radians, so design will tell us the gear ratio
        )

        self.armUpCommand: Command = InstantCommand(
            lambda: self.setArmPosition(0) # 0 because the arm starts up so it is zeroed there
        )

        self.intakeFuel: Command = InstantCommand(
            lambda: self.setIntakeSpeed(1) # rotations per second
        )

        self.stopIntakeFuel: Command = InstantCommand(
            lambda: self.setIntakeSpeed(0) # rotations per second
        )

    def periodic(self):
        self.intakeMotor.set_control(self.intakeMotorSpeed)
        self.armMotor.set_control(self.armMotorPositioning)

    def setArmPosition(self, position: float):
        self.armMotorPositioning.with_position(position)

    def setIntakeSpeed(self, speed: float):
        self.intakeMotorSpeed.with_velocity(speed)
