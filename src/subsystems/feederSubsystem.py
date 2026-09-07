from commands2 import Subsystem
from constants import Feeder
from phoenix6 import hardware, controls, configs, signals, StatusCode

class FeederSubsystem(Subsystem):
    def __init__(self):
        super().__init__()

        self.backConveyor = hardware.TalonFX(Feeder.CANids.backConveyor)
        backConveyorConfig = configs.TalonFXConfiguration()
        backConveyorConfig.with_current_limits(
            configs.CurrentLimitsConfigs()
            .with_stator_current_limit(20)
            .with_supply_current_limit(10)
        )
        self.backConveyor.configurator.apply(backConveyorConfig)

        self.leftConveyor = hardware.TalonFX(Feeder.CANids.leftConveyor)
        self.rightConveyor = hardware.TalonFX(Feeder.CANids.rightConveyor)
        mainConveyorConfig = configs.TalonFXConfiguration()
        mainConveyorConfig.with_current_limits(
            configs.CurrentLimitsConfigs()
            .with_stator_current_limit(40)
            .with_supply_current_limit(20)
        )
        self.leftConveyor.configurator.apply(mainConveyorConfig)
        self.rightConveyor.configurator.apply(mainConveyorConfig)

    def feedForward(self):
        self.backConveyor.set_control(controls.DutyCycleOut(Feeder.Consts.backConveyorDutyCycle, enable_foc=True))
        self.leftConveyor.set_control(controls.DutyCycleOut(Feeder.Consts.leftConveyorDutyCycle, enable_foc=True))
        self.rightConveyor.set_control(controls.DutyCycleOut(Feeder.Consts.rightConveyorDutyCycle, enable_foc=True))

    def stopFeed(self):
        self.backConveyor.stopMotor()
        self.leftConveyor.stopMotor()
        self.rightConveyor.stopMotor()

    def reverseFeed(self):
        self.backConveyor.set_control(controls.DutyCycleOut(-Feeder.Consts.backConveyorDutyCycle, enable_foc=True))
        self.leftConveyor.set_control(controls.DutyCycleOut(-Feeder.Consts.leftConveyorDutyCycle, enable_foc=True))
        self.rightConveyor.set_control(controls.DutyCycleOut(-Feeder.Consts.rightConveyorDutyCycle, enable_foc=True))
        