from commands2 import Subsystem
from phoenix6 import hardware, configs, signals, controls

from constants import Climber

class ClimbingSubsystem(Subsystem):
    def __init__(self) -> None:
        """
        Subsystem responsible for managing the climbing mechanism of the robot.

        This subsystem controls the extension and retraction of the climbing arms,
        as well as monitoring the status of the climbing process.
        """
        # Initialize climbing mechanism components here
        
        super().__init__()

        self.climbingMotor: hardware.TalonFX = hardware.TalonFX(Climber.Consts.motorId)

        climberConfigs: configs.TalonFXConfiguration = configs.TalonFXConfiguration()
        # slot0 for pulling the robot up
        # slot1 for raising the climbing arm (there is no robot to lift so no feed forward)
        climberConfigs.slot0.with_k_p(1).with_k_i(0).with_k_d(0).with_k_g(0).with_gravity_type(signals.spn_enums.GravityTypeValue.ELEVATOR_STATIC)
        climberConfigs.slot1.with_k_p(1).with_k_i(0).with_k_d(0)
        
        self.climbingMotor.setNeutralMode(signals.NeutralModeValue.BRAKE)

        self.climbingMotor.configurator.apply(climberConfigs)

        self.climbingMotor.set_position(0.0)

        self.targetPosition: float = 0.0
        self.currentSlot: int = 1

    def switchToSlot(self, slot: int = 0):
        self.currentSlot = slot

    def periodic(self) -> None:
        positionRequest = controls.PositionTorqueCurrentFOC(position=self.targetPosition, slot=self.currentSlot)
        self.climbingMotor.set_control(positionRequest)
