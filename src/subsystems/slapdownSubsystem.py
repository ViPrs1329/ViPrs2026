from commands2 import Subsystem
from constants import Slapdown
from phoenix6 import hardware, controls, configs, signals

class SlapdownSubsystem(Subsystem):
    def __init__(self) -> None:
        super().__init__()

        self.slapdownMotor = hardware.TalonFX(Slapdown.CANids.slapdownMotor)

        slapdownConfig = configs.TalonFXConfiguration()
        slapdownConfig.with_current_limits(
            configs.CurrentLimitsConfigs()
            .with_stator_current_limit(40)
            .with_supply_current_limit(20)
        )
        slapdownConfig.with_feedback(
            configs.FeedbackConfigs()
            .with_sensor_to_mechanism_ratio(Slapdown.Consts.gearRatio)
        )
        slapdownConfig.with_motion_magic(
            configs.MotionMagicConfigs()
            .with_motion_magic_cruise_velocity(1)
            .with_motion_magic_acceleration(2)
            .with_motion_magic_jerk(20)
        )
        # All gains start at 0 for manual tuning. GravityType is Arm_Cosine so that
        # once kG is tuned, feedforward scales with the cosine of the arm's angle from
        # horizontal: ~0 at vertical (top of stroke), maximal near horizontal (fully
        # extended).
        slapdownConfig.slot0.with_k_p(0).with_k_i(0).with_k_d(0).with_k_s(0).with_k_v(0).with_k_a(0).with_k_g(0).with_gravity_type(signals.GravityTypeValue.ARM_COSINE).with_static_feedforward_sign(signals.StaticFeedforwardSignValue.USE_CLOSED_LOOP_SIGN)
        self.slapdownMotor.configurator.apply(slapdownConfig)

        # Seed the relative encoder with the arm's true angle at power-on: stowed is
        # not horizontal, so the raw position must start offset rather than at 0.
        # Verify sign on the real robot -- position should read 0 as the arm passes
        # vertical (35 degrees into the stroke) for kG to be applied in the right
        # direction.
        self.slapdownMotor.set_position(Slapdown.Consts.stowedPosition)

    def extendIntake(self) -> None:
        self.slapdownMotor.set_control(controls.MotionMagicTorqueCurrentFOC(Slapdown.Consts.extendedPosition))

    def retractIntake(self) -> None:
        self.slapdownMotor.set_control(controls.MotionMagicTorqueCurrentFOC(Slapdown.Consts.stowedPosition))
