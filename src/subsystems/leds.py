from ctre import CANdle, Animation
import wpilib

class LEDs:
    def __init__(self, can_id: int, led_count: int = 60):
        self.led_count = led_count
        self.candle = CANdle(can_id)

        config = CANdle.Configuration()
        config.stripType = CANdle.LEDStripType.GRB
        config.brightnessScalar = 0.8
        self.candle.configAllSettings(config)

    # -------- Basic colors --------
    def off(self):
        self.candle.setLEDs(0, 0, 0, 0, 0, self.led_count)

    def solid(self, r: int, g: int, b: int):
        self.candle.setLEDs(r, g, b, 0, 0, self.led_count)

    def green(self):
        self.solid(0, 255, 0)

    def red(self):
        self.solid(255, 0, 0)

    def blue(self):
        self.solid(0, 0, 255)

    # -------- Animations --------
    def rainbow(self):
        self.candle.animate(
            Animation.RainbowAnimation(1.0, 0.5, self.led_count)
        )

    def fire(self):
        self.candle.animate(
            Animation.FireAnimation(0.8, 0.7, self.led_count, 0.5, 0.5)
        )

    def clear_animation(self):
        self.candle.clearAnimation(0)

    # -------- Robot-state helpers --------
    def set_alliance_color(self):
        alliance = wpilib.DriverStation.getAlliance()
        if alliance == wpilib.DriverStation.Alliance.kRed:
            self.red()
        elif alliance == wpilib.DriverStation.Alliance.kBlue:
            self.blue()
        else:
            self.off()
