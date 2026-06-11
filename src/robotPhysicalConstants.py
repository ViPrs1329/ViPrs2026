import math

class DrivePhysicalConstants:
    # user input here
    robotMass: float = 68.0 # kg
    JTurret: float = 0.00091777 + 0.0004162041 + 0.0039357551 # kg m^2
    
    wheelRadius: float = 0.0508 # meters

    driveCurrentLimit: float = 30.0 # A
    steerCurrentLimit: float = 80.0 # A

    driveKP: float = 10 # A/(rot/s)
    driveKI: float = 0.1 # A/rot
    driveKD: float = 0.01 # A/(rot/s^2)
    driveKV: float = 0.01 # A/(rot/s)
    driveKS: float = 2.5 # A

    steerKP: float = 50.0 # A/rot
    steerKI: float = 0.0 # A/(rot*s)
    steerKD: float = 0.0 # A/(rot/s)
    steerKV: float = 0.0 # A/(rot/s)
    steerKS: float = 0.22 # A

    driveGearRatio: float = 6.746031746031747
    steerGearRatio: float = 21.428571428571427

    driveEfficiency: float = 0.92
    steerEfficiency: float = 0.95

    ktDrive: float = 0.0197 # Nm/A
    ktSteer: float = 0.0197 # Nm/A
    JDrive: float = 0.005
    JSteer: float = 0.005

    # calculated from user input
    driveJRobot: float = robotMass * wheelRadius * wheelRadius # kg m^2 # The equivalent rotational inertia (J_robot) felt at the wheel axles due to the linear mass of the robot
    driveJMotor: float = driveJRobot / (4 * driveGearRatio * driveGearRatio * driveEfficiency) # kg m^2 per motor # the approximate equivalent rotational inertia of the motor
    driveKARad: float = driveJMotor / ktDrive # A/(rad/s^2)
    driveKA: float = driveKARad * 2 * math.pi # A/(rot/s^2)
    driveMotionMagicAcceleration: float = (driveCurrentLimit - driveKS) / driveKA # (rot/s^2) # the approximate maximum acceleration of the motor
    
    steerJMotor: float = JTurret / (steerGearRatio * steerGearRatio * steerEfficiency)
    steerKARad: float = steerJMotor / ktSteer # A/(rad/s^2)
    steerKA: float = steerKARad * 2 * math.pi + JSteer # A/(rot/s^2)
    steerMotionMagicAcceleration: float = (steerCurrentLimit - steerKS) / steerKA # (rot/s^2) # the approximate maximum acceleration of the motor
    
print(DrivePhysicalConstants.steerKA)

