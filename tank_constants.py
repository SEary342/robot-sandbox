# Tank Drive Constants
from wpimath.kinematics import DifferentialDriveKinematics

# The CAN IDs for the drivetrain motor controllers.
kLeftMotor1CAN = 7
kLeftMotor2CAN = 2
kRightMotor1CAN = 3
kRightMotor2CAN = 5

# Encoders and their respective motor controllers.
kLeftEncoderSign = +1
kRightEncoderSign = -1  # reversed

# In meters, distance between wheels on each side of robot.
kTrackWidthMeters = 0.69
kDriveKinematics = DifferentialDriveKinematics(kTrackWidthMeters)
kDriveSpeedAtMaxRPM = 5.0  # meters per second

# Encoder counts per revolution/rotation.
kEncoderCPR = 1024
kWheelDiameterMeters = 0.15

# Please calibrate to your robot
kEncoderPositionConversionFactor = 0.7

# Gyro config
kGyroReversed = -1  # make this +1 if not inverted

class DrivetrainConstants:
    initialP = 2.5 / 10000.0
    initialD = 5.0 / 10000.0
    initialFF = 1.4 / 10000.0
    maxRPM = 3000
