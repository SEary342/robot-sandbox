import math
from wpimath import units
from wpimath.geometry import Translation2d
from wpimath.kinematics import SwerveDrive4Kinematics
from rev import SparkBaseConfig, FeedbackSensor, SparkMaxConfig, AbsoluteEncoderConfig


class DriveConstants:
    kMaxSpeedMetersPerSecond = 2  # 4.8
    kMaxAngularSpeed = 1.5 * math.pi
    kMagnitudeSlewRate = 9.8
    kRotationalSlewRate = 24.0
    kInvertDirection = 1  # -1 inverts because math

    kTrackWidth = units.inchesToMeters(26.5)
    kWheelBase = units.inchesToMeters(26.5)

    kModulePositions = [
        Translation2d(kWheelBase / 2, kTrackWidth / 2),
        Translation2d(kWheelBase / 2, -kTrackWidth / 2),
        Translation2d(-kWheelBase / 2, kTrackWidth / 2),
        Translation2d(-kWheelBase / 2, -kTrackWidth / 2),
    ]
    kDriveKinematics = SwerveDrive4Kinematics(*kModulePositions)

    kFrontLeftDrivingCanId, kFrontLeftTurningCanId = 7, 8
    kRearLeftDrivingCanId, kRearLeftTurningCanId = 6, 5
    kFrontRightDrivingCanId, kFrontRightTurningCanId = 3, 4
    kRearRightDrivingCanId, kRearRightTurningCanId = 2, 1
    kGyroReversed = -1

    # Angular offsets of the modules relative to the chassis in radians
    kFrontLeftChassisAngularOffset = -math.pi / 2
    kFrontRightChassisAngularOffset = 0
    kBackLeftChassisAngularOffset = math.pi
    kBackRightChassisAngularOffset = math.pi / 2


class ModuleConstants:
    # --- Drive Settings ---
    kWheelDiameterMeters = 0.0762
    kDrivingMotorPinionTeeth = 13
    kDrivingMotorReduction = (45.0 * 22) / (kDrivingMotorPinionTeeth * 15)
    kDrivingEncoderPositionFactor = (
        kWheelDiameterMeters * math.pi
    ) / kDrivingMotorReduction
    kDrivingEncoderVelocityFactor = kDrivingEncoderPositionFactor / 60.0

    kDrivingP, kDrivingI, kDrivingD = 0.04, 0, 0
    kDrivingFF = 1.0 / (
        (5676 / 60 * (kWheelDiameterMeters * math.pi)) / kDrivingMotorReduction
    )
    kDrivingMotorCurrentLimit = 50
    kDrivingMinSpeedMetersPerSecond = 0.01

    kTurningMotorCurrentLimit = 20
    turningFactor = 2 * math.pi

    # --- Tuning Reference (Set these in REV Hardware Client) ---
    # Position Conversion Factor: 6.283185  (math.tau)
    # Velocity Conversion Factor: 0.104719  (math.tau / 60)
    # P: 1.0, I: 0.0, D: 0.0
    # Position Wrapping: Enabled (0 to 6.283185)


def getSwerveDrivingMotorConfig() -> SparkMaxConfig:
    drivingConfig = SparkMaxConfig()
    drivingConfig.setIdleMode(SparkMaxConfig.IdleMode.kCoast)
    drivingConfig.smartCurrentLimit(ModuleConstants.kDrivingMotorCurrentLimit)
    drivingConfig.encoder.positionConversionFactor(
        ModuleConstants.kDrivingEncoderPositionFactor
    )
    drivingConfig.encoder.velocityConversionFactor(
        ModuleConstants.kDrivingEncoderVelocityFactor
    )
    drivingConfig.closedLoop.setFeedbackSensor(FeedbackSensor.kPrimaryEncoder)
    drivingConfig.closedLoop.pid(
        ModuleConstants.kDrivingP, ModuleConstants.kDrivingI, ModuleConstants.kDrivingD
    )
    drivingConfig.closedLoop.velocityFF(ModuleConstants.kDrivingFF)
    drivingConfig.closedLoop.outputRange(-1, 1)
    return drivingConfig


def getSwerveTurningMotorConfig() -> SparkMaxConfig:

    turningConfig = SparkMaxConfig()
    turningConfig.setIdleMode(SparkBaseConfig.IdleMode.kBrake)
    turningConfig.smartCurrentLimit(ModuleConstants.kTurningMotorCurrentLimit)
    turningConfig.absoluteEncoder.velocityConversionFactor(
        ModuleConstants.turningFactor / 60.0
    ).apply(AbsoluteEncoderConfig.Presets.REV_ThroughBoreEncoderV2()).inverted(
        True
    ).positionConversionFactor(ModuleConstants.turningFactor)
    
    turningConfig.closedLoop.setFeedbackSensor(FeedbackSensor.kAbsoluteEncoder)
    turningConfig.closedLoop.pid(1, 0, 0).outputRange(-1, 1).positionWrappingEnabled(
        True
    ).positionWrappingInputRange(0, ModuleConstants.turningFactor)

    return turningConfig
