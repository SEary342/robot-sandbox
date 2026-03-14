import math
from wpimath import units
from wpimath.geometry import Translation2d
from wpimath.kinematics import SwerveDrive4Kinematics
from rev import SparkBaseConfig, FeedbackSensor

class NeoMotorConstants:
    kFreeSpeedRpm = 5676

class DriveConstants:
    kMaxSpeedMetersPerSecond = 4.8
    kMaxAngularSpeed = 1.5 * math.tau

    kMagnitudeSlewRate = 9.8
    kRotationalSlewRate = 24.0

    kTrackWidth = units.inchesToMeters(26.5)
    kWheelBase = units.inchesToMeters(26.5)

    kModulePositions = [
        Translation2d(kWheelBase / 2, kTrackWidth / 2),
        Translation2d(kWheelBase / 2, -kTrackWidth / 2),
        Translation2d(-kWheelBase / 2, kTrackWidth / 2),
        Translation2d(-kWheelBase / 2, -kTrackWidth / 2),
    ]
    kDriveKinematics = SwerveDrive4Kinematics(*kModulePositions)

    # MOTOR CAN IDs (Using IDs from example)
    kFrontLeftDrivingCanId = 7
    kRearLeftDrivingCanId = 6
    kFrontRightDrivingCanId = 3
    kRearRightDrivingCanId = 2

    kFrontLeftTurningCanId = 8
    kRearLeftTurningCanId = 5
    kFrontRightTurningCanId = 4
    kRearRightTurningCanId = 1

    # CANCODER CAN IDs (If not using, set to -1)
    kFrontLeftCancoderCanId = -1
    kRearLeftCancoderCanId = -1
    kFrontRightCancoderCanId = -1
    kRearRightCancoderCanId = -1

    kGyroReversed = -1

class ModuleConstants:
    kWheelDiameterMeters = 0.0762
    kTurningReductionRatio = 150.0 / 7.0

    kTurningEncoderInverted = False
    kTurningMotorInverted = True

    kDrivingMotorPinionTeeth = 14
    kWheelCircumferenceMeters = kWheelDiameterMeters * math.pi
    kDrivingMotorFreeSpeedRps = NeoMotorConstants.kFreeSpeedRpm / 60
    kDrivingMotorReduction = (45.0 * 22) / (kDrivingMotorPinionTeeth * 15)
    kDriveWheelFreeSpeedRps = (
        kDrivingMotorFreeSpeedRps * kWheelCircumferenceMeters
    ) / kDrivingMotorReduction

    kDrivingEncoderPositionFactor = (
        kWheelDiameterMeters * math.pi
    ) / kDrivingMotorReduction
    kDrivingEncoderVelocityFactor = (
        (kWheelDiameterMeters * math.pi) / kDrivingMotorReduction
    ) / 60.0

    kTurningEncoderPositionFactor = math.tau
    kTurningEncoderVelocityFactor = math.tau / 60.0

    kDrivingP = 0.04
    kDrivingI = 0
    kDrivingD = 0
    kDrivingFF = 1.0 / kDriveWheelFreeSpeedRps
    kDrivingMinOutput = -1
    kDrivingMaxOutput = 1

    kTurningP = 1.0
    kTurningI = 0
    kTurningD = 0
    kTurningFF = 0
    kTurningMinOutput = -1
    kTurningMaxOutput = 1

    kDrivingMotorCurrentLimit = 50
    kTurningMotorCurrentLimit = 20
    kDrivingMinSpeedMetersPerSecond = 0.01

def getSwerveDrivingMotorConfig() -> SparkBaseConfig:
    drivingConfig = SparkBaseConfig()
    drivingConfig.setIdleMode(SparkBaseConfig.IdleMode.kCoast)
    drivingConfig.smartCurrentLimit(stallLimit=ModuleConstants.kDrivingMotorCurrentLimit)
    drivingConfig.encoder.positionConversionFactor(ModuleConstants.kDrivingEncoderPositionFactor)
    drivingConfig.encoder.velocityConversionFactor(ModuleConstants.kDrivingEncoderVelocityFactor)
    drivingConfig.closedLoop.setFeedbackSensor(FeedbackSensor.kPrimaryEncoder)
    drivingConfig.closedLoop.pid(ModuleConstants.kDrivingP, ModuleConstants.kDrivingI, ModuleConstants.kDrivingD)
    drivingConfig.closedLoop.velocityFF(ModuleConstants.kDrivingFF)
    drivingConfig.closedLoop.outputRange(ModuleConstants.kDrivingMinOutput, ModuleConstants.kDrivingMaxOutput)
    return drivingConfig

def getSwerveTurningMotorConfig(turnMotorInverted: bool, useAbsoluteEncoderGoals: bool = True) -> SparkBaseConfig:
    turningConfig = SparkBaseConfig()
    turningConfig.inverted(turnMotorInverted)
    turningConfig.setIdleMode(SparkBaseConfig.IdleMode.kBrake)
    turningConfig.smartCurrentLimit(stallLimit=ModuleConstants.kTurningMotorCurrentLimit)
    turningConfig.absoluteEncoder.positionConversionFactor(ModuleConstants.kTurningEncoderPositionFactor)
    turningConfig.absoluteEncoder.velocityConversionFactor(ModuleConstants.kTurningEncoderVelocityFactor)
    turningConfig.absoluteEncoder.inverted(ModuleConstants.kTurningEncoderInverted)

    turningConfig.encoder.positionConversionFactor(1.0)
    turningConfig.encoder.velocityConversionFactor(1.0 / 60)

    if useAbsoluteEncoderGoals:
        turningConfig.closedLoop.setFeedbackSensor(FeedbackSensor.kAbsoluteEncoder)
        turningConfig.closedLoop.pid(ModuleConstants.kTurningP, 0.0, ModuleConstants.kTurningD)
    else:
        turningConfig.closedLoop.setFeedbackSensor(FeedbackSensor.kPrimaryEncoder)
        turningConfig.closedLoop.pid(ModuleConstants.kTurningP * math.tau, 0.0, ModuleConstants.kTurningD)

    turningConfig.closedLoop.velocityFF(ModuleConstants.kTurningFF)
    turningConfig.closedLoop.outputRange(ModuleConstants.kTurningMinOutput, ModuleConstants.kTurningMaxOutput)
    if useAbsoluteEncoderGoals:
        turningConfig.closedLoop.positionWrappingEnabled(True)
        turningConfig.closedLoop.positionWrappingInputRange(0, ModuleConstants.kTurningEncoderPositionFactor)
    else:
        turningConfig.closedLoop.positionWrappingEnabled(False)

    return turningConfig
