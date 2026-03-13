#
# Copyright (c) FIRST and other WPILib contributors.
# Open Source Software; you can modify and/or share it under the terms of
# the WPILib BSD license file in the root directory of this project.
#

import rev
import ntcore
from commands2 import Subsystem
from rev import ResetMode, PersistMode

from wpilib import DriverStation, Field2d, SmartDashboard
from wpimath.geometry import Pose2d, Rotation2d, Transform3d, Translation3d, Rotation3d
from wpimath.kinematics import (
    ChassisSpeeds,
    SwerveModuleState,
    SwerveModulePosition,
)
from wpimath.estimator import SwerveDrive4PoseEstimator

from photonlibpy.photonCamera import PhotonCamera
from photonlibpy.photonPoseEstimator import PhotonPoseEstimator
from robotpy_apriltag import AprilTagFieldLayout, AprilTagField

from pathplannerlib.auto import AutoBuilder
from pathplannerlib.config import RobotConfig, PIDConstants
from pathplannerlib.controller import PPHolonomicDriveController

import constants
from . import swerveconstants


class SwerveModule:
    def __init__(self, driveMotorID: int, turningMotorID: int, driveInverted: bool):
        self.driveMotor = rev.SparkMax(driveMotorID, rev.SparkMax.MotorType.kBrushless)
        self.turningMotor = rev.SparkMax(turningMotorID, rev.SparkMax.MotorType.kBrushless)

        self.driveEncoder = self.driveMotor.getEncoder()
        self.turningEncoder = self.turningMotor.getEncoder()
        self.drivePID = self.driveMotor.getClosedLoopController()
        self.turningPID = self.turningMotor.getClosedLoopController()

        # --- Drive Configuration ---
        driveConfig = rev.SparkMaxConfig()
        driveConfig.inverted(driveInverted)
        driveConfig.setIdleMode(rev.SparkBaseConfig.IdleMode.kBrake)
        driveConfig.smartCurrentLimit(constants.kDriveMotorCurrentLimit)
        
        driveConfig.encoder.positionConversionFactor(swerveconstants.SwerveConstants.kDriveEncoderPositionFactor)
        driveConfig.encoder.velocityConversionFactor(swerveconstants.SwerveConstants.kDriveEncoderVelocityFactor)
        
        driveConfig.closedLoop.pid(swerveconstants.SwerveConstants.kDrivingP, 0, 0)
        # Fixed: velocityFF is a direct child of closedLoop
        driveConfig.closedLoop.velocityFF(swerveconstants.SwerveConstants.kDrivingFF)

        # --- Turning Configuration ---
        turnConfig = rev.SparkMaxConfig()
        turnConfig.setIdleMode(rev.SparkBaseConfig.IdleMode.kBrake)
        turnConfig.smartCurrentLimit(20) 
        
        turnConfig.encoder.positionConversionFactor(swerveconstants.SwerveConstants.kTurningEncoderPositionFactor)
        
        turnConfig.closedLoop.pid(swerveconstants.SwerveConstants.kTurningP, 0, 0)
        turnConfig.closedLoop.positionWrappingEnabled(True)
        turnConfig.closedLoop.positionWrappingMinInput(0)
        turnConfig.closedLoop.positionWrappingMaxInput(swerveconstants.SwerveConstants.kTurningEncoderPositionFactor)

        self.driveMotor.configure(driveConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters)
        self.turningMotor.configure(turnConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters)

    def getState(self) -> SwerveModuleState:
        return SwerveModuleState(self.driveEncoder.getVelocity(), Rotation2d(self.turningEncoder.getPosition()))

    def getPosition(self) -> SwerveModulePosition:
        return SwerveModulePosition(self.driveEncoder.getPosition(), Rotation2d(self.turningEncoder.getPosition()))

    def setDesiredState(self, desiredState: SwerveModuleState):
        currentAngle = Rotation2d(self.turningEncoder.getPosition())
        desiredState.optimize(currentAngle)

        self.drivePID.setReference(desiredState.speed, rev.SparkMax.ControlType.kVelocity)
        self.turningPID.setReference(desiredState.angle.radians(), rev.SparkMax.ControlType.kPosition)


class SwerveSubsystem(Subsystem):
    def __init__(self):
        super().__init__()

        self.frontLeft = SwerveModule(
            swerveconstants.SwerveConstants.kFrontLeftDrivingCanId,
            swerveconstants.SwerveConstants.kFrontLeftTurningCanId,
            False
        )
        self.frontRight = SwerveModule(
            swerveconstants.SwerveConstants.kFrontRightDrivingCanId,
            swerveconstants.SwerveConstants.kFrontRightTurningCanId,
            False
        )
        self.rearLeft = SwerveModule(
            swerveconstants.SwerveConstants.kRearLeftDrivingCanId,
            swerveconstants.SwerveConstants.kRearLeftTurningCanId,
            False
        )
        self.rearRight = SwerveModule(
            swerveconstants.SwerveConstants.kRearRightDrivingCanId,
            swerveconstants.SwerveConstants.kRearRightTurningCanId,
            False
        )

        self.inst = ntcore.NetworkTableInstance.getDefault()
        self.sensehat_table = self.inst.getTable("SenseHat")
        self.gyro_yaw_entry = self.sensehat_table.getDoubleTopic("yaw").getEntry(0.0)
        self.gyroOffset = 0.0

        self.poseEstimator = SwerveDrive4PoseEstimator(
            swerveconstants.SwerveConstants.kKinematics,
            self.getGyroHeading(),
            self.getModulePositions(),
            Pose2d(),
        )

        self.field_layout = None
        self.photon_estimator = None
        if not constants.kTestBench:
            try:
                self.camera = PhotonCamera(constants.kCamera1Name)
                self.field_layout = AprilTagFieldLayout.loadField(AprilTagField.k2026RebuiltAndyMark)
                
                robot_to_camera = Transform3d(
                    Translation3d(constants.kCameraOffsetX, constants.kCameraOffsetY, constants.kCameraHeight),
                    Rotation3d(0, constants.kCameraPitch, 0),
                )
                self.photon_estimator = PhotonPoseEstimator(self.field_layout, robot_to_camera)
            except Exception as e:
                print(f"Vision Init Failed: {e}")

        self.configurePathPlanner()

        self.field = Field2d()
        SmartDashboard.putData("Field", self.field)

    def configurePathPlanner(self):
        config = RobotConfig.fromGUISettings()

        AutoBuilder.configure(
            self.getPose,
            self.resetOdometry,
            self.getChassisSpeeds,
            self.driveChassisSpeeds, 
            PPHolonomicDriveController(
                PIDConstants(5.0, 0.0, 0.0), 
                PIDConstants(5.0, 0.0, 0.0)  
            ),
            config,
            self.shouldFlipPath,
            self
        )

    def shouldFlipPath(self) -> bool:
        return DriverStation.getAlliance() == DriverStation.Alliance.kRed

    def periodic(self):
        self.poseEstimator.update(self.getGyroHeading(), self.getModulePositions())

        if self.photon_estimator:
            for result in self.camera.getAllUnreadResults():
                est = self.photon_estimator.estimateCoprocMultiTagPose(result)
                if not est:
                    est = self.photon_estimator.estimateLowestAmbiguityPose(result)
                
                if est:
                    self.poseEstimator.addVisionMeasurement(
                        est.estimatedPose.toPose2d(), est.timestampSeconds
                    )

        pose = self.getPose()
        self.field.setRobotPose(pose)
        SmartDashboard.putNumber("Robot Heading", pose.rotation().degrees())

    def drive(self, xSpeed: float, ySpeed: float, rot: float, fieldRelative: bool):
        speeds = (
            ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot, self.getPose().rotation())
            if fieldRelative else ChassisSpeeds(xSpeed, ySpeed, rot)
        )
        self.driveChassisSpeeds(speeds, None)

    def driveChassisSpeeds(self, speeds: ChassisSpeeds, feedforwards):
        moduleStates = swerveconstants.SwerveConstants.kKinematics.toSwerveModuleStates(speeds)
        swerveconstants.SwerveConstants.kKinematics.desaturateWheelSpeeds(
            moduleStates, swerveconstants.SwerveConstants.kMaxSpeedMetersPerSecond
        )
        
        self.frontLeft.setDesiredState(moduleStates[0])
        self.frontRight.setDesiredState(moduleStates[1])
        self.rearLeft.setDesiredState(moduleStates[2])
        self.rearRight.setDesiredState(moduleStates[3])

    def getModulePositions(self) -> tuple[SwerveModulePosition, SwerveModulePosition, SwerveModulePosition, SwerveModulePosition]:
        return (
            self.frontLeft.getPosition(),
            self.frontRight.getPosition(),
            self.rearLeft.getPosition(),
            self.rearRight.getPosition(),
        )

    def getChassisSpeeds(self) -> ChassisSpeeds:
        return swerveconstants.SwerveConstants.kKinematics.toChassisSpeeds((
            self.frontLeft.getState(), self.frontRight.getState(),
            self.rearLeft.getState(), self.rearRight.getState()
        ))

    def getPose(self) -> Pose2d:
        return self.poseEstimator.getEstimatedPosition()

    def resetOdometry(self, pose: Pose2d):
        self.poseEstimator.resetPosition(self.getGyroHeading(), self.getModulePositions(), pose)

    def getGyroHeading(self) -> Rotation2d:
        raw_yaw = self.gyro_yaw_entry.get()
        return Rotation2d.fromDegrees(-(raw_yaw - self.gyroOffset))

    def zeroHeading(self):
        self.gyroOffset = self.gyro_yaw_entry.get()
        currentPose = self.getPose()
        self.resetOdometry(Pose2d(currentPose.translation(), Rotation2d(0)))

    def stop(self):
        self.driveChassisSpeeds(ChassisSpeeds(0, 0, 0), None)