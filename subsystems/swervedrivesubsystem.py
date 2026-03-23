import math
import typing
import ntcore
from commands2 import Subsystem
from wpimath.filter import SlewRateLimiter
from wpimath.geometry import Pose2d, Rotation2d, Transform3d, Translation3d, Rotation3d
from wpimath.kinematics import (
    ChassisSpeeds,
    SwerveModuleState,
    SwerveModulePosition,
    SwerveDrive4Kinematics,
)
from wpimath.estimator import SwerveDrive4PoseEstimator
from wpilib import SmartDashboard, Field2d, DriverStation, Timer
from photonlibpy.photonCamera import PhotonCamera
from photonlibpy.photonPoseEstimator import PhotonPoseEstimator
from robotpy_apriltag import AprilTagFieldLayout, AprilTagField

# --- PathPlanner Imports ---
from pathplannerlib.auto import AutoBuilder
from pathplannerlib.config import RobotConfig, PIDConstants
from pathplannerlib.controller import PPHolonomicDriveController

import constants
from .swervemodule import SwerveModule

class SwerveDriveSubsystem(Subsystem):
    def __init__(self) -> None:
        super().__init__()

        # Create Swerve Modules
        self.frontLeft = SwerveModule(
            constants.DriveConstants.kFrontLeftDrivingCanId,
            constants.DriveConstants.kFrontLeftTurningCanId,
            constants.DriveConstants.kFrontLeftChassisAngularOffset,
            placement="FL",
        )

        self.frontRight = SwerveModule(
            constants.DriveConstants.kFrontRightDrivingCanId,
            constants.DriveConstants.kFrontRightTurningCanId,
            constants.DriveConstants.kFrontRightChassisAngularOffset,
            placement="FR",
        )

        self.rearLeft = SwerveModule(
            constants.DriveConstants.kRearLeftDrivingCanId,
            constants.DriveConstants.kRearLeftTurningCanId,
            constants.DriveConstants.kBackLeftChassisAngularOffset,
            placement="RL",
        )

        self.rearRight = SwerveModule(
            constants.DriveConstants.kRearRightDrivingCanId,
            constants.DriveConstants.kRearRightTurningCanId,
            constants.DriveConstants.kBackRightChassisAngularOffset,
            placement="RR",
        )

        # Gyro Setup (SenseHat via NetworkTables)
        self.inst = ntcore.NetworkTableInstance.getDefault()
        self.sensehat_table = self.inst.getTable("SenseHat")
        self.gyro_yaw_entry = self.sensehat_table.getDoubleTopic("yaw").getEntry(0.0)

        self.xySpeedLimiter = SlewRateLimiter2d(constants.DriveConstants.kMagnitudeSlewRate)
        self.rotLimiter = SlewRateLimiter(constants.DriveConstants.kRotationalSlewRate)

        # Pose Estimator
        self.poseEstimator = SwerveDrive4PoseEstimator(
            constants.DriveConstants.kDriveKinematics,
            self.getGyroHeading(),
            self.getModulePositions(),
            Pose2d(),
        )

        self.field = Field2d()
        SmartDashboard.putData("Field", self.field)

        # Vision Setup
        self.setupVision()

        # PathPlanner Setup
        try:
            self.configurePathPlanner()
        except Exception as e:
            print(f"PathPlanner Config Failed: {e}")

    def setupVision(self):
        try:
            self.camera = PhotonCamera(constants.kCamera1Name)
            self.field_layout = AprilTagFieldLayout.loadField(AprilTagField.k2026RebuiltAndyMark)
            
            robot_to_camera = Transform3d(
                Translation3d(constants.kCamera1OffsetX, constants.kCamera1OffsetY, constants.kCamera1Height),
                Rotation3d(0, constants.kCamera1Pitch, 0),
            )
            
            self.photon_estimator = PhotonPoseEstimator(
                self.field_layout, robot_to_camera
            )
        except Exception as e:
            print(f"Swerve Vision Init Failed: {e}")
            self.photon_estimator = None

    def configurePathPlanner(self):
        """Configures PathPlanner for a holonomic (swerve) drivetrain."""
        # This loads the 'robot_config.json' from your deploy directory
        config = RobotConfig.fromGUISettings()

        AutoBuilder.configure(
            self.getPose,
            self.resetOdometry,
            self.getChassisSpeeds,
            self.driveChassisSpeeds,
            PPHolonomicDriveController(
                PIDConstants(5.0, 0.0, 0.0), # Translation PID constants
                PIDConstants(5.0, 0.0, 0.0)  # Rotation PID constants
            ),
            config,
            self.shouldFlipPath,
            self,
        )

    def shouldFlipPath(self) -> bool:
        # Flip paths if we are on the Red Alliance
        return DriverStation.getAlliance() == DriverStation.Alliance.kRed

    def periodic(self) -> None:
        self.poseEstimator.update(
            self.getGyroHeading(),
            self.getModulePositions(),
        )

        if self.photon_estimator:
            for result in self.camera.getAllUnreadResults():
                est = self.photon_estimator.estimateCoprocMultiTagPose(result)
                if est is None:
                    est = self.photon_estimator.estimateLowestAmbiguityPose(result)
                
                if est is not None:
                    self.poseEstimator.addVisionMeasurement(est.estimatedPose.toPose2d(), est.timestampSeconds)

        pose = self.getPose()
        SmartDashboard.putNumber("x", pose.x)
        SmartDashboard.putNumber("y", pose.y)
        SmartDashboard.putNumber("heading", pose.rotation().degrees())
        self.field.setRobotPose(pose)

    def getPose(self) -> Pose2d:
        return self.poseEstimator.getEstimatedPosition()

    def getGyroHeading(self) -> Rotation2d:
        return Rotation2d.fromDegrees(self.gyro_yaw_entry.get() * constants.DriveConstants.kGyroReversed)

    def getModulePositions(self) -> typing.Tuple[SwerveModulePosition, SwerveModulePosition, SwerveModulePosition, SwerveModulePosition]:
        return (
            self.frontLeft.getPosition(),
            self.frontRight.getPosition(),
            self.rearLeft.getPosition(),
            self.rearRight.getPosition(),
        )

    def getModuleStates(self) -> typing.Tuple[SwerveModuleState, SwerveModuleState, SwerveModuleState, SwerveModuleState]:
        return (
            self.frontLeft.getState(),
            self.frontRight.getState(),
            self.rearLeft.getState(),
            self.rearRight.getState(),
        )

    def getChassisSpeeds(self) -> ChassisSpeeds:
        """Returns the current chassis speeds measured by the swerve modules."""
        # Removed the '*' - we pass the 4-tuple directly to the kinematics object
        return constants.DriveConstants.kDriveKinematics.toChassisSpeeds(
            self.getModuleStates()
        )

    def driveChassisSpeeds(self, speeds: ChassisSpeeds, feedforwards) -> None:
        """
        Drives the robot using ChassisSpeeds. 
        Required for 2026 PathPlanner AutoBuilder.
        """
        # 1. Convert target ChassisSpeeds to individual module states
        swerveModuleStates = constants.DriveConstants.kDriveKinematics.toSwerveModuleStates(speeds)
        
        # 2. Desaturate wheel speeds so no module exceeds its physical max speed.
        # RobotPy returns these as a tuple, which we unpack.
        fl, fr, rl, rr = SwerveDrive4Kinematics.desaturateWheelSpeeds(
            swerveModuleStates, 
            constants.DriveConstants.kMaxSpeedMetersPerSecond
        )
        
        # 3. Apply states to the hardware
        self.frontLeft.setDesiredState(fl)
        self.frontRight.setDesiredState(fr)
        self.rearLeft.setDesiredState(rl)
        self.rearRight.setDesiredState(rr)

    def resetOdometry(self, pose: Pose2d) -> None:
        self.poseEstimator.resetPosition(
            self.getGyroHeading(),
            self.getModulePositions(),
            pose,
        )

    def drive(
        self,
        xSpeed: float,
        ySpeed: float,
        rotSpeed: float,
        fieldRelative: bool,
        rateLimit: bool,
        square: bool = False
    ) -> None:
        """Standard drive method used for teleop (joystick) control."""
        if square:
            xSpeed = xSpeed * abs(xSpeed)
            ySpeed = ySpeed * abs(ySpeed)
            rotSpeed = rotSpeed * abs(rotSpeed)

        xSpeedGoal = xSpeed * constants.DriveConstants.kMaxSpeedMetersPerSecond
        ySpeedGoal = ySpeed * constants.DriveConstants.kMaxSpeedMetersPerSecond
        rotSpeedGoal = rotSpeed * constants.DriveConstants.kMaxAngularSpeed

        if fieldRelative:
            heading = self.getPose().rotation()
            if DriverStation.getAlliance() == DriverStation.Alliance.kRed:
                heading = heading + Rotation2d.fromDegrees(180)
            targetChassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                xSpeedGoal, ySpeedGoal, rotSpeedGoal, heading
            )
        else:
            targetChassisSpeeds = ChassisSpeeds(xSpeedGoal, ySpeedGoal, rotSpeedGoal)

        if rateLimit:
            slewedX, slewedY = self.xySpeedLimiter.calculate(targetChassisSpeeds.vx, targetChassisSpeeds.vy)
            targetChassisSpeeds.vx = slewedX
            targetChassisSpeeds.vy = slewedY
            targetChassisSpeeds.omega = self.rotLimiter.calculate(targetChassisSpeeds.omega)

        # We route through driveChassisSpeeds to keep the desaturation logic in one place
        self.driveChassisSpeeds(targetChassisSpeeds, None)

    def setX(self) -> None:
        self.frontLeft.setDesiredState(SwerveModuleState(0, Rotation2d.fromDegrees(45)))
        self.frontRight.setDesiredState(SwerveModuleState(0, Rotation2d.fromDegrees(-45)))
        self.rearLeft.setDesiredState(SwerveModuleState(0, Rotation2d.fromDegrees(-45)))
        self.rearRight.setDesiredState(SwerveModuleState(0, Rotation2d.fromDegrees(45)))

    def stop(self):
        self.drive(0, 0, 0, False, False)

    def getDistanceToClosestTagInList(self, tags: typing.Sequence[int]) -> float:
        # Helper for shooting logic, similar to tank drive
        pose = self.getPose()
        min_dist = 999.0
        if not self.field_layout:
            return 2.0 # Fallback
            
        for tag_id in tags:
            tag_pose = self.field_layout.getTagPose(tag_id)
            if tag_pose:
                dist = pose.translation().distance(tag_pose.toPose2d().translation())
                if dist < min_dist:
                    min_dist = dist
        return min_dist

    def getRotationToClosestTagInList(self, tags: typing.Sequence[int]) -> Rotation2d:
        """
        Calculates the rotation required to face the closest AprilTag from a given list.
        """
        min_dist = float("inf")
        closest_tag_pose = None

        if self.field_layout is None:
            return self.getPose().rotation()

        for tag_id in tags:
            tag_pose = self.field_layout.getTagPose(tag_id)
            if tag_pose is not None:
                dist = self.getPose().translation().distance(tag_pose.toPose2d().translation())
                if dist < min_dist:
                    min_dist = dist
                    closest_tag_pose = tag_pose

        if closest_tag_pose:
            vector = closest_tag_pose.toPose2d().translation() - self.getPose().translation()
            return vector.angle()
        
        return self.getPose().rotation()

class SlewRateLimiter2d:
    def __init__(self, rate) -> None:
        self.rate = rate
        self.t = Timer.getFPGATimestamp()
        self.x = self.y = 0.0

    def calculate(self, x, y) -> typing.Tuple[float, float]:
        t = Timer.getFPGATimestamp()
        dt = t - self.t
        dx = x - self.x
        dy = y - self.y

        if x != 0 or y != 0:
            dt = min(dt, 0.02)

        limit = self.rate * abs(dt)
        self.t = t

        change = math.hypot(dx, dy)

        if limit > 0:
            if change < limit:
                self.x, self.y = x, y
            else:
                fraction = limit / change
                self.x += dx * fraction
                self.y += dy * fraction

        return self.x, self.y