#
# Copyright (c) FIRST and other WPILib contributors.
# Open Source Software; you can modify and/or share it under the terms of
# the WPILib BSD license file in the root directory of this project.
#
import math

import ntcore
from commands2 import Subsystem
from rev import ResetMode, PersistMode

from wpilib import MotorControllerGroup, DriverStation
from wpilib.drive import DifferentialDrive
from wpilib import SmartDashboard, Field2d, RobotBase, Timer

from wpimath.kinematics import DifferentialDriveWheelSpeeds, ChassisSpeeds
from wpimath.geometry import Rotation2d, Pose2d, Transform3d, Translation3d, Rotation3d
from wpimath.estimator import DifferentialDrivePoseEstimator

from photonlibpy.photonCamera import PhotonCamera
from photonlibpy.photonPoseEstimator import PhotonPoseEstimator, PoseStrategy
from robotpy_apriltag import AprilTagFieldLayout, AprilTagField

from pathplannerlib.auto import AutoBuilder
from pathplannerlib.config import RobotConfig

import constants
import rev


class DrivetrainConstants:
    initialP = 2.5 / 10000.0
    initialD = 5.0 / 10000.0  # coincidentally same as initialP, but really does not need to be
    initialFF = 1.4 / 10000.0  # if setting it to nonzero, be careful and start small
    maxRPM = 3000


class DriveSubsystem(Subsystem):
    # noinspection PyInterpreter
    def __init__(self,
                 usePIDController=True,
                 l1MotorInverted=False,
                 l2MotorInverted=False,
                 r1MotorInverted=True,
                 r2MotorInverted=True
    ):
        super().__init__()

        self.desiredLeftVelocity = 0.0
        self.desiredRightVelocity = 0.0

        # The motors on the left side of the drive.
        self.motorL1 = rev.SparkMax(constants.kLeftMotor1CAN, rev.SparkMax.MotorType.kBrushless)
        self.motorL1.configure(
            _getLeadMotorConfig(l1MotorInverted, constants.kEncoderPositionConversionFactor),
            ResetMode.kResetSafeParameters,
            PersistMode.kPersistParameters)

        self.motorL2 = rev.SparkMax(constants.kLeftMotor2CAN, rev.SparkMax.MotorType.kBrushless)
        self.motorL2.configure(
            _getFollowMotorConfig(constants.kLeftMotor1CAN, l2MotorInverted != l1MotorInverted),
            ResetMode.kResetSafeParameters,
            PersistMode.kPersistParameters)

        # The motors on the right side of the drive.
        self.motorR1 = rev.SparkMax(constants.kRightMotor1CAN, rev.SparkMax.MotorType.kBrushless)
        self.motorR1.configure(
            _getLeadMotorConfig(r1MotorInverted, constants.kEncoderPositionConversionFactor),
            ResetMode.kResetSafeParameters,
            PersistMode.kPersistParameters)

        self.motorR2 = rev.SparkMax(constants.kRightMotor2CAN, rev.SparkMax.MotorType.kBrushless)
        self.motorR2.configure(
            _getFollowMotorConfig(constants.kRightMotor1CAN, r2MotorInverted != r1MotorInverted),
            ResetMode.kResetSafeParameters,
            PersistMode.kPersistParameters)

        if usePIDController:
            # do not use basic differential drive, take advantage of low-level PID control from Rev
            self.diffDrive = None
            self.leftMotors = None
            self.rightMotors = None
            self.leftPIDController = self.motorL1.getClosedLoopController()
            self.rightPIDController = self.motorR1.getClosedLoopController()
        else:
            # use the basic differential drive (robot will be less responsive and slower)
            # We need to invert one side of the drivetrain so that positive voltages
            # result in both sides moving forward. Depending on how your robot's
            # gearbox is constructed, you might have to invert the left side instead.
            self.leftMotors = MotorControllerGroup(self.motorL1, self.motorL2)
            self.rightMotors = MotorControllerGroup(self.motorR1, self.motorR2)
            self.rightMotors.setInverted(True)
            self.diffDrive = DifferentialDrive(self.motorL1, self.motorR1)

        # The left-side drive encoder
        self.leftEncoder = self.motorL1.getEncoder()

        # The right-side drive encoder
        self.rightEncoder = self.motorR1.getEncoder()

        # --- Gyro Setup (SenseHat via NetworkTables) ---
        self.inst = ntcore.NetworkTableInstance.getDefault()
        self.sensehat_table = self.inst.getTable("SenseHat")
        self.gyro_yaw_entry = self.sensehat_table.getDoubleTopic("yaw").getEntry(0.0)
        self.gyroOffset = 0.0

        # --- Pose Estimator (Replaces Odometry) ---
        self.poseEstimator = DifferentialDrivePoseEstimator(
            constants.kDriveKinematics,
            self.getGyroHeading(),
            self.leftEncoder.getPosition() * constants.kLeftEncoderSign,
            self.rightEncoder.getPosition() * constants.kRightEncoderSign,
            Pose2d()
        )

        self.field = Field2d()
        SmartDashboard.putData("Field", self.field)

        # --- Vision Setup ---
        try:
            self.camera = PhotonCamera("Arducam_OV9281_USB_Camera")
            self.field_layout = AprilTagFieldLayout.loadField(AprilTagField.k2026RebuiltAndyMark)
            robot_to_camera = Transform3d(
                Translation3d(constants.kCameraOffsetX, constants.kCameraOffsetY, constants.kCameraHeight),
                Rotation3d(0, constants.kCameraPitch, 0)
            )
            self.photon_estimator = PhotonPoseEstimator(
                self.field_layout,
                PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
                self.camera,
                robot_to_camera
            )
        except Exception as e:
            print(f"Vision Init Failed: {e}")
            self.photon_estimator = None

        # --- PathPlanner Setup ---
        try:
            self.configurePathPlanner()
        except Exception as e:
            print(f"PathPlanner Config Failed: {e}")

        SmartDashboard.setDefaultNumber("driveKPMult", 0.5)
        SmartDashboard.setDefaultNumber("driveKDMult", 0.5)
        SmartDashboard.setDefaultNumber("driveKFFMult", 1.0)
        SmartDashboard.setDefaultNumber("driveMaxSpeedMult", 1.0)
        SmartDashboard.setDefaultNumber("driveMaxAccMult", 1.0)

        self.simPhysics = None

    def configurePathPlanner(self):
        config = RobotConfig.fromGUISettings()
        AutoBuilder.configureRamsete(
            self.getPose,
            self.resetOdometry,
            self.getChassisSpeeds,
            self.driveChassisSpeeds,
            config,
            self.shouldFlipPath,
            self
        )

    def shouldFlipPath(self):
        return DriverStation.getAlliance() == DriverStation.Alliance.kRed

    def stop(self):
        self.desiredLeftVelocity = 0
        self.desiredRightVelocity = 0
        if self.diffDrive:
            self.diffDrive.stopMotor()
        else:
            self.leftPIDController.setReference(0, rev.SparkBase.ControlType.kVelocity)
            self.rightPIDController.setReference(0, rev.SparkBase.ControlType.kVelocity)

    def periodic(self):
        if self.simPhysics is not None:
            self.simPhysics.periodic()

        # 1. Update Pose Estimator with Encoders + Gyro
        self.poseEstimator.update(
            self.getGyroHeading(),
            self.leftEncoder.getPosition() * constants.kLeftEncoderSign,
            self.rightEncoder.getPosition() * constants.kRightEncoderSign,
        )

        # 2. Update Pose Estimator with Vision
        if self.photon_estimator:
            vision_result = self.photon_estimator.update()
            if vision_result:
                estimated_pose = vision_result.estimatedPose.toPose2d()
                self.poseEstimator.addVisionMeasurement(estimated_pose, vision_result.timestampSeconds)

        # Update the pose of the robot (x, y, heading) on the SmartDashboard
        pose = self.getPose()
        SmartDashboard.putNumber("x", pose.x)
        SmartDashboard.putNumber("y", pose.y)
        SmartDashboard.putNumber("heading", pose.rotation().degrees())
        self.field.setRobotPose(pose)

    def getPose(self):
        """Returns the currently-estimated pose of the robot."""
        return self.poseEstimator.getEstimatedPosition()

    def getWheelSpeeds(self):
        """Returns the current wheel speeds of the robot."""
        return DifferentialDriveWheelSpeeds(
            self.leftEncoder.getVelocity(), self.rightEncoder.getVelocity() * constants.kRightEncoderSign
        )

    def getChassisSpeeds(self):
        """Returns the current chassis speeds of the robot."""
        return constants.kDriveKinematics.toChassisSpeeds(self.getWheelSpeeds())

    def driveChassisSpeeds(self, speeds: ChassisSpeeds):
        """Drives the robot with the given chassis speeds."""
        wheelSpeeds = constants.kDriveKinematics.toWheelSpeeds(speeds)
        
        # Convert m/s to RPM to match arcadeDrive's usage of desiredVelocity
        self.desiredLeftVelocity = wheelSpeeds.left * 60 / (math.pi * constants.kWheelDiameterMeters)
        self.desiredRightVelocity = wheelSpeeds.right * 60 / (math.pi * constants.kWheelDiameterMeters)
        
        self.leftPIDController.setReference(self.desiredLeftVelocity, rev.SparkBase.ControlType.kVelocity)
        self.rightPIDController.setReference(self.desiredRightVelocity, rev.SparkBase.ControlType.kVelocity)

    def resetOdometry(self, pose):
        """Resets the odometry to the specified pose."""
        self.poseEstimator.resetPosition(
            self.getGyroHeading(),
            self.leftEncoder.getPosition() * constants.kLeftEncoderSign,
            self.rightEncoder.getPosition() * constants.kRightEncoderSign,
            pose,
        )

    def drive(self, xSpeed, ySpeed, rot, fieldRelative, rateLimit) -> None:
        assert False, "ERROR: swerve drive not available on this drivetrain"

    def arcadeDrive(self, fwd, rot, assumeManualInput=False):
        """Drives the robot using arcade controls."""

        # use curves to take smoother input from human
        if assumeManualInput:
            fwd = fwd * fwd * fwd
            rot = rot * abs(rot)

        if rot > 1:
            rot = 1
        if rot < -1:
            rot = -1

        speedLimit = max((0, 1 - abs(rot)))
        if fwd > speedLimit:
            fwd = speedLimit
        if fwd < -speedLimit:
            fwd = -speedLimit
        # ^^ when asked to rotate at speed 0.6, we can only drive forward at speedLimit=1-0.6=0.4

        self.desiredRightVelocity = (fwd + rot) * DrivetrainConstants.maxRPM
        self.desiredLeftVelocity = (fwd - rot) * DrivetrainConstants.maxRPM

        if self.diffDrive:
            # use basic DifferentialDrive and don't take advantage of low-level Rev PID controller
            self.diffDrive.arcadeDrive(fwd, rot)
        else:
            # use Rev PID control for better speed and acceleration
            self.leftPIDController.setReference(self.desiredLeftVelocity, rev.SparkBase.ControlType.kVelocity)
            self.rightPIDController.setReference(self.desiredRightVelocity, rev.SparkBase.ControlType.kVelocity)

    def getAverageEncoderDistance(self):
        """Gets the average distance of the two encoders."""
        return (self.leftEncoder.getPosition() * constants.kLeftEncoderSign +
                self.rightEncoder.getPosition() * constants.kRightEncoderSign) / 2

    def setMaxOutput(self, maxOutput):
        """Sets the max output of the drive. Useful for scaling the drive to drive more slowly."""
        if self.diffDrive:
            self.diffDrive.setMaxOutput(maxOutput)

    def zeroHeading(self):
        """Zeroes the heading of the robot."""
        # Since we can't easily reset the remote SenseHat, we store an offset
        self.gyroOffset = self.gyro_yaw_entry.get()

    def getHeading(self):
        """Returns the heading of the robot."""
        return self.getPose().rotation()

    def getGyroHeading(self):
        """Returns the heading of the robot."""
        raw_yaw = self.gyro_yaw_entry.get()
        return Rotation2d.fromDegrees(raw_yaw - self.gyroOffset)

    def getTurnRate(self):
        """Returns the turn rate of the robot."""
        return 0.0 # Rate not currently implemented from SenseHat


def _getFollowMotorConfig(leadCanID, inverted):
    config = rev.SparkBaseConfig()
    config.follow(leadCanID, inverted)
    return config


def _getLeadMotorConfig(
    inverted: bool,
    positionFactor: float,
) -> rev.SparkBaseConfig:
    config = rev.SparkBaseConfig()
    config.inverted(inverted)
    config.setIdleMode(rev.SparkBaseConfig.IdleMode.kBrake)
    config.limitSwitch.forwardLimitSwitchEnabled(False)
    config.limitSwitch.reverseLimitSwitchEnabled(False)
    config.encoder.positionConversionFactor(positionFactor)
    #config.encoder.velocityConversionFactor(positionFactor / 60)  # 60 seconds per minute
    config.closedLoop.pid(DrivetrainConstants.initialP, 0.0, DrivetrainConstants.initialD)
    config.closedLoop.velocityFF(DrivetrainConstants.initialFF)
    config.closedLoop.outputRange(-1, +1)
    return config


class BadSimPhysics(object):
    """
    this is the wrong way to do it, it does not scale!!!
    the right way is shown here: https://github.com/robotpy/examples/blob/main/Physics/src/physics.py
    and documented here: https://robotpy.readthedocs.io/projects/pyfrc/en/stable/physics.html
    (but for a swerve drive it will take some work to add correctly)
    """
    def __init__(self, drivetrain: DriveSubsystem, robot: RobotBase):
        self.drivetrain = drivetrain
        self.robot = robot
        self.t = 0

    def periodic(self):
        past = self.t
        self.t = Timer.getFPGATimestamp()
        if past == 0:
            return  # it was first time

        dt = self.t - past
        if self.robot.isEnabled():
            drivetrain = self.drivetrain
            toDriveSpeed = constants.kDriveSpeedAtMaxRPM / DrivetrainConstants.maxRPM

            states = DifferentialDriveWheelSpeeds(
                left=drivetrain.desiredLeftVelocity * toDriveSpeed,
                right=drivetrain.desiredRightVelocity * toDriveSpeed,
            )
            speeds = constants.kDriveKinematics.toChassisSpeeds(states)

            # Calculate new pose using Twist2d (kinematics integration)
            from wpimath.geometry import Twist2d
            currentPose = drivetrain.getPose()
            twist = Twist2d(speeds.vx * dt, speeds.vy * dt, speeds.omega * dt)
            newPose = currentPose.exp(twist)

            # Update the "fake" gyro and force the pose estimator
            drivetrain.gyro_yaw_entry.set(newPose.rotation().degrees() + drivetrain.gyroOffset)
            drivetrain.resetOdometry(newPose)
