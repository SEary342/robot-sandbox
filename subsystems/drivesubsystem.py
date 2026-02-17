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
from photonlibpy.photonPoseEstimator import PhotonPoseEstimator
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
                 # STUDENTS: If your robot drives backwards or spins in place,
                 # change these True/False values!
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
        self.field_layout = None
        try:
            # 1. Connect to the Camera
            # "Arducam..." must match the Camera Name in the PhotonVision dashboard (http://photonvision.local:5800)
            self.camera = PhotonCamera("Arducam_OV9281_USB_Camera")

            # 2. Load the Field Map
            # This loads the official locations of all AprilTags for the current game.
            self.field_layout = AprilTagFieldLayout.loadField(AprilTagField.k2026RebuiltAndyMark)

            # 3. Define Camera Position (Crucial!)
            # We must tell the math exactly where the camera is relative to the center of the robot.
            # constants.kCameraOffsetX/Y/Z are defined in constants.py
            robot_to_camera = Transform3d(
                Translation3d(constants.kCameraOffsetX, constants.kCameraOffsetY, constants.kCameraHeight),
                Rotation3d(0, constants.kCameraPitch, 0)
            )

            # 4. Create the Pose Estimator
            # This tool combines the Camera data + Field Map + Camera Position to calculate "Where am I?"
            self.photon_estimator = PhotonPoseEstimator(
                self.field_layout,
                robot_to_camera
            )
        except Exception as e:
            # If the camera isn't plugged in or PhotonVision isn't running, this prevents the code from crashing.
            print(f"Vision Init Failed: {e}")
            self.photon_estimator = None

        # --- PathPlanner Setup ---
        # PathPlanner is a tool that lets us draw paths on a computer and have the robot follow them.
        try:
            self.configurePathPlanner()
        except Exception as e:
            # If something goes wrong (like the config file is missing), don't crash the whole robot.
            print(f"PathPlanner Config Failed: {e}")

        SmartDashboard.setDefaultNumber("driveKPMult", 0.5)
        SmartDashboard.setDefaultNumber("driveKDMult", 0.5)
        SmartDashboard.setDefaultNumber("driveKFFMult", 1.0)
        SmartDashboard.setDefaultNumber("driveMaxSpeedMult", 1.0)
        SmartDashboard.setDefaultNumber("driveMaxAccMult", 1.0)

        self.simPhysics = None

    def configurePathPlanner(self):
        """
        This function connects the PathPlanner library to our specific robot code.
        It tells PathPlanner how to read our position and how to make the wheels spin.
        """
        # Load the robot settings (width, max speed) from the GUI file
        config = RobotConfig.fromGUISettings()

        # Configure the "Ramsete" controller. 
        # Ramsete is a specific math algorithm good for tank-drive robots (like this one).
        AutoBuilder.configureRamsete(
            self.getPose,            # "Where am I?" (Function that returns x, y, angle)
            self.resetOdometry,      # "Start here!" (Function to reset position at start of auto)
            self.getChassisSpeeds,   # "How fast am I going?" (Current speed)
            self.driveChassisSpeeds, # "Move!" (Function to set motor speeds)
            config,                  # The robot settings we loaded above
            self.shouldFlipPath,     # "Am I on the Red Alliance?" (Flips path if needed)
            self                     # The subsystem (this file)
        )

    def shouldFlipPath(self):
        # PathPlanner needs to know if we are on the Red Alliance.
        # If we are, it flips the path so we don't drive into the wall!
        return DriverStation.getAlliance() == DriverStation.Alliance.kRed

    def stop(self):
        """
        Safely stops the robot.
        """
        self.desiredLeftVelocity = 0
        self.desiredRightVelocity = 0
        
        # If we are using the basic DifferentialDrive (no PID), just stop.
        if self.diffDrive:
            self.diffDrive.stopMotor()
        else:
            # If we are using PID, tell the motor controllers to target 0 RPM.
            self.leftPIDController.setReference(0, rev.SparkBase.ControlType.kVelocity)
            self.rightPIDController.setReference(0, rev.SparkBase.ControlType.kVelocity)

    def periodic(self):
        if self.simPhysics is not None:
            self.simPhysics.periodic()

        # --- Odometry Update (Where is the robot?) ---
        # 1. Update Pose Estimator with Encoders + Gyro
        self.poseEstimator.update(
            self.getGyroHeading(),
            self.leftEncoder.getPosition() * constants.kLeftEncoderSign,
            self.rightEncoder.getPosition() * constants.kRightEncoderSign,
        )

        # 2. Update Pose Estimator with Vision, using the logic from the PhotonLib example
        if self.photon_estimator:
            for result in self.camera.getAllUnreadResults():
                # First, try to get a multi-tag estimate from the coprocessor.
                est = self.photon_estimator.estimateCoprocMultiTagPose(result)
                if est is None:
                    # If that fails, fall back to a single-tag estimate with the lowest ambiguity.
                    est = self.photon_estimator.estimateLowestAmbiguityPose(result)

                # If we have a valid estimate, add it to the pose estimator.
                if est is not None:
                    estimated_pose = est.estimatedPose.toPose2d()
                    self.poseEstimator.addVisionMeasurement(estimated_pose, est.timestampSeconds)

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
            # Cubing the input (x^3) makes the joystick less sensitive near the center.
            # This helps the driver make small adjustments without jerking the robot.
            fwd = fwd * fwd * fwd
            rot = rot * abs(rot)

        if rot > 1:
            rot = 1
        if rot < -1:
            rot = -1

        # Prevent "Motor Saturation" (asking for > 100% power).
        # If we try to go 100% Forward AND 100% Turn, the math would ask one motor for 200%.
        # This logic prioritizes Turning. If you turn 60%, you can only go forward 40%.
        speedLimit = max((0, 1 - abs(rot)))
        if fwd > speedLimit:
            fwd = speedLimit
        if fwd < -speedLimit:
            fwd = -speedLimit
        # ^^ when asked to rotate at speed 0.6, we can only drive forward at speedLimit=1-0.6=0.4

        # Calculate target RPM for each side
        # Left = Forward - Turn, Right = Forward + Turn
        self.desiredRightVelocity = (fwd + rot) * DrivetrainConstants.maxRPM
        self.desiredLeftVelocity = (fwd - rot) * DrivetrainConstants.maxRPM

        if self.diffDrive:
            # use basic DifferentialDrive and don't take advantage of low-level Rev PID controller
            self.diffDrive.arcadeDrive(fwd, rot)
        else:
            # use Rev PID control for better speed and acceleration
            # This tells the SparkMax: "Spin at exactly this RPM, no matter the load."
            self.leftPIDController.setReference(self.desiredLeftVelocity, rev.SparkBase.ControlType.kVelocity)
            self.rightPIDController.setReference(self.desiredRightVelocity, rev.SparkBase.ControlType.kVelocity)

    def getAverageEncoderDistance(self):
        """
        Gets the average distance of the two encoders.
        Useful for simple autonomous commands like "Drive forward 2 meters".
        """
        return (self.leftEncoder.getPosition() * constants.kLeftEncoderSign +
                self.rightEncoder.getPosition() * constants.kRightEncoderSign) / 2

    def setMaxOutput(self, maxOutput):
        """
        Sets the max output of the drive. Useful for scaling the drive to drive more slowly.
        Example: setMaxOutput(0.5) makes the robot go half speed max.
        """
        if self.diffDrive:
            self.diffDrive.setMaxOutput(maxOutput)

    def zeroHeading(self):
        """
        Resets the Gyro so the current direction becomes "0 degrees" (Forward).
        Call this at the start of a match!
        """
        # Since we can't easily reset the remote SenseHat hardware, we just remember 
        # the current value and subtract it from future readings.
        self.gyroOffset = self.gyro_yaw_entry.get()

    def getHeading(self):
        """Returns the direction the robot is facing (0 to 360 degrees)."""
        return self.getPose().rotation()

    def getGyroHeading(self):
        """
        Reads the raw angle from the Raspberry Pi SenseHat.
        """
        raw_yaw = self.gyro_yaw_entry.get()
        # Subtract the offset to account for when we pressed "Reset Gyro"
        return Rotation2d.fromDegrees(raw_yaw - self.gyroOffset)

    def getTurnRate(self):
        """Returns the turn rate of the robot."""
        return 0.0 # Rate not currently implemented from SenseHat

    def getDistanceToTag(self, tag_id: int) -> float:
        """
        Calculates the straight-line distance to a specific AprilTag.
        Useful for shooting: "If I am 3 meters away, spin shooter to 3500 RPM".
        """
        if self.field_layout is not None:
            # 1. Find where the tag is on the field map
            tag_pose = self.field_layout.getTagPose(tag_id)
            if tag_pose is not None:
                # 2. Calculate distance between Robot (getPose) and Tag
                return self.getPose().translation().distance(tag_pose.toPose2d().translation())
        return -1.0


def _getFollowMotorConfig(leadCanID, inverted):
    """
    Creates settings for a 'Follower' motor.
    A follower motor just copies what the 'Leader' motor does.
    """
    config = rev.SparkBaseConfig()
    config.follow(leadCanID, inverted)
    return config


def _getLeadMotorConfig(
    inverted: bool,
    positionFactor: float,
) -> rev.SparkBaseConfig:
    """
    Creates settings for a 'Leader' motor.
    This sets up the brakes, the encoder math, and the PID (speed control) settings.
    """
    config = rev.SparkBaseConfig()
    config.inverted(inverted)

    # Brake mode: The robot stops quickly when you let go of the stick.
    config.setIdleMode(rev.SparkBaseConfig.IdleMode.kBrake)

    # Disable limit switches (we don't use them on wheels)
    config.limitSwitch.forwardLimitSwitchEnabled(False)
    config.limitSwitch.reverseLimitSwitchEnabled(False)

    # Convert "Rotations" to "Meters"
    config.encoder.positionConversionFactor(positionFactor)
    #config.encoder.velocityConversionFactor(positionFactor / 60)  # 60 seconds per minute

    # Setup PID (The math that keeps speed constant)
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
