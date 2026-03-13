#
# Copyright (c) FIRST and other WPILib contributors.
# Open Source Software; you can modify and/or share it under the terms of
# the WPILib BSD license file in the root directory of this project.
#

"""
A place for the constant values in the code that may be used in more than one place.
This offers a convenient resources to teams who need to make both quick and universal
changes.
"""

import math
from wpimath.kinematics import DifferentialDriveKinematics

# =============================================================================
#  ROBOT SETTINGS (CONSTANTS)
#  Students: This is the main place to change numbers!
# =============================================================================

# Set to True if testing on a bench without camera/motors to prevent crashes
kTestBench = False
kCamConfigured = (
    False  # Determines if we are using the camera or just turning on the lauchner.
)
kLaunchMotorCurrentLimit = 20  # Amps, to protect the motor and battery when testing without a camera. Adjust as needed.
kDriveMotorCurrentLimit = 20  # Amps, to protect the motors and battery. Adjust as needed.
# TODO Input smoothing
# TODO Pi distance calculation to target (debug and make sure it?)

# ID for the driver's joystick.
kDriverControllerPort = (
    0  # CHECK: Is your controller plugged into port 0 in Driver Station?
)

# The CAN IDs for the drivetrain motor controllers.
# CHECK: Use the Rev Hardware Client to verify these IDs match your SparkMaxes.
kLeftMotor1CAN = 7
kLeftMotor2CAN = 2
kRightMotor1CAN = 3
kRightMotor2CAN = 5

# Encoders and their respective motor controllers.
kLeftEncoderSign = +1
kRightEncoderSign = -1  # reversed

# In meters, distance between wheels on each side of robot.
# MEASURE: Measure from the center of the left wheel to the center of the right wheel.
kTrackWidthMeters = 0.69
kDriveKinematics = DifferentialDriveKinematics(kTrackWidthMeters)
kDriveSpeedAtMaxRPM = 5.0  # meters per second

# Encoder counts per revolution/rotation.
# CHECK: If using a different encoder (like a through-bore), change this.
kEncoderCPR = 1024
kWheelDiameterMeters = 0.15

# Please calibrate to your robot
kEncoderPositionConversionFactor = 0.7

# Gyro config
kGyroReversed = -1  # make this +1 if not inverted


class AutoConstants:
    kUseSqrtControl = True  # compatibility with commands from https://github.com/epanov1602/CommandRevSwerve/blob/main/docs/Command_Driving_Aiming.md


# Camera Constants
kCameraOffsetX = 0.2
kCameraOffsetY = 0.0
kCameraHeight = 0.5
kCameraPitch = math.radians(-30.0)
# --- Camera Constants ---
# MEASURE: For each camera, measure its position relative to the center of the robot.
# X is forward, Y is left, Z is up.

# Camera 1 Constants (e.g., the left camera)
kCamera1Name = (
    "Arducam_OV9281_USB_Camera"  # CHECK: Must match the name in the PhotonVision UI
)
kCamera1OffsetX = 0.2  # meters
kCamera1OffsetY = 0.1  # meters
kCamera1Height = 0.5  # meters
kCamera1Pitch = math.radians(-30.0)

# Add more cameras by copying the block above and changing the numbers!
# kCamera2Name = "Camera_Right", kCamera2OffsetX = 0.2, etc.

# Shooter Constants
kShooterMotorCAN = 6
kIntakeMotorCAN = 4  # New motor for intake/feeding

kIntakeSpeed = -1  # Negative speed pulls the ball IN
kOuttakeSpeed = 0.5  # Positive speed pushes the ball OUT to the shooter


# Physics Model Constants
kGoalHeightMeters = 1.83
kShooterHeightMeters = 0.5
kShooterAngleDegrees = 70.0
kShooterWheelDiameterMeters = 0.1016
kShooterRecoveryFactor = 2.0
kShooterPhysicsTuning = 1.0

# PID Constants
# TUNE: If the shooter oscillates (shakes) or doesn't reach speed, change these.
kShooterP = 0.0005
kShooterI = 0.0
kShooterD = 0.0
kShooterFF = 0.00017
kShooterMaxOutput = 1.0
kShooterMinOutput = -1.0

kShooterMaxRPM = 5700
kShooterIntakeRPM = 1000  # RPM for when we are intaking
kShooterToleranceRPM = 50
kShooterMinRange = 1.0
kShooterMaxRange = 6.5

# Experimental Data: Distance (meters) -> Speed (RPM)
# TEST: Place the robot at these distances, find the best RPM, and update this table.
kShooterDistanceToRPM = {
    1.5: 2500,
    2.0: 2800,
    3.0: 3500,
    4.0: 4200,
    5.0: 4800,
    6.0: 5500,
}

redTargets = (9, 10)
blueTargets = (25, 26)
