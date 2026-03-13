#
# Copyright (c) FIRST and other WPILib contributors.
# Open Source Software; you can modify and/or share it under the terms of
# the WPILib BSD license file in the root directory of this project.
#

import math
from wpimath.geometry import Translation2d
from wpimath.kinematics import SwerveDrive4Kinematics


class SwerveConstants:
    """
    Holds all of the constants for the swerve drive.

    NOTE: All values are example values only - DO NOT USE THESE ON A REAL ROBOT!
    These values must be tuned for your specific robot.
    """

    # --- Robot Physical Characteristics ---
    # The robot's trackwidth, which is the distance between the centers of the right and left wheels.
    kTrackWidth = 0.5  # meters
    # The robot's wheelbase, which is the distance between the centers of the front and back wheels.
    kWheelBase = 0.5  # meters

    # The locations of the swerve modules relative to the robot center.
    # The convention is: +X is forward, +Y is left.
    kKinematics = SwerveDrive4Kinematics(
        Translation2d(kWheelBase / 2, kTrackWidth / 2),  # Front Left
        Translation2d(kWheelBase / 2, -kTrackWidth / 2),  # Front Right
        Translation2d(-kWheelBase / 2, kTrackWidth / 2),  # Back Left
        Translation2d(-kWheelBase / 2, -kTrackWidth / 2),  # Back Right
    )

    # --- Motor CAN IDs ---
    # It's a good idea to label your controllers with these IDs.
    kFrontLeftDrivingCanId = 7
    kFrontLeftTurningCanId = 8
    kRearLeftDrivingCanId = 6
    kRearLeftTurningCanId = 5
    kFrontRightDrivingCanId = 3
    kFrontRightTurningCanId = 4
    kRearRightDrivingCanId = 2
    kRearRightTurningCanId = 1

    # --- Physical Properties ---
    # You specified 3-inch wheels.
    kWheelDiameterMeters = 0.0762  # 3 inches in meters

    # This is an example gearing for a MAXSwerve module with NEO motors.
    # Update with your robot's actual gearing.
    # Drive motor gear ratio (motor rotations per wheel rotation).
    kDriveMotorGearRatio = 6.75
    # Turning motor gear ratio (motor rotations per module rotation).
    kTurningMotorGearRatio = 150.0 / 7.0

    # --- Encoder Conversion Factors ---
    # These convert from motor-level units (rotations, RPM) to robot-level units (meters, m/s, radians, rad/s).

    # Drive: Motor rotations to wheel distance (meters)
    kDriveEncoderPositionFactor = (math.pi * kWheelDiameterMeters) / kDriveMotorGearRatio
    # Drive: Motor RPM to wheel velocity (m/s)
    kDriveEncoderVelocityFactor = kDriveEncoderPositionFactor / 60.0

    # Turn: Motor rotations to module angle (radians)
    kTurningEncoderPositionFactor = (2 * math.pi) / kTurningMotorGearRatio
    # Turn: Motor RPM to module angular velocity (rad/s)
    kTurningEncoderVelocityFactor = kTurningEncoderPositionFactor / 60.0

    # --- PID & Feedforward Gains (MUST BE TUNED) ---
    # These values are highly dependent on the robot's weight, motors, and gearing.
    # Start with small values and tune systematically.

    # Turning PID Controller
    kTurningP = 1.0  # Proportional gain

    # Driving PID Controller
    kDrivingP = 0.04  # Proportional gain
    # Driving Feedforward
    # A simple feedforward is 1 / max_velocity_in_RPM.
    # You can find max free speed RPM of your motor from its datasheet. (e.g., NEO is 5676)
    kDrivingFF = 1 / 5676

    # --- Robot Speed and Acceleration Limits ---
    kMaxSpeedMetersPerSecond = 4.5
    kMaxAngularSpeedRadiansPerSecond = math.pi * 4
    # Slew rate limiters prevent jerky movements by limiting acceleration.
    kMagnitudeSlewRate = 1.8  # m/s per second
    kRotationalSlewRate = 2.0  # rad/s per second