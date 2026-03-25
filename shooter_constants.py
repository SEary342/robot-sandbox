import math
import numpy as np
from typing import Dict


class LookupTable:
    """
    Allows you to lookup points in a pre-calibrated table, using linear interpolation.
    """

    def __init__(self, points: Dict[float, float]):
        sorted_points = sorted(points.items())
        self.x = np.array([x for x, y in sorted_points], dtype=np.float32)
        self.y = np.array([y for x, y in sorted_points], dtype=np.float32)

    def interpolate(self, x: float):
        return float(np.interp(x, self.x, self.y))


class FuelConstants:
    INDEXER_INTAKING_PERCENT = 0.4
    INDEXER_LAUNCHING_PERCENT = -0.3
    INDEXER_SPIN_UP_PRE_LAUNCH_PERCENT = 0.5

    INTAKE_INTAKING_PERCENT = 0.6
    LAUNCHING_LAUNCHER_PERCENT = 0.85
    INTAKE_EJECT_PERCENT = -0.8

    SPIN_UP_SECONDS = 0.75


# Shooter Constants
# TODO these need to be changed on the robot to not collide with swerve CAN ids
kRightIntakeCAN = 6
kLeftIntakeCAN = 5
kIndexerCAN = 8

kIndexerCurrentLimit = 80
kLauncherCurrentLimit = 80

# Physics Model Constants
kGoalHeightMeters = 1.83
kShooterHeightMeters = 0.5
kShooterAngleDegrees = 70.0
kShooterWheelDiameterMeters = 0.1016
kShooterRecoveryFactor = 2.0
kShooterPhysicsTuning = 1.0

# PID Constants
kShooterP = 0.0005
kShooterI = 0.0
kShooterD = 0.0
kShooterFF = 0.00017
kShooterMaxOutput = 1.0
kShooterMinOutput = -1.0

kShooterMaxRPM = 5700
kShooterIntakeRPM = 1000
kShooterToleranceRPM = 50
kShooterMinRange = 1.0
kShooterMaxRange = 6.5

# Experimental Data: Distance (meters) -> Speed (RPM)
kShooterDistanceToRPM = LookupTable(
    {
        1.5: 2500,
        2.0: 2800,
        3.0: 3500,
        4.0: 4200,
        5.0: 4800,
        6.0: 5500,
    }
)

redTargets = (9, 10)
blueTargets = (25, 26)

# Camera Constants
kCameraOffsetX = 0.2
kCameraOffsetY = 0.0
kCameraHeight = 0.5
kCameraPitch = math.radians(-30.0)

kCamera1Name = "Arducam_OV9281_USB_Camera"
kCamera1OffsetX = 0.2
kCamera1OffsetY = 0.1
kCamera1Height = 0.5
kCamera1Pitch = math.radians(-30.0)
