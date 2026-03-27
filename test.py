import constants
from typing import Optional
import math


class LauncherPhysics:
    """Pre-calculates constants for trajectory math to save CPU cycles."""

    def __init__(
        self,
        goal_h: float,
        shooter_h: float,
        angle_deg: float,
        wheel_d: float,
        recovery: float,
    ) -> None:
        self.h = goal_h - shooter_h
        self.theta = math.radians(angle_deg)
        self.tan_theta = math.tan(self.theta)
        self.cos_sq_theta = math.cos(self.theta) ** 2
        self.rpm_multiplier = (60.0 * recovery) / (math.pi * wheel_d)
        self.g = 9.806

    def calculate_rpm(self, distance_m: float) -> Optional[float]:
        try:
            num = self.g * (distance_m**2)
            den = 2 * self.cos_sq_theta * (distance_m * self.tan_theta - self.h)
            if den <= 0:
                return None
            return math.sqrt(num / den) * self.rpm_multiplier
        except (ValueError, ZeroDivisionError):
            return None


if __name__ == "__main__":
    launch = LauncherPhysics(
        constants.kGoalHeightMeters,
        constants.kShooterHeightMeters,
        constants.kShooterAngleDegrees,
        constants.kWheelDiameterMeters,
        constants.kShooterRecoveryFactor,
    )
    rpm = launch.calculate_rpm(5)
    print(rpm)
