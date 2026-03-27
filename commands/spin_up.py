from typing import Optional, Callable
from commands2 import Command
from subsystems.shootersubsystem import ShooterSubsystem
from wpilib import SmartDashboard
import constants

class SpinUp(Command):
    def __init__(self, shooter: ShooterSubsystem, distance_supplier: Optional[Callable[[], float]] = None):
        super().__init__()
        self.shooter = shooter
        self.distance_supplier = distance_supplier
        self.addRequirements(self.shooter)

    def initialize(self):
        if self.distance_supplier is not None:
            distance = self.distance_supplier()
            # If no target found (distance 0 or None), use a safe fallback distance (e.g. subwoofer shot)
            if not distance:
                distance = 3.5
            self.shooter.setSpeedFromDistance(distance)
        else:
            # Fallback to fixed percent if no distance supplier
            # Note: ShooterSubsystem uses PID for RPM, so setting percent power 
            # might conflict if we don't switch modes. 
            # But the Java code uses fixed percent.
            self.shooter.setIntakeLauncherRoller(
                SmartDashboard.getNumber("Launching launcher roller value", constants.FuelConstants.LAUNCHING_LAUNCHER_PERCENT)
            )
        
        self.shooter.setFeederRoller(
            SmartDashboard.getNumber("Launching spin-up feeder value", constants.FuelConstants.INDEXER_SPIN_UP_PRE_LAUNCH_PERCENT)
        )

    def execute(self):
        pass

    def end(self, interrupted: bool):
        pass

    def isFinished(self) -> bool:
        # If we use PID, we might want to wait until it's at speed.
        # However, the sequence has a timeout (FuelConstants.SPIN_UP_SECONDS).
        return False
