from typing import Optional, Callable
from commands2 import Command
from subsystems.shootersubsystem import ShooterSubsystem
from wpilib import SmartDashboard
import constants

class Launch(Command):
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
            self.shooter.setIntakeLauncherRoller(
                SmartDashboard.getNumber("Launching launcher roller value", constants.FuelConstants.LAUNCHING_LAUNCHER_PERCENT)
            )
            
        self.shooter.setFeederRoller(
            SmartDashboard.getNumber("Launching feeder roller value", constants.FuelConstants.INDEXER_LAUNCHING_PERCENT)
        )

    def execute(self):
        pass

    def end(self, interrupted: bool):
        # We might want to stop when it's done launching
        pass

    def isFinished(self) -> bool:
        return False
