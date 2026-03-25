from commands2 import Command
from subsystems.shootersubsystem import ShooterSubsystem
from wpilib import SmartDashboard
import constants


class Intake(Command):
    def __init__(self, shooter: ShooterSubsystem):
        super().__init__()
        self.shooter = shooter
        self.addRequirements(self.shooter)

    def initialize(self):
        self.shooter.setIntakeLauncherRoller(
            SmartDashboard.getNumber(
                "Intaking intake roller value",
                constants.FuelConstants.INTAKE_INTAKING_PERCENT,
            )
        )
        self.shooter.setFeederRoller(
            SmartDashboard.getNumber(
                "Intaking feeder roller value",
                constants.FuelConstants.INDEXER_INTAKING_PERCENT,
            )
        )

    def execute(self):
        pass

    def end(self, interrupted: bool):
        self.shooter.stop()

    def isFinished(self) -> bool:
        return False
