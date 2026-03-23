from commands2 import Command
from subsystems.shootersubsystem import ShooterSubsystem
from wpilib import SmartDashboard
import constants

class Eject(Command):
    """
    A command that ejects fuel back out the intake.
    """
    def __init__(self, shooter: ShooterSubsystem):
        super().__init__()
        self.shooter = shooter
        self.addRequirements(self.shooter)

    def initialize(self):
        # Java logic: eject speed = -1 * dashboard value OR -0.8 default
        # SmartDashboard value name might vary, so we fallback to FuelConstants.INTAKE_EJECT_PERCENT
        eject_speed = SmartDashboard.getNumber(
            "Intaking intake roller value", 
            constants.FuelConstants.INTAKE_EJECT_PERCENT
        )
        # Match Java's -1 multiplier if intended, though constants.INTAKE_EJECT_PERCENT is already -0.8
        self.shooter.setIntakeLauncherRoller(-1 * abs(eject_speed))
        
        self.shooter.setFeederRoller(
            SmartDashboard.getNumber(
                "Launching feeder roller value", 
                constants.FuelConstants.INDEXER_LAUNCHING_PERCENT
            )
        )

    def execute(self):
        pass

    def end(self, interrupted: bool):
        self.shooter.stop()

    def isFinished(self) -> bool:
        return False
