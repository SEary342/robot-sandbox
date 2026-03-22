from commands2 import Command
from subsystems.climbersubsystem import ClimberSubsystem
import constants

class ClimbDown(Command):
    """
    A command that powers the climber down.
    """
    def __init__(self, climber: ClimberSubsystem):
        super().__init__()
        self.climber = climber
        self.addRequirements(self.climber)

    def initialize(self):
        self.climber.setClimber(constants.kClimberDownPercent)

    def execute(self):
        pass

    def end(self, interrupted: bool):
        self.climber.stop()

    def isFinished(self) -> bool:
        return False
