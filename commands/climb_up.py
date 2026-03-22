from commands2 import Command
from subsystems.climbersubsystem import ClimberSubsystem
import constants

class ClimbUp(Command):
    """
    A command that powers the climber up.
    """
    def __init__(self, climber: ClimberSubsystem):
        super().__init__()
        self.climber = climber
        self.addRequirements(self.climber)

    def initialize(self):
        self.climber.setClimber(constants.kClimberUpPercent)

    def execute(self):
        pass

    def end(self, interrupted: bool):
        self.climber.stop()

    def isFinished(self) -> bool:
        return False
