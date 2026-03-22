import rev
from commands2 import Subsystem
import constants

class ClimberSubsystem(Subsystem):
    def __init__(self):
        super().__init__()
        
        # create brushed motor for climber
        self.climberMotor = rev.SparkMax(constants.kClimberCAN, rev.SparkMax.MotorType.kBrushed)
        
        # create the configuration for the climb motor
        climbConfig = rev.SparkMaxConfig()
        climbConfig.smartCurrentLimit(constants.kClimberCurrentLimit)
        climbConfig.setIdleMode(rev.SparkMaxConfig.IdleMode.kBrake)
        
        self.climberMotor.configure(
            climbConfig, 
            rev.ResetMode.kResetSafeParameters, 
            rev.PersistMode.kPersistParameters
        )

    def setClimber(self, power: float):
        """Sets the climber motor percent power."""
        self.climberMotor.set(power)

    def stop(self):
        """Stops the climber motor."""
        self.climberMotor.set(0)

    def periodic(self):
        pass
