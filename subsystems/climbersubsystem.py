import rev
from wpilib import SmartDashboard
from commands2 import Subsystem
import constants


class ClimberSubsystem(Subsystem):
    def __init__(self):
        super().__init__()

        # create brushed motor for climber
        self.climberMotor = rev.SparkMax(
            constants.kClimberCAN, rev.SparkMax.MotorType.kBrushless
        )

        # create the configuration for the climb motor
        climbConfig = rev.SparkMaxConfig()
        climbConfig.smartCurrentLimit(constants.kClimberCurrentLimit)
        climbConfig.setIdleMode(rev.SparkMaxConfig.IdleMode.kBrake)

        # Configure software limits
        climbConfig.softLimit.forwardSoftLimitEnabled(constants.kEnableLimit).forwardSoftLimit(
            constants.kClimberForwardSoftLimit
        )
        climbConfig.softLimit.reverseSoftLimitEnabled(constants.kEnableLimit).reverseSoftLimit(
            constants.kClimberReverseSoftLimit
        )

        self.climberMotor.configure(
            climbConfig,
            rev.ResetMode.kResetSafeParameters,
            rev.PersistMode.kPersistParameters,
        )

    def setClimber(self, power: float):
        """Sets the climber motor percent power."""
        self.climberMotor.set(power)

    def stop(self):
        """Stops the climber motor."""
        self.climberMotor.set(0)

    def periodic(self):
        SmartDashboard.putNumber(
            "Climber/Position", self.climberMotor.getEncoder().getPosition()
        )
