import rev
from commands2 import Subsystem
from wpilib import SmartDashboard
from wpimath.interpolation import InterpolatingDoubleTreeMap
import constants

class ShooterSubsystem(Subsystem):
    def __init__(self):
        super().__init__()
        
        # Initialize the motor
        self.shooterMotor = rev.SparkMax(constants.kShooterMotorCAN, rev.SparkMax.MotorType.kBrushless)
        self.shooterMotor.restoreFactoryDefaults()
        
        # Configure the motor and PID controller
        config = rev.SparkBaseConfig()
        config.setIdleMode(rev.SparkBaseConfig.IdleMode.kCoast)
        config.closedLoop.pid(constants.kShooterP, constants.kShooterI, constants.kShooterD)
        config.closedLoop.velocityFF(constants.kShooterFF)
        config.closedLoop.outputRange(constants.kShooterMinOutput, constants.kShooterMaxOutput)
        
        # Apply configuration
        self.shooterMotor.configure(config, rev.ResetMode.kResetSafeParameters, rev.PersistMode.kPersistParameters)
        
        self.pidController = self.shooterMotor.getClosedLoopController()
        self.encoder = self.shooterMotor.getEncoder()
        
        # Initialize the interpolation table with experimental data
        self.rpmTable = InterpolatingDoubleTreeMap()
        for dist, rpm in constants.kShooterDistanceToRPM.items():
            self.rpmTable.put(dist, rpm)
            
        self.targetRPM = 0.0

    def setSpeedFromDistance(self, distance: float):
        """
        Sets the target RPM based on the distance to the target using interpolation.
        """
        if distance < constants.kShooterMinRange or distance > constants.kShooterMaxRange:
            print(f"Warning: Distance {distance:.2f}m is out of effective range.")
            
        target_rpm = self.rpmTable.get(distance)
        self.setTargetRPM(target_rpm)

    def setTargetRPM(self, rpm: float):
        """
        Sets the target RPM directly.
        """
        self.targetRPM = rpm
        self.pidController.setReference(self.targetRPM, rev.SparkBase.ControlType.kVelocity)

    def stop(self):
        """
        Stops the shooter motor.
        """
        self.targetRPM = 0.0
        self.shooterMotor.stopMotor()

    def isAtSpeed(self) -> bool:
        """
        Returns true if the shooter is at the target speed within tolerance.
        """
        return abs(self.encoder.getVelocity() - self.targetRPM) <= constants.kShooterToleranceRPM

    def periodic(self):
        # Publish data to SmartDashboard for debugging and driver feedback
        SmartDashboard.putNumber("Shooter/TargetRPM", self.targetRPM)
        SmartDashboard.putNumber("Shooter/CurrentRPM", self.encoder.getVelocity())
        SmartDashboard.putNumber("Shooter/AppliedOutput", self.shooterMotor.getAppliedOutput())
        SmartDashboard.putBoolean("Shooter/AtSpeed", self.isAtSpeed())
