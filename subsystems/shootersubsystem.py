import rev
from commands2 import Subsystem
from wpilib import SmartDashboard
import constants

class ShooterSubsystem(Subsystem):
    def __init__(self):
        super().__init__()
        
        # Initialize the motor
        self.shooterMotor = rev.SparkMax(constants.kShooterMotorCAN, rev.SparkMax.MotorType.kBrushless)
        
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
        
        # Cache sorted keys for interpolation
        self.sorted_distances = sorted(constants.kShooterDistanceToRPM.keys())
            
        self.targetRPM = 0.0

    def setSpeedFromDistance(self, distance: float):
        """
        Sets the target RPM based on the distance to the target using interpolation.
        """
        if distance < constants.kShooterMinRange or distance > constants.kShooterMaxRange:
            print(f"Warning: Distance {distance:.2f}m is out of effective range.")
            
        # Linear Interpolation
        if distance <= self.sorted_distances[0]:
            target_rpm = constants.kShooterDistanceToRPM[self.sorted_distances[0]]
        elif distance >= self.sorted_distances[-1]:
            target_rpm = constants.kShooterDistanceToRPM[self.sorted_distances[-1]]
        else:
            # Find the two points bounding the distance
            for i in range(len(self.sorted_distances) - 1):
                d1 = self.sorted_distances[i]
                d2 = self.sorted_distances[i+1]
                if d1 <= distance <= d2:
                    rpm1 = constants.kShooterDistanceToRPM[d1]
                    rpm2 = constants.kShooterDistanceToRPM[d2]
                    target_rpm = rpm1 + (distance - d1) * (rpm2 - rpm1) / (d2 - d1)
                    break
        
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
