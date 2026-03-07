import rev
import math
from typing import Optional
from commands2 import Subsystem
from wpilib import SmartDashboard
import constants


class LauncherPhysics:
    """Pre-calculates constants for trajectory math to save CPU cycles."""

    def __init__(
        self,
        goal_h: float,
        shooter_h: float,
        angle_deg: float,
        wheel_d: float,
        recovery: float,
    ) -> None:
        self.h = goal_h - shooter_h
        self.theta = math.radians(angle_deg)
        self.tan_theta = math.tan(self.theta)
        self.cos_sq_theta = math.cos(self.theta) ** 2
        self.rpm_multiplier = (60.0 * recovery) / (math.pi * wheel_d)
        self.g = 9.806

    def calculate_rpm(self, distance_m: float) -> Optional[float]:
        try:
            num = self.g * (distance_m**2)
            den = 2 * self.cos_sq_theta * (distance_m * self.tan_theta - self.h)
            if den <= 0:
                return None
            return math.sqrt(num / den) * self.rpm_multiplier
        except (ValueError, ZeroDivisionError):
            return None


class ShooterSubsystem(Subsystem):
    def __init__(self):
        super().__init__()

        # Initialize the motor
        self.shooterMotor = rev.SparkMax(
            constants.kShooterMotorCAN, rev.SparkMax.MotorType.kBrushless
        )
        self.intakeMotor = rev.SparkMax(
            constants.kIntakeMotorCAN, rev.SparkMax.MotorType.kBrushless
        )

        # --- Configure Intake Motor ---
        # This is the modern way to configure SparkMax controllers.
        intake_config = rev.SparkBaseConfig()
        intake_config.setIdleMode(rev.SparkBaseConfig.IdleMode.kCoast)
        intake_config.inverted(False)  # Set to True if it runs backwards
        self.intakeMotor.configure(
            intake_config,
            rev.ResetMode.kResetSafeParameters,
            rev.PersistMode.kPersistParameters,
        )

        # --- Configure Shooter Motor ---
        # This configures the "brain" inside the motor controller to keep speed constant.
        config = rev.SparkBaseConfig()
        config.setIdleMode(rev.SparkBaseConfig.IdleMode.kCoast)
        config.closedLoop.pid(
            constants.kShooterP, constants.kShooterI, constants.kShooterD
        )
        config.closedLoop.velocityFF(constants.kShooterFF)
        config.closedLoop.outputRange(
            constants.kShooterMinOutput, constants.kShooterMaxOutput
        )

        # Apply configuration
        self.shooterMotor.configure(
            config,
            rev.ResetMode.kResetSafeParameters,
            rev.PersistMode.kPersistParameters,
        )

        self.pidController = self.shooterMotor.getClosedLoopController()
        self.encoder = self.shooterMotor.getEncoder()

        # Physics Calculator Instance
        self.physics_calc = LauncherPhysics(
            constants.kGoalHeightMeters,
            constants.kShooterHeightMeters,
            constants.kShooterAngleDegrees,
            constants.kShooterWheelDiameterMeters,
            constants.kShooterRecoveryFactor,  # Usually 2.0 for hooded shooters
        )

        # Toggle state
        self.use_physics_model = False

        # Cache sorted keys for interpolation
        # This helps us calculate speed for distances between our known points.
        self.sorted_distances = sorted(constants.kShooterDistanceToRPM.keys())
        self.targetRPM = 0.0

    def toggleShooterLogic(self):
        """Switches between Interpolation and Physics models."""
        self.use_physics_model = not self.use_physics_model
        SmartDashboard.putBoolean("Shooter/UsingPhysics", self.use_physics_model)

    def setSpeedFromDistance(self, distance: float):
        """Sets target RPM based on distance using the selected implementation."""
        target_rpm: Optional[float] = 0.0

        if self.use_physics_model:
            # --- PURE PHYSICS SOLUTION ---
            calculated_rpm = self.physics_calc.calculate_rpm(distance)
            if calculated_rpm is not None:
                # Apply a tuning factor (e.g. 1.05) to account for air resistance/friction
                target_rpm = calculated_rpm * constants.kShooterPhysicsTuning
            else:
                target_rpm = 0.0  # Or maintain last valid speed
        else:
            # --- INTERPOLATED SOLUTION ---
            if distance <= self.sorted_distances[0]:
                target_rpm = constants.kShooterDistanceToRPM[self.sorted_distances[0]]
            elif distance >= self.sorted_distances[-1]:
                target_rpm = constants.kShooterDistanceToRPM[self.sorted_distances[-1]]
            else:
                for i in range(len(self.sorted_distances) - 1):
                    d1, d2 = self.sorted_distances[i], self.sorted_distances[i + 1]
                    if d1 <= distance <= d2:
                        rpm1, rpm2 = (
                            constants.kShooterDistanceToRPM[d1],
                            constants.kShooterDistanceToRPM[d2],
                        )
                        target_rpm = rpm1 + (distance - d1) * (rpm2 - rpm1) / (d2 - d1)
                        break

        self.setTargetRPM(target_rpm)

    def runIntake(self):
        """Runs the intake motor to acquire a note."""
        self.intakeMotor.set(constants.kIntakeSpeed)

    def runOuttake(self):
        """Runs the intake motor to feed a note to the shooter."""
        self.intakeMotor.set(constants.kOuttakeSpeed)

    def stopIntake(self):
        """Stops only the intake motor."""
        self.intakeMotor.stopMotor()

    def setTargetRPM(self, rpm: float):
        """
        Sets the target RPM directly.
        """
        self.targetRPM = rpm
        self.pidController.setReference(
            self.targetRPM, rev.SparkBase.ControlType.kVelocity
        )

    def stop(self):
        """
        Stops the shooter and intake motors.
        """
        self.targetRPM = 0.0
        self.shooterMotor.stopMotor()
        self.intakeMotor.stopMotor()

    def isAtSpeed(self) -> bool:
        """
        Returns true if the shooter is at the target speed within tolerance.
        """
        return (
            abs(self.encoder.getVelocity() - self.targetRPM)
            <= constants.kShooterToleranceRPM
        )

    def periodic(self):
        # Publish data to SmartDashboard for debugging and driver feedback
        # STUDENTS: This sends the numbers to the laptop screen so you can see them!
        SmartDashboard.putNumber("Shooter/TargetRPM", self.targetRPM)
        SmartDashboard.putNumber("Shooter/CurrentRPM", self.encoder.getVelocity())
        SmartDashboard.putNumber(
            "Shooter/AppliedOutput", self.shooterMotor.getAppliedOutput()
        )
        SmartDashboard.putBoolean("Shooter/UsingPhysics", self.use_physics_model)
        SmartDashboard.putBoolean("Shooter/AtSpeed", self.isAtSpeed())
