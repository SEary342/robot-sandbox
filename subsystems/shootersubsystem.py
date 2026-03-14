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
        intake_config = rev.SparkBaseConfig()
        intake_config.setIdleMode(rev.SparkBaseConfig.IdleMode.kCoast)
        intake_config.smartCurrentLimit(constants.kLaunchMotorCurrentLimit)
        intake_config.inverted(True)
        self.intakeMotor.configure(
            intake_config,
            rev.ResetMode.kResetSafeParameters,
            rev.PersistMode.kPersistParameters,
        )

        # --- Configure Shooter Motor ---
        config = rev.SparkBaseConfig()
        config.setIdleMode(rev.SparkBaseConfig.IdleMode.kCoast)
        config.smartCurrentLimit(constants.kLaunchMotorCurrentLimit)
        config.closedLoop.pid(
            constants.kShooterP, constants.kShooterI, constants.kShooterD
        )
        config.closedLoop.velocityFF(constants.kShooterFF)
        config.closedLoop.outputRange(
            constants.kShooterMinOutput, constants.kShooterMaxOutput
        )

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
            constants.kShooterRecoveryFactor,
        )

        self.use_physics_model = False
        self.targetRPM = 0.0

    def toggleShooterLogic(self):
        """Switches between Interpolation and Physics models."""
        self.use_physics_model = not self.use_physics_model
        SmartDashboard.putBoolean("Shooter/UsingPhysics", self.use_physics_model)

    def setSpeedFromDistance(self, distance: float):
        """Sets target RPM based on distance using the selected implementation."""
        target_rpm: float = 0.0

        if self.use_physics_model:
            calculated_rpm = self.physics_calc.calculate_rpm(distance)
            if calculated_rpm is not None:
                target_rpm = calculated_rpm * constants.kShooterPhysicsTuning
        else:
            # Using the new LookupTable from constants
            target_rpm = constants.kShooterDistanceToRPM.interpolate(distance)

        self.setTargetRPM(target_rpm)

    def runIntake(self):
        self.intakeMotor.set(constants.kIntakeSpeed)

    def runOuttake(self):
        self.intakeMotor.set(constants.kOuttakeSpeed)

    def stopIntake(self):
        self.intakeMotor.stopMotor()

    def setTargetRPM(self, rpm: float):
        self.targetRPM = rpm
        self.pidController.setReference(
            self.targetRPM, rev.SparkBase.ControlType.kVelocity
        )

    def stop(self):
        self.targetRPM = 0.0
        self.shooterMotor.stopMotor()
        self.intakeMotor.stopMotor()

    def ready_to_fire(self) -> str:
        """
        Returns an empty string if ready to fire, or a reason why it's not ready.
        Useful for driver feedback on the dashboard.
        """
        if self.targetRPM <= 0:
            return "No target RPM set"
            
        current_rpm = self.encoder.getVelocity()
        if current_rpm < self.targetRPM - constants.kShooterToleranceRPM:
            return f"Under speed: {current_rpm:.0f} < {self.targetRPM:.0f}"
        if current_rpm > self.targetRPM + constants.kShooterToleranceRPM:
            return f"Over speed: {current_rpm:.0f} > {self.targetRPM:.0f}"
            
        return "" # Ready!

    def isAtSpeed(self) -> bool:
        return self.ready_to_fire() == ""

    def periodic(self):
        SmartDashboard.putNumber("Shooter/TargetRPM", self.targetRPM)
        SmartDashboard.putNumber("Shooter/CurrentRPM", self.encoder.getVelocity())
        SmartDashboard.putBoolean("Shooter/AtSpeed", self.isAtSpeed())
        SmartDashboard.putString("Shooter/Status", self.ready_to_fire())
