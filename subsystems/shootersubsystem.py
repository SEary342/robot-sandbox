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
        self.leftIntakeLauncher = rev.SparkMax(
            constants.kLeftIntakeCAN, rev.SparkMax.MotorType.kBrushless
        )
        self.rightIntakeLauncher = rev.SparkMax(
            constants.kRightIntakeCAN, rev.SparkMax.MotorType.kBrushless
        )
        self.indexer = rev.SparkMax(
            constants.kIndexerCAN, rev.SparkMax.MotorType.kBrushed
        )

        self.pidController = self.rightIntakeLauncher.getClosedLoopController()

        feederConfig = rev.SparkMaxConfig()
        feederConfig.smartCurrentLimit(constants.kIndexerCurrentLimit)
        self.indexer.configure(feederConfig, rev.ResetMode.kResetSafeParameters, rev.PersistMode.kPersistParameters)

        launcherConfig = rev.SparkMaxConfig()
        launcherConfig.smartCurrentLimit(constants.kLauncherCurrentLimit)
        launcherConfig.voltageCompensation(12)
        launcherConfig.setIdleMode(rev.SparkMaxConfig.IdleMode.kCoast)
        
        # PID coefficients
        launcherConfig.closedLoop.pid(constants.kShooterP, constants.kShooterI, constants.kShooterD)
        launcherConfig.closedLoop.velocityFF(constants.kShooterFF)
        launcherConfig.closedLoop.outputRange(constants.kShooterMinOutput, constants.kShooterMaxOutput)

        self.rightIntakeLauncher.configure(launcherConfig, rev.ResetMode.kResetSafeParameters, rev.PersistMode.kPersistParameters)
        
        # Left motor follows right motor, but inverted
        leftConfig = rev.SparkMaxConfig()
        leftConfig.apply(launcherConfig)
        leftConfig.follow(self.rightIntakeLauncher, True)
        self.leftIntakeLauncher.configure(leftConfig, rev.ResetMode.kResetSafeParameters, rev.PersistMode.kPersistParameters)

        SmartDashboard.putNumber("Intaking feeder roller value", constants.FuelConstants.INDEXER_INTAKING_PERCENT)
        SmartDashboard.putNumber("Intaking intake roller value", constants.FuelConstants.INTAKE_INTAKING_PERCENT)
        SmartDashboard.putNumber("Launching feeder roller value", constants.FuelConstants.INDEXER_LAUNCHING_PERCENT)
        SmartDashboard.putNumber("Launching launcher roller value", constants.FuelConstants.LAUNCHING_LAUNCHER_PERCENT)
        SmartDashboard.putNumber("Launching spin-up feeder value", constants.FuelConstants.INDEXER_SPIN_UP_PRE_LAUNCH_PERCENT)

        self.physics_calc = LauncherPhysics(
            constants.kGoalHeightMeters,
            constants.kShooterHeightMeters,
            constants.kShooterAngleDegrees,
            constants.kShooterWheelDiameterMeters,
            constants.kShooterRecoveryFactor,
        )
        self.use_physics_model = True
        self.targetRPM = 0.0

    def setIntakeLauncherRoller(self, power: float):
        self.leftIntakeLauncher.set(power)
        self.rightIntakeLauncher.set(power)

    def setFeederRoller(self, power: float):
        self.indexer.set(power)

    def runOuttake(self):
        self.setFeederRoller(SmartDashboard.getNumber("Launching feeder roller value", constants.FuelConstants.INDEXER_LAUNCHING_PERCENT))

    def stop(self):
        self.indexer.stopMotor()
        self.leftIntakeLauncher.stopMotor()
        self.rightIntakeLauncher.stopMotor()

    def periodic(self):
        pass

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

    def setTargetRPM(self, rpm: float):
        self.targetRPM = rpm
        self.pidController.setReference(
            self.targetRPM, rev.SparkBase.ControlType.kVelocity
        )
