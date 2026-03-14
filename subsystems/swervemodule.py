import math
from rev import SparkMax, SparkLowLevel, SparkBase, SparkClosedLoopController, SparkRelativeEncoder, \
    ResetMode, PersistMode
from wpilib import SmartDashboard
from wpimath.geometry import Rotation2d
from wpimath.kinematics import SwerveModuleState, SwerveModulePosition

import constants

class SwerveModule:
    def __init__(
        self,
        drivingCANId: int,
        turningCANId: int,
        turnMotorInverted = True,
        placement: str = "",
    ) -> None:
        """Constructs a swerve module using Rev (SparkMax) motor controllers."""
        self.desiredState = SwerveModuleState(0.0, Rotation2d())

        # Turning motor setup
        self.turningRevMotor = SparkMax(
            turningCANId, SparkLowLevel.MotorType.kBrushless
        )
        
        # In this implementation, we assume we have an absolute encoder plugged into the SparkMax
        useAbsoluteAngleGoals = True 
        
        self.turningRevMotor.configure(
            constants.getSwerveTurningMotorConfig(turnMotorInverted, useAbsoluteEncoderGoals=useAbsoluteAngleGoals),
            ResetMode.kResetSafeParameters,
            PersistMode.kPersistParameters)
            
        self.turningRevRelEncoder = self.turningRevMotor.getEncoder()
        self.turningRevAbsEncoder = self.turningRevMotor.getAbsoluteEncoder()
        
        self.turningRevAbsController = self.turningRevMotor.getClosedLoopController()

        # Driving motor setup
        self.drivingRevMotor = SparkMax(
            drivingCANId, SparkLowLevel.MotorType.kBrushless
        )
        self.drivingRevMotor.configure(
            constants.getSwerveDrivingMotorConfig(),
            ResetMode.kResetSafeParameters,
            PersistMode.kPersistParameters
        )
        self.drivingRevPIDController = self.drivingRevMotor.getClosedLoopController()
        self.drivingRevEncoder = self.drivingRevMotor.getEncoder()
        self.drivingRevEncoder.setPosition(0)

        # Initial state
        self.desiredState.angle = Rotation2d(self.getAbsoluteRadians())

    def getRelativeRadians(self) -> float:
        return self.turningRevRelEncoder.getPosition() / constants.ModuleConstants.kTurningReductionRatio * math.tau

    def getAbsoluteRadians(self) -> float:
        return self.turningRevAbsEncoder.getPosition()

    def setAbsoluteRadiansGoal(self, goal) -> None:
        self.turningRevAbsController.setReference(goal, SparkLowLevel.ControlType.kPosition)

    def getState(self) -> SwerveModuleState:
        return SwerveModuleState(self.drivingRevEncoder.getVelocity(), Rotation2d(self.getAbsoluteRadians()))

    def getPosition(self) -> SwerveModulePosition:
        return SwerveModulePosition(self.drivingRevEncoder.getPosition(), Rotation2d(self.getAbsoluteRadians()))

    def setDesiredState(self, desiredState: SwerveModuleState) -> None:
        if abs(desiredState.speed) < constants.ModuleConstants.kDrivingMinSpeedMetersPerSecond:
            self.stop()
            return

        # Optimize the reference state to avoid spinning further than 90 degrees.
        optimizedDesiredState = desiredState
        SwerveModuleState.optimize(optimizedDesiredState, Rotation2d(self.getAbsoluteRadians()))

        self.setAbsoluteRadiansGoal(optimizedDesiredState.angle.radians())
        self.drivingRevPIDController.setReference(
            optimizedDesiredState.speed, SparkLowLevel.ControlType.kVelocity
        )
        self.desiredState = desiredState

    def stop(self):
        self.drivingRevPIDController.setReference(0, SparkLowLevel.ControlType.kVelocity)
        angle = self.getAbsoluteRadians()
        self.setAbsoluteRadiansGoal(angle)
        if self.desiredState.speed != 0:
            self.desiredState = SwerveModuleState(speed=0, angle=Rotation2d(angle))

    def resetEncoders(self) -> None:
        self.drivingRevEncoder.setPosition(0)
