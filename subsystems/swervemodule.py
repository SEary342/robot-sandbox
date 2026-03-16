from rev import SparkMax, SparkLowLevel, ResetMode, PersistMode
from wpimath.geometry import Rotation2d
from wpimath.kinematics import SwerveModuleState, SwerveModulePosition
import constants

class SwerveModule:
    def __init__(
        self,
        drivingCANId: int,
        turningCANId: int,
        placement: str = "",
    ) -> None:
        self.desiredState = SwerveModuleState(0.0, Rotation2d())

        # Turning motor setup
        self.turningRevMotor = SparkMax(turningCANId, SparkLowLevel.MotorType.kBrushless)
        
        # WE DO NOT factory reset this motor. 
        # This allows the Hardware Client settings (Offsets, PID, Inversion) to persist.
        self.turningRevMotor.configure(
            constants.getSwerveTurningMotorConfig(),
            ResetMode.kNoResetSafeParameters, 
            PersistMode.kPersistParameters
        )
            
        self.turningRevRelEncoder = self.turningRevMotor.getEncoder()
        self.turningRevAbsEncoder = self.turningRevMotor.getAbsoluteEncoder()
        self.turningRevAbsController = self.turningRevMotor.getClosedLoopController()

        # Driving motor setup
        self.drivingRevMotor = SparkMax(drivingCANId, SparkLowLevel.MotorType.kBrushless)
        
        # We DO factory reset the drive motor because its config is fully handled in code.
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

    def resetEncoders(self) -> None:
        self.drivingRevEncoder.setPosition(0)