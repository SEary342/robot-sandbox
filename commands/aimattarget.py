import math
import wpilib
from commands2 import Command
from wpimath.controller import PIDController
import constants

class AimAtTarget(Command):
    """
    A command that rotates the robot to face the closest target AprilTag.
    Works for both Tank and Swerve drivetrains.
    """
    def __init__(self, drivetrain, target_tags: tuple[int, ...]) -> None:
        super().__init__()
        self.drivetrain = drivetrain
        self.target_tags = target_tags
        self.addRequirements(drivetrain)

        # Simple PID for rotation
        # TUNE these values!
        self.turnController = PIDController(0.05, 0, 0)
        self.turnController.enableContinuousInput(-180, 180)
        self.turnController.setTolerance(2.0) # 2 degrees tolerance

    def execute(self) -> None:
        target_rotation = self.drivetrain.getRotationToClosestTagInList(self.target_tags)
        current_rotation = self.drivetrain.getPose().rotation()

        # Calculate rotation speed
        # PIDController expects degrees or radians consistently. 
        # We'll use degrees for easier tuning.
        turn_speed = self.turnController.calculate(
            current_rotation.degrees(), 
            target_rotation.degrees()
        )

        # Clamp output
        turn_speed = max(-1.0, min(1.0, turn_speed))

        if constants.kSwerveInstalled:
            # For swerve, we just rotate (keep x/y at 0)
            self.drivetrain.drive(0, 0, turn_speed, True, False)
        else:
            # For tank, we rotate in place
            # arcadeDrive(fwd, rot) -> (0, turn_speed)
            self.drivetrain.arcadeDrive(0, turn_speed)

    def isFinished(self) -> bool:
        # Command continues as long as button is held, 
        # or we could finish if at tolerance.
        return self.turnController.atSetpoint()

    def end(self, interrupted: bool) -> None:
        self.drivetrain.stop()
