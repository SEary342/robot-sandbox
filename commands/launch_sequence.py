from commands2 import SequentialCommandGroup
from commands.spin_up import SpinUp
from commands.launch import Launch
from subsystems.shootersubsystem import ShooterSubsystem
import constants


class LaunchSequence(SequentialCommandGroup):
    """
    A command that spins up the shooter based on distance to the speaker
    and then launches the fuel.
    """

    def __init__(self, shooter: ShooterSubsystem, drivetrain):
        super().__init__()

        def get_distance_to_speaker() -> float:
            target_tags = constants.targets
            # Both DriveSubsystem and SwerveDriveSubsystem implement this method
            return drivetrain.getDistanceToClosestTagInList(target_tags)

        self.addCommands(
            SpinUp(shooter, get_distance_to_speaker).withTimeout(
                constants.FuelConstants.SPIN_UP_SECONDS
            ),
            Launch(shooter, get_distance_to_speaker),
        )
