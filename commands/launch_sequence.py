from commands2 import SequentialCommandGroup
from commands.spin_up import SpinUp
from commands.launch import Launch
from subsystems.shootersubsystem import ShooterSubsystem
import constants


class LaunchSequence(SequentialCommandGroup):
    def __init__(self, shooter: ShooterSubsystem, drivetrain, rpm_override = None):
        super().__init__()

        # Logic to decide which RPM to use
        def get_target_rpm() -> float:
            if rpm_override is not None:
                return rpm_override
            
            # Fallback to distance calculation if no override
            target_tags = constants.targets
            distance = drivetrain.getDistanceToClosestTagInList(target_tags)
            # You might need a helper in ShooterSubsystem to return the 
            # calculated RPM without setting it immediately, 
            # but for a simple override, we can just use the float.
            return 5000.0 # Your current hardcoded default

        self.addCommands(
            # Pass the logic into the commands
            # Note: You'll need to update SpinUp/Launch to accept a 
            # lambda/callable for RPM if they don't already.
            SpinUp(shooter, get_target_rpm, rpm_override).withTimeout(
                constants.FuelConstants.SPIN_UP_SECONDS
            ),
            Launch(shooter, get_target_rpm),
        )
