# Toggle this to switch between Tank and Swerve drivetrain code/IDs
kSwerveInstalled = False

# Common Constants
kDriverControllerPort = 0
kTestBench = False
kInputSlewRate = 2.0
kDriveMotorCurrentLimit = 60

class OIConstants:
    kDriverControllerPort = 0
    kDriveDeadband = 0.05

# Import Shooter/Camera/Climber Constants (Common to both)
from shooter_constants import *  # noqa: E402, F403
from climber_constants import *  # noqa: E402, F403

if kSwerveInstalled:
    from swerve_constants import *  # noqa: E402, F403
else:
    from tank_constants import *  # noqa: E402, F403
