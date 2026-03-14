# Master Constants File
import math

# Toggle this to switch between Tank and Swerve drivetrain code/IDs
kSwerveInstalled = True

# Common Constants
kDriverControllerPort = 0
kTestBench = False
kInputSlewRate = 2.0
kLaunchMotorCurrentLimit = 40
kDriveMotorCurrentLimit = 40

# Import Shooter/Camera Constants (Common to both)
from shooter_constants import *

if kSwerveInstalled:
    from swerve_constants import *
else:
    from tank_constants import *
