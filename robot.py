import wpilib
import commands2
import wpimath.geometry
import wpimath.kinematics
import math
import ntcore # Added for NetworkTables
from photonlibpy.photonCamera import PhotonCamera
from pathplannerlib.auto import AutoBuilder
from pathplannerlib.config import RobotConfig, PIDConstants
from pathplannerlib.controller import PPHolonomicDriveController

# Constants
CAMERA_HEIGHT_METERS = 0.5
TARGET_HEIGHT_METERS = 2.5
CAMERA_PITCH_RADIANS = math.radians(30.0)
SHOOTER_PITCH_RADIANS = math.radians(45.0)
SHOOTER_HEIGHT_METERS = 0.5
GRAVITY = 9.81
YAW_TOLERANCE = 3.0

class Shooter(commands2.Subsystem):
    def __init__(self):
        super().__init__()
    def set_velocity(self, velocity: float): pass
    def stop(self): pass

class Climber(commands2.Subsystem):
    def __init__(self):
        super().__init__()
    def deploy_hooks(self): pass
    def climb(self): pass

class MyRobot(commands2.TimedCommandRobot):
    def robotInit(self):
        # --- NetworkTables / Gyro Initialization ---
        self.inst = ntcore.NetworkTableInstance.getDefault()
        self.sensehat_table = self.inst.getTable("SenseHat")
        # The 'yaw' entry published by your Pi script
        self.gyro_yaw_entry = self.sensehat_table.getDoubleTopic("yaw").getEntry(0.0)
        
        # --- PhotonVision Initialization ---
        self.camera = PhotonCamera("Arducam_OV9281_USB_Camera")

        # --- Subsystems ---
        self.shooter = Shooter()
        self.climber = Climber()

        # --- Drivetrain & PathPlanner State ---
        # We initialize the pose using the current gyro heading
        self.pose = wpimath.geometry.Pose2d(0, 0, self.get_gyro_rotation())

        def get_pose():
            # In a full swerve implementation, you would use a PoseEstimator here.
            # For now, we update the rotation part of the pose with the live gyro data.
            return wpimath.geometry.Pose2d(self.pose.translation(), self.get_gyro_rotation())

        def reset_pose(new_pose):
            self.pose = new_pose
            # Note: You may need a 'gyro_offset' if you want to zero the gyro in code.

        def get_robot_relative_speeds():
            return wpimath.kinematics.ChassisSpeeds()

        def drive_robot_relative(speeds):
            # Drive logic goes here
            pass

        self.auto_chooser = None

        try:
            config = RobotConfig.fromGUISettings()
            AutoBuilder.configure(
                get_pose,
                reset_pose,
                get_robot_relative_speeds,
                drive_robot_relative,
                PPHolonomicDriveController(
                    PIDConstants(5.0, 0.0, 0.0),
                    PIDConstants(5.0, 0.0, 0.0),
                ),
                config,
                self.shouldFlipPath,
                self
            )
            self.auto_chooser = AutoBuilder.buildAutoChooser()
            wpilib.SmartDashboard.putData("Auto Mode", self.auto_chooser)
        except Exception as e:
            wpilib.reportError(f"PathPlanner configuration failed: {e}")

    def get_gyro_rotation(self) -> wpimath.geometry.Rotation2d:
        """Helper to get Rotation2d from the Sense HAT gyro data."""
        # Waveshare/AHRS output is usually CCW positive (standard).
        # If your robot turns left and the value decreases, add a '-' before the get().
        yaw_deg = self.gyro_yaw_entry.get()
        return wpimath.geometry.Rotation2d.fromDegrees(yaw_deg)

    def shouldFlipPath(self):
        return wpilib.DriverStation.getAlliance() == wpilib.DriverStation.Alliance.kRed

    def robotPeriodic(self):
        super().robotPeriodic()
        
        # Update Dashboard with Gyro info
        wpilib.SmartDashboard.putNumber("Gyro/Yaw", self.get_gyro_rotation().degrees())

        ready, velocity = self.get_shooting_solution()
        if self.isEnabled():
            if ready:
                self.shooter.set_velocity(velocity)
            else:
                self.shooter.stop()
        self.update_dashboard(ready, velocity)

    def update_dashboard(self, ready, velocity):
        wpilib.SmartDashboard.putBoolean("Shooter/Ready", ready)
        wpilib.SmartDashboard.putNumber("Shooter/TargetVelocity", velocity)

    def get_shooting_solution(self):
        result = self.camera.getLatestResult()
        if not result.hasTargets():
            return False, 0.0

        target = result.getBestTarget()
        if abs(target.getYaw()) > YAW_TOLERANCE:
            return False, 0.0

        distance = (TARGET_HEIGHT_METERS - CAMERA_HEIGHT_METERS) / math.tan(
            CAMERA_PITCH_RADIANS + math.radians(target.getPitch())
        )

        if distance < 1.5 or distance > 6.0:
            return False, 0.0

        h = TARGET_HEIGHT_METERS - SHOOTER_HEIGHT_METERS
        if distance * math.tan(SHOOTER_PITCH_RADIANS) <= h:
            return False, 0.0

        velocity = (distance / math.cos(SHOOTER_PITCH_RADIANS)) * math.sqrt(
            GRAVITY / (2 * (distance * math.tan(SHOOTER_PITCH_RADIANS) - h))
        )
        return True, velocity

    def autonomousInit(self):
        if self.auto_chooser:
            self.autonomous_command = self.auto_chooser.getSelected()
            if self.autonomous_command:
                self.autonomous_command.schedule()

    def teleopInit(self):
        if hasattr(self, "autonomous_command") and self.autonomous_command:
            self.autonomous_command.cancel()

if __name__ == "__main__":
    wpilib.run(MyRobot)