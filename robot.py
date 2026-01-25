import wpilib
import commands2
import wpimath.geometry as geom
import wpimath.kinematics as kinematics
import wpimath.estimator as estimator
import ntcore
import math

from photonlibpy.photonCamera import PhotonCamera
from photonlibpy.photonPoseEstimator import PhotonPoseEstimator, PoseStrategy
from robotpy_apriltag import AprilTagFieldLayout, AprilTagField

from pathplannerlib.auto import AutoBuilder
from pathplannerlib.config import RobotConfig, PIDConstants
from pathplannerlib.controller import PPHolonomicDriveController

# --- Physical Constants ---
CAMERA_OFFSET_X = 0.2  # Meters forward from center
CAMERA_OFFSET_Y = 0.0  # Meters left/right from center
CAMERA_HEIGHT = 0.5    # Meters from ground
CAMERA_PITCH = math.radians(-30.0) # 30 degrees tilted UP

TARGET_HEIGHT_METERS = 2.5 # Target height (e.g. Speaker)
SHOOTER_HEIGHT_METERS = 0.5
GRAVITY = 9.81

class MyRobot(commands2.TimedCommandRobot):
    def robotInit(self):
        # 1. NetworkTables & Sense HAT Gyro
        self.inst = ntcore.NetworkTableInstance.getDefault()
        self.sensehat_table = self.inst.getTable("SenseHat")
        self.gyro_yaw_entry = self.sensehat_table.getDoubleTopic("yaw").getEntry(0.0)

        # 2. PhotonVision & AprilTag Setup
        self.camera = PhotonCamera("Arducam_OV9281_USB_Camera")
        
        # Load the 2024 Field Layout for X,Y coordinate translation
        self.field_layout = AprilTagFieldLayout.loadField(AprilTagField.k2025ReefscapeWelded)
        
        # Define where the camera is on the robot
        robot_to_camera = geom.Transform3d(
            geom.Translation3d(CAMERA_OFFSET_X, CAMERA_OFFSET_Y, CAMERA_HEIGHT),
            geom.Rotation3d(0, CAMERA_PITCH, 0)
        )

        # This object handles turning "Tag ID" into "X, Y coordinates"
        self.photon_estimator = PhotonPoseEstimator(
            self.field_layout,
            PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
            self.camera,
            robot_to_camera
        )

        # 3. Drivetrain Kinematics (Swerve Example)
        # Offset of each module from robot center (Meters)
        self.kinematics = kinematics.SwerveDrive4Kinematics(
            geom.Translation2d(0.3, 0.3),   # Front Left
            geom.Translation2d(0.3, -0.3),  # Front Right
            geom.Translation2d(-0.3, 0.3),  # Back Left
            geom.Translation2d(-0.3, -0.3)  # Back Right
        )
        
        # 4. Main Pose Estimator (The "Brain")
        # Fuses Wheels + Gyro + Vision into one X,Y,Theta Pose
        self.pose_estimator = estimator.SwerveDrivePoseEstimator(
            self.kinematics,
            self.get_gyro_rotation(),
            self.get_module_positions(),
            geom.Pose2d()
        )

        # 5. PathPlanner Configuration
        try:
            config = RobotConfig.fromGUISettings()
            AutoBuilder.configure(
                self.get_estimated_pose,      # Pose supplier
                self.reset_pose,               # Pose resetter
                self.get_chassis_speeds,       # Robot-relative speed supplier
                self.drive_robot_relative,     # Output function
                PPHolonomicDriveController(
                    PIDConstants(5.0, 0.0, 0.0), # Translation PID
                    PIDConstants(5.0, 0.0, 0.0)  # Rotation PID
                ),
                config,
                self.should_flip_path,
                self
            )
            self.auto_chooser = AutoBuilder.buildAutoChooser()
            wpilib.SmartDashboard.putData("Auto Mode", self.auto_chooser)
        except Exception as e:
            wpilib.reportError(f"PathPlanner failed: {e}")

    # --- Sensor Data Methods ---

    def get_gyro_rotation(self) -> geom.Rotation2d:
        """Get rotation from Raspberry Pi."""
        return geom.Rotation2d.fromDegrees(self.gyro_yaw_entry.get())

    def get_module_positions(self):
        """TODO: Replace with actual encoder data from your swerve modules."""
        return (kinematics.SwerveModulePosition(), kinematics.SwerveModulePosition(),
                kinematics.SwerveModulePosition(), kinematics.SwerveModulePosition())

    def get_estimated_pose(self) -> geom.Pose2d:
        return self.pose_estimator.getEstimatedPosition()

    def reset_pose(self, pose: geom.Pose2d):
        self.pose_estimator.resetPosition(self.get_gyro_rotation(), self.get_module_positions(), pose)

    def get_chassis_speeds(self) -> kinematics.ChassisSpeeds:
        # TODO: Calculate from motor encoders
        return kinematics.ChassisSpeeds()

    def drive_robot_relative(self, speeds: kinematics.ChassisSpeeds):
        # TODO: Command your swerve modules using these speeds
        pass

    def should_flip_path(self):
        return wpilib.DriverStation.getAlliance() == wpilib.DriverStation.Alliance.kRed

    # --- Main Loop ---

    def robotPeriodic(self):
        super().robotPeriodic()
        
        # 1. Update Position using Wheels and Sense HAT Gyro
        self.pose_estimator.update(self.get_gyro_rotation(), self.get_module_positions())

        # 2. Update Position using PhotonVision (AprilTags)
        # This finds the robot's X,Y coordinates on the field
        vision_result = self.photon_estimator.update()
        if vision_result:
            estimated_pose = vision_result.estimatedPose.toPose2d()
            self.pose_estimator.addVisionMeasurement(estimated_pose, vision_result.timestampSeconds)

        # 3. Telemetry to Shuffleboard
        pose = self.get_estimated_pose()
        wpilib.SmartDashboard.putNumber("Robot/Field_X", pose.X())
        wpilib.SmartDashboard.putNumber("Robot/Field_Y", pose.Y())
        wpilib.SmartDashboard.putNumber("Robot/Gyro_Heading", pose.rotation().degrees())

    def autonomousInit(self):
        if hasattr(self, "auto_chooser"):
            self.auto_command = self.auto_chooser.getSelected()
            if self.auto_command:
                self.auto_command.schedule()

if __name__ == "__main__":
    wpilib.run(MyRobot)