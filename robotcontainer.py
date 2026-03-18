#
# Copyright (c) FIRST and other WPILib contributors.
# Open Source Software; you can modify and/or share it under the terms of
# the WPILib BSD license file in the root directory of this project.
#

from __future__ import annotations
import swerve_constants
from commands.holonomicdrive import HolonomicDrive
import typing

from wpilib import XboxController
from wpimath.geometry import Pose2d, Rotation2d
import wpilib

from commands2 import InstantCommand, RunCommand
from commands2.button import CommandXboxController
import commands2

from pathplannerlib.auto import AutoBuilder
from subsystems.drivesubsystem import DriveSubsystem, BadSimPhysics
from subsystems.swervedrivesubsystem import SwerveDriveSubsystem
from subsystems.shootersubsystem import ShooterSubsystem
from commands.aimattarget import AimAtTarget

import constants


class RobotContainer:
    """
    This class is where the bulk of the robot should be declared. Since Command-based is a
    "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
    periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
    subsystems, subsystems, and button mappings) should be declared here.
    """

    def __init__(self, robot):
        # --- 1. Setup Subsystems (The Robot's Body Parts) ---
        # The robot's subsystems
        if constants.kSwerveInstalled:
            self.robotDrive = SwerveDriveSubsystem()
        else:
            self.robotDrive = DriveSubsystem()

        self.shooter = ShooterSubsystem()

        self.is_tank_drive = False
        self.field_relative = True  # Default for swerve

        # The driver's controller.
        self.driverController = CommandXboxController(constants.kDriverControllerPort)

        # Put a number box on the dashboard for shooter tuning
        wpilib.SmartDashboard.putNumber("Shooter/TuningRPM", 3000)

        # --- 2. Configure Controls (Buttons) ---
        # Configure the button bindings
        fpvButton = self.configureButtonBindings()
        self.configureAutos()

        # Configure default subsystems
        if constants.kSwerveInstalled:
            self.robotDrive.setDefaultCommand(
                HolonomicDrive(
                    self.robotDrive,
                    forwardSpeed=lambda: (
                        -self.driverController.getRawAxis(XboxController.Axis.kLeftY)
                        * swerve_constants.DriveConstants.kInvertDirection
                    ),
                    leftSpeed=lambda: (
                        -self.driverController.getRawAxis(XboxController.Axis.kLeftX)
                        * swerve_constants.DriveConstants.kInvertDirection
                    ),
                    rotationSpeed=lambda: (
                        -0.7
                        * self.driverController.getRawAxis(XboxController.Axis.kRightX)
                    ),
                    fieldRelative=lambda: not fpvButton.getAsBoolean(),
                    deadband=constants.OIConstants.kDriveDeadband,
                    rateLimit=True,
                    square=True,
                )
            )
        else:
            # Set the default drive command to split-stick arcade drive
            # This runs whenever no other drive command is happening and can be toggled
            # between arcade and tank drive using the 'B' button.
            self.robotDrive.setDefaultCommand(
                RunCommand(
                    lambda: (
                        self.robotDrive.tankDrive(
                            -self.driverController.getRawAxis(
                                XboxController.Axis.kLeftY
                            ),
                            -self.driverController.getRawAxis(
                                XboxController.Axis.kRightY
                            ),
                            assumeManualInput=True,
                        )
                        if self.is_tank_drive
                        else self.robotDrive.arcadeDrive(
                            -self.driverController.getRawAxis(
                                XboxController.Axis.kLeftY
                            ),
                            -self.driverController.getRawAxis(
                                XboxController.Axis.kLeftX
                            ),
                            assumeManualInput=True,
                        )
                    ),
                    self.robotDrive,
                )
            )

        # Default command for shooter is to stop (coast)
        self.shooter.setDefaultCommand(RunCommand(self.shooter.stop, self.shooter))

        if (
            not constants.kSwerveInstalled
            and commands2.TimedCommandRobot.isSimulation()
        ):
            self.robotDrive.simPhysics = BadSimPhysics(self.robotDrive, robot)

    def toggle_drive_mode(self):
        """Toggles between arcade and tank drive modes (Tank) or field-relative (Swerve)."""
        if constants.kSwerveInstalled:
            self.field_relative = not self.field_relative
            wpilib.SmartDashboard.putBoolean(
                "Field Relative Active", self.field_relative
            )
        else:
            self.is_tank_drive = not self.is_tank_drive
            wpilib.SmartDashboard.putBoolean("Tank Drive Active", self.is_tank_drive)

    def configureButtonBindings(self):
        """
        Use this method to define your button->command mappings. Buttons can be created by
        instantiating a GenericHID or one of its subclasses (Joystick or XboxController),
        and then calling passing it to a JoystickButton.

        STUDENTS: This is where you tell the robot what buttons do what!
        """

        # 'A' button: Toggles the shooter logic between physics and interpolation table.
        self.driverController.a().onTrue(
            InstantCommand(self.shooter.toggleShooterLogic, self.shooter)
        )

        # 'B' button: Toggles the drive mode between arcade and tank drive.
        self.driverController.b().onTrue(InstantCommand(self.toggle_drive_mode))

        # 'X' button: Aim at the speaker target.
        self.driverController.x().whileTrue(
            commands2.DeferredCommand(
                lambda: AimAtTarget(
                    self.robotDrive,
                    constants.blueTargets
                    if wpilib.DriverStation.getAlliance()
                    == wpilib.DriverStation.Alliance.kBlue
                    else constants.redTargets,
                ),
                self.robotDrive,
            )
        )

        # 'Y' button: Manual RPM tuning mode.
        # Allows you to set a target RPM on the SmartDashboard and spin the shooter to it.
        self.driverController.y().whileTrue(
            RunCommand(
                lambda: (
                    self.shooter.setTargetRPM(
                        wpilib.SmartDashboard.getNumber("Shooter/TuningRPM", 0)
                    ),
                    self.shooter.runOuttake(),
                ),
                self.shooter,
            )
        )

        # POV Up: Reset odometry to a known starting position (e.g., Blue Alliance)
        self.driverController.povUp().onTrue(
            InstantCommand(
                lambda: self.robotDrive.resetOdometry(
                    Pose2d(1.0, 4.0, Rotation2d.fromDegrees(0))
                ),
                self.robotDrive,
            )
        )

        # POV Down: Reset odometry to a known starting position (e.g., Red Alliance)
        self.driverController.povDown().onTrue(
            InstantCommand(
                lambda: self.robotDrive.resetOdometry(
                    Pose2d(7.0, 4.0, Rotation2d.fromDegrees(180))
                ),
                self.robotDrive,
            )
        )

        fpvButton = self.driverController.button(XboxController.Button.kStart)
        return fpvButton

        # --- Shooting and Intake Logic ---

        def shoot_sequence():
            """A helper function to contain the logic for shooting."""
            # 1. Determine which speaker tags to use based on alliance
            alliance = wpilib.DriverStation.getAlliance()
            target_tags = (
                constants.blueTargets
                if alliance == wpilib.DriverStation.Alliance.kBlue
                else constants.redTargets
            )
            distance = self.robotDrive.getDistanceToClosestTagInList(target_tags)

            # 2. Set the shooter speed based on the calculated distance
            self.shooter.setSpeedFromDistance(distance)

            # 3. Only run the feeder motor if the flywheel is at the target speed
            if self.shooter.isAtSpeed():
                self.shooter.runOuttake()
            else:
                self.shooter.stopIntake()

        # Right Bumper: Aim and shoot. This has priority over intake.
        self.driverController.rightBumper().whileTrue(
            RunCommand(shoot_sequence, self.shooter)
        )

        # Left Bumper: Run intake, but only if the right bumper (shoot) is not held.
        (
            self.driverController.leftBumper().and_(
                self.driverController.rightBumper().not_()
            )
        ).whileTrue(
            RunCommand(
                lambda: (
                    self.shooter.setTargetRPM(constants.kShooterIntakeRPM),
                    self.shooter.runIntake(),
                ),
                self.shooter,
            )
        )

    def disablePIDSubsystems(self) -> None:
        """Disables all ProfiledPIDSubsystem and PIDSubsystem instances.
        This should be called on robot disable to prevent integral windup."""

    def getAutonomousCommand(self) -> commands2.Command:
        """
        :returns: the command to run in autonomous
        """
        # Check if chosenAuto exists and has a selection
        if self.chosenAuto is not None:
            return self.chosenAuto.getSelected()

        # Fallback: Return a command that does nothing so the robot doesn't crash
        return commands2.PrintCommand(
            "No autonomous command selected or AutoBuilder failed"
        )

    def configureAutos(self):
        # Initialize the attribute to None first
        self.chosenAuto = None

        try:
            self.chosenAuto = AutoBuilder.buildAutoChooser()
            if self.chosenAuto is not None:
                wpilib.SmartDashboard.putData("Chosen Auto", self.chosenAuto)
        except Exception as e:
            wpilib.reportError(f"AutoBuilder failed: {e}")

    def getTestCommand(self) -> typing.Optional[commands2.Command]:
        """
        :returns: the command to run in test mode (to exercise all systems)
        """
        return None
