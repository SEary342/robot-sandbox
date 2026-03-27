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
import wpilib

import commands2
from commands2 import InstantCommand, RunCommand
from commands2.button import CommandXboxController

from pathplannerlib.auto import AutoBuilder, NamedCommands
from subsystems.drivesubsystem import DriveSubsystem, BadSimPhysics
from subsystems.swervedrivesubsystem import SwerveDriveSubsystem
from subsystems.shootersubsystem import ShooterSubsystem
from subsystems.climbersubsystem import ClimberSubsystem
from commands.aimattarget import AimAtTarget
from commands.launch_sequence import LaunchSequence
from commands.intake import Intake
from commands.eject import Eject
from commands.climb_up import ClimbUp
from commands.climb_down import ClimbDown

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
        self.climber = ClimberSubsystem()

        self.is_tank_drive = False
        self.field_relative = True  # Default for swerve
        self.drive_multiplier = 1.0  # 1.0 for forward, -1.0 for inverted

        # The driver's controller.
        self.driverController = CommandXboxController(constants.kDriverControllerPort)

        # The operator's controller (Port 1).
        self.operatorController = CommandXboxController(1)

        # Put a number box on the dashboard for shooter tuning
        wpilib.SmartDashboard.putNumber("Shooter/TuningRPM", 3000)
        wpilib.SmartDashboard.putBoolean("Drive Inverted", False)

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
                        * self.drive_multiplier
                    ),
                    leftSpeed=lambda: (
                        -self.driverController.getRawAxis(XboxController.Axis.kLeftX)
                        * swerve_constants.DriveConstants.kInvertDirection
                        * self.drive_multiplier
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
            self.robotDrive.setDefaultCommand(
                RunCommand(
                    lambda: (
                        self.robotDrive.tankDrive(
                            -self.driverController.getRawAxis(
                                XboxController.Axis.kLeftY
                            )
                            * self.drive_multiplier,
                            -self.driverController.getRawAxis(
                                XboxController.Axis.kRightY
                            )
                            * self.drive_multiplier,
                            assumeManualInput=True,
                        )
                        if self.is_tank_drive
                        else self.robotDrive.arcadeDrive(
                            -self.driverController.getRawAxis(
                                XboxController.Axis.kLeftY
                            )
                            * self.drive_multiplier,
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

        # Default command for climber is to stop
        self.climber.setDefaultCommand(RunCommand(self.climber.stop, self.climber))

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

    def toggle_drive_direction(self):
        """Inverts the front/back direction of the robot controls."""
        self.drive_multiplier *= -1.0
        wpilib.SmartDashboard.putBoolean("Drive Inverted", self.drive_multiplier < 0)

    def configureButtonBindings(self):
        """
        Use this method to define your button->command mappings.
        """

        # --- Drive Controls (Driver Only) ---
        # 'B' button: Toggles the drive mode between arcade and tank drive.
        self.driverController.b().onTrue(InstantCommand(self.toggle_drive_mode))

        # 'Back' button: Inverts the driving direction
        self.driverController.back().onTrue(InstantCommand(self.toggle_drive_direction))

        # 'X' button: Aim at the speaker target.
        self.driverController.x().whileTrue(
            commands2.DeferredCommand(
                lambda: AimAtTarget(self.robotDrive, constants.targets),
                self.robotDrive,
            )
        )

        # --- Shared Controls (Driver & Operator) ---
        for controller in [self.driverController, self.operatorController]:
            # 'A' button: Eject fuel back out the intake (while held)
            controller.a().whileTrue(Eject(self.shooter))

            # 'Y' button: Manual RPM tuning mode.
            controller.y().whileTrue(
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
            # Right Bumper: Aim and shoot using the automated sequence.
            controller.rightBumper().whileTrue(
                LaunchSequence(self.shooter, self.robotDrive)
            )

            # Left Bumper: Run intake.
            (controller.leftBumper().and_(controller.rightBumper().not_())).whileTrue(
                Intake(self.shooter)
            )

            # POV Up: Climb up the tower (while held)
            controller.povDown().whileTrue(ClimbUp(self.climber))

            # POV Down: Climb down (while held)
            controller.povUp().whileTrue(ClimbDown(self.climber))

        fpvButton = self.driverController.button(XboxController.Button.kStart)
        return fpvButton

    def disablePIDSubsystems(self) -> None:
        """Disables all ProfiledPIDSubsystem and PIDSubsystem instances."""

    def getAutonomousCommand(self) -> commands2.Command:
        """
        :returns: the command to run in autonomous
        """
        if self.chosenAuto is not None:
            return self.chosenAuto.getSelected()

        return commands2.PrintCommand(
            "No autonomous command selected or AutoBuilder failed"
        )

    def configureAutos(self):
        self.chosenAuto = None
        try:
            # Register Named Commands for use in PathPlanner
            NamedCommands.registerCommand("Intake", Intake(self.shooter))
            NamedCommands.registerCommand("Shoot", LaunchSequence(self.shooter, self.robotDrive))
            NamedCommands.registerCommand("Eject", Eject(self.shooter))
            NamedCommands.registerCommand("ClimbUp", ClimbUp(self.climber))
            NamedCommands.registerCommand("ClimbDown", ClimbDown(self.climber))
            NamedCommands.registerCommand("AimAtTarget", AimAtTarget(self.robotDrive, constants.targets))

            self.chosenAuto = AutoBuilder.buildAutoChooser()
            wpilib.SmartDashboard.putData("Chosen Auto", self.chosenAuto)
        except Exception as e:
            wpilib.reportError(f"AutoBuilder failed: {e}")

    def getTestCommand(self) -> typing.Optional[commands2.Command]:
        return None
