#
# Copyright (c) FIRST and other WPILib contributors.
# Open Source Software; you can modify and/or share it under the terms of
# the WPILib BSD license file in the root directory of this project.
#

from __future__ import annotations
import typing

from wpilib import XboxController
from wpimath.geometry import Pose2d, Rotation2d
import wpilib

from commands2 import InstantCommand, RunCommand
from commands2.button import CommandXboxController
import commands2

from pathplannerlib.auto import AutoBuilder
from subsystems.drivesubsystem import DriveSubsystem, BadSimPhysics
from subsystems.shootersubsystem import ShooterSubsystem

import constants


class RobotContainer:
    """
    This class is where the bulk of the robot should be declared. Since Command-based is a
    "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
    periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
    subsystems, subsystems, and button mappings) should be declared here.
    """

    def __init__(self, robot):
        # The robot's subsystems
        self.robotDrive = DriveSubsystem()
        self.shooter = ShooterSubsystem()

        # The driver's controller.
        self.driverController = CommandXboxController(constants.kDriverControllerPort)

        # Configure the button bindings
        self.configureButtonBindings()
        self.configureAutos()

        # Configure default subsystems
        # Set the default drive command to split-stick arcade drive
        self.robotDrive.setDefaultCommand(RunCommand(
            lambda: self.robotDrive.arcadeDrive(
                -self.driverController.getRawAxis(XboxController.Axis.kLeftY),
                -self.driverController.getRawAxis(XboxController.Axis.kLeftX),
                assumeManualInput=True
            ),
            self.robotDrive
        ))

        # Default command for shooter is to stop (coast)
        self.shooter.setDefaultCommand(RunCommand(self.shooter.stop, self.shooter))

        if commands2.TimedCommandRobot.isSimulation():
            self.robotDrive.simPhysics = BadSimPhysics(self.robotDrive, robot)

    def configureButtonBindings(self):
        """
        Use this method to define your button->command mappings. Buttons can be created by
        instantiating a GenericHID or one of its subclasses (Joystick or XboxController),
        and then calling passing it to a JoystickButton.
        """

        # example 2: when "POV-up" button pressed, reset robot field position to "facing North"
        self.driverController.povUp().onTrue(
            InstantCommand(
                lambda: self.robotDrive.resetOdometry(Pose2d(1.0, 4.0, Rotation2d.fromDegrees(0))),
                self.robotDrive
            )
        )

        # example 3: when "POV-down" is pressed, reset robot field position to "facing South"
        self.driverController.povDown().onTrue(
            InstantCommand(
                lambda: self.robotDrive.resetOdometry(Pose2d(7.0, 4.0, Rotation2d.fromDegrees(180))),
                self.robotDrive
            )
        )

        # example 4: when Right Bumper is held, spin up shooter based on distance to Tag 7
        # (Tag 7 is Blue Speaker, Tag 4 is Red Speaker)
        self.driverController.rightBumper().whileTrue(
            RunCommand(
                lambda: self.shooter.setSpeedFromDistance(
                    self.robotDrive.getDistanceToTag(7)
                ),
                self.shooter
            )
        )

    def getAutonomousCommand(self) -> commands2.Command:
        """
        :returns: the command to run in autonomous
        """
        command = self.chosenAuto.getSelected()
        return command()

    def configureAutos(self):
        try:
            self.chosenAuto = AutoBuilder.buildAutoChooser()
            wpilib.SmartDashboard.putData("Chosen Auto", self.chosenAuto)
        except Exception as e:
            wpilib.reportError(f"AutoBuilder failed: {e}")

    def getTestCommand(self) -> typing.Optional[commands2.Command]:
        """
        :returns: the command to run in test mode (to exercise all systems)
        """
        return None
