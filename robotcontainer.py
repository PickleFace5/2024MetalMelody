#
# Copyright (c) FIRST and other WPILib contributors.
# Open Source Software; you can modify and/or share it under the terms of
# the WPILib BSD license file in the root directory of this project.
#

import commands2
import commands2.button
from commands2 import cmd
from commands2.button import Trigger
from commands2.conditionalcommand import ConditionalCommand
from commands2.sysid import SysIdRoutine

from constants import Constants
from generated.tuner_constants import TunerConstants
from subsystems.leds import LedSubsystem, PatternLevel, Zone
from subsystems.leds.patterns import LedFlashPattern
from subsystems.lift import LiftSubsystem
from subsystems.pivot import PivotSubsystem
from subsystems.intake import IntakeSubsystem
from subsystems.superstructure import Superstructure
from robot_state import RobotState

import math
from pathplannerlib.auto import AutoBuilder
from pathplannerlib.path import PathConstraints, PathPlannerPath
from phoenix6 import swerve
from phoenix6.swerve.utility.phoenix_pid_controller import PhoenixPIDController
from wpilib import Color, DriverStation, RobotBase, RobotController, SmartDashboard
from wpimath.geometry import Rotation2d
from wpimath.units import rotationsToRadians


class RobotContainer:
    """
    This class is where the bulk of the robot should be declared. Since Command-based is a
    "declarative" paradigm, very little robot logic should actually be handled in the :class:`.Robot`
    periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
    subsystems, commands, and button mappings) should be declared here.
    """

    def __init__(self) -> None:
        self._max_speed = (
            TunerConstants.speed_at_12_volts
        )  # speed_at_12_volts desired top speed
        self._max_angular_rate = rotationsToRadians(
            1
        )  # 3/4 of a rotation per second max angular velocity

        self._driver_controller = commands2.button.CommandXboxController(0)
        self._function_controller = commands2.button.CommandXboxController(1)

        self.robot_state = RobotState()
        

        self.drivetrain = TunerConstants.create_drivetrain()

        # Setting up bindings for necessary control of the swerve drive platform
        self._drive = (
            swerve.requests.FieldCentric()
            .with_deadband(self._max_speed * 0.1)
            .with_rotational_deadband(
                self._max_angular_rate * 0.1
            )  # Add a 10% deadband
            .with_drive_request_type(
                swerve.SwerveModule.DriveRequestType.OPEN_LOOP_VOLTAGE
            )  # Use open-loop control for drive motors
        )
        self._brake = swerve.requests.SwerveDriveBrake()
        self._point = swerve.requests.PointWheelsAt()
        self._face = (
            swerve.requests.FieldCentricFacingAngle()
            .with_deadband(self._max_speed * 0.1)
            .with_rotational_deadband(
                self._max_angular_rate * 0.1
            )
            .with_drive_request_type(
                swerve.SwerveModule.DriveRequestType.OPEN_LOOP_VOLTAGE
            )
        )
        self._face.heading_controller = PhoenixPIDController(18.749, 0.45773, 0)
        self._face.heading_controller.enableContinuousInput(-math.pi, math.pi)
        
        self.intake = IntakeSubsystem()
        self.leds = LedSubsystem()
        self.lift = LiftSubsystem()
        self.pivot = PivotSubsystem()

        self.superstructure = Superstructure(self.pivot, self.lift)
        self.robot_state.create_mechanism_2d(self.lift, self.pivot)

        # Path follower
        self._auto_chooser = AutoBuilder.buildAutoChooser("Auto Chooser")
        SmartDashboard.putData("Auto Mode", self._auto_chooser)
        
        self.drivetrain.setDefaultCommand(
            self.drivetrain.apply_request(
                lambda: (
                    self._drive.with_velocity_x(
                        -self._driver_controller.getLeftY() * self._max_speed
                    )
                    .with_velocity_y(
                        -self._driver_controller.getLeftX() * self._max_speed
                    )
                    .with_rotational_rate(
                        -self._driver_controller.getRightX() * self._max_angular_rate
                    )
                )
            )
        )

        # Configure the button bindings
        self.configureButtonBindings()

    def configureButtonBindings(self) -> None:
        """
        Use this method to define your button->command mappings. Buttons can be created by
        instantiating a :GenericHID or one of its subclasses (Joystick or XboxController),
        and then passing it to a JoystickButton.
        """

        self._driver_controller.a().whileTrue(self.drivetrain.apply_request(lambda: self._brake))
        self._driver_controller.b().whileTrue(
            self.drivetrain.apply_request(
                lambda: self._point.with_module_direction(
                    Rotation2d(-self._driver_controller.getLeftY(), -self._driver_controller.getLeftX())
                )
            )
        )
        
        # We can't test these until we get a Limelight onto MM and until PathPlanner Beta 5 releases :(
        self._driver_controller.x().whileTrue(
            self.drivetrain.apply_request(
                lambda: self._face.with_velocity_x(
                    -self._driver_controller.getLeftY() * self._max_speed
                )
                .with_velocity_y(
                    -self._driver_controller.getLeftX() * self._max_speed
                )
                .with_target_direction(
                    # Gets the angle to our alliance's speaker
                    (Constants.k_apriltag_layout.getTagPose(
                        4 if (DriverStation.getAlliance() or DriverStation.Alliance.kBlue) == DriverStation.Alliance.kRed else 7).toPose2d().translation() - self.drivetrain.get_state().pose.translation()).angle() + Rotation2d.fromDegrees(180)
                )
            )
        )
        
        self._driver_controller.y().whileTrue(
            AutoBuilder.pathfindThenFollowPath(PathPlannerPath.fromPathFile("ScoreAmp"), PathConstraints(1, 1, 1, 1, unlimited=True))
        )

        # Run SysId routines when holding back/start and X/Y.
        # Note that each routine should be run exactly once in a single 
        # log.
        (self._driver_controller.back() & self._driver_controller.y()).whileTrue(
            self.drivetrain.sys_id_dynamic(SysIdRoutine.Direction.kForward)
        )
        (self._driver_controller.back() & self._driver_controller.x()).whileTrue(
            self.drivetrain.sys_id_dynamic(SysIdRoutine.Direction.kReverse)
        )
        (self._driver_controller.start() & self._driver_controller.y()).whileTrue(
            self.drivetrain.sys_id_quasistatic(SysIdRoutine.Direction.kForward)
        )
        (self._driver_controller.start() & self._driver_controller.x()).whileTrue(
            self.drivetrain.sys_id_quasistatic(SysIdRoutine.Direction.kReverse)
        )

        # reset the field-centric heading on left bumper press
        self._driver_controller.leftBumper().onTrue(
            self.drivetrain.runOnce(lambda: self.drivetrain.seed_field_centric())
        )

        self._function_controller.leftBumper().whileTrue(
            cmd.parallel(
                self.superstructure.set_goal_command(Superstructure.Goal.INTAKE),
                self.intake.set_desired_state(IntakeSubsystem.DesiredState.INTAKE)
            )
        ).onFalse(
            cmd.parallel(
                self.superstructure.set_goal_command(Superstructure.Goal.DEFAULT_LOW),
                self.intake.set_desired_state(IntakeSubsystem.DesiredState.IDLE)
            )
        )
        
        self._function_controller.rightBumper().whileTrue(
            cmd.parallel(
                self.intake.set_desired_state(IntakeSubsystem.DesiredState.EJECT),
                self.leds.show_pattern_command(LedFlashPattern(Color.kRed, 0.1), PatternLevel.INTAKE_STATE).repeatedly()
            )
        ).onFalse(
            self.superstructure.set_goal_command(Superstructure.Goal.DEFAULT_LOW)
        )
        
        self._function_controller.y().onTrue(
            self.superstructure.set_goal_command(Superstructure.Goal.SCORE_HIGH)
        )
        
        (self._function_controller.x() | self._driver_controller.x()).onTrue(
            ConditionalCommand(
                self.superstructure.set_goal_command(Superstructure.Goal.DEFAULT_HIGH),
                self.superstructure.set_goal_command(Superstructure.Goal.DEFAULT_LOW),
                lambda: self.lift.get_current_state() is self.lift.CurrentState.RAISED
            )
        )
        
        self._function_controller.a().onTrue(
            self.superstructure.set_goal_command(Superstructure.Goal.DEFAULT_LOW)
        )
        
        self._function_controller.b().onTrue(
            commands2.cmd.parallel(
                self.superstructure.set_goal_command(Superstructure.Goal.CLIMB),
                commands2.InstantCommand(lambda: self.lift.set_climb_output(-self._function_controller.getLeftTriggerAxis())).repeatedly()
            )
        )
        
        Trigger(lambda: self.intake.has_note()).debounce(0.05).onTrue(
            self.leds.show_pattern_command(LedFlashPattern(Color.kBlue, 0.1), PatternLevel.INTAKE_STATE).repeatedly().withTimeout(1)
        )
        
        Trigger(lambda: self._function_controller.leftBumper() and not RobotBase.isReal()).debounce(0.05).whileTrue(
            self.leds.show_pattern_command(LedFlashPattern(Color.kBlue, 0.1), PatternLevel.INTAKE_STATE).repeatedly()
        )

        self.drivetrain.register_telemetry(
            lambda state: self.robot_state.log_drivetrain_state(state)
        )

    def getAutonomousCommand(self) -> commands2.Command:
        """Use this to pass the autonomous command to the main {@link Robot} class.

        :returns: the command to run in autonomous
        """
        return self._auto_chooser.getSelected()
