from enum import auto, Enum

from commands2 import Command, InstantCommand, Subsystem, cmd
from wpilib import DriverStation, SmartDashboard

from subsystems import StateSubsystem
from subsystems.intake import IntakeSubsystem
from subsystems.lift import LiftSubsystem
from subsystems.pivot import PivotSubsystem
from subsystems.swerve import SwerveSubsystem

class Superstructure(Subsystem):
        
        
    """States needed:
    1. Default High
    2. Default Low
    3. Intake
    4. Score High
    5. Score Low
    6. Climb
    The IntakeSubsystem is going to be an external subsystem in this
    case, since it would be easier to make it seperate than add in like
     6 extra states.
    """
        
    class Goal(Enum):
        DEFAULT_HIGH = auto()
        DEFAULT_LOW = auto()
        INTAKE = auto()
        SCORE_HIGH = auto()
        SCORE_LOW = auto()
        CLIMB = auto()

    def __init__(self, pivot: PivotSubsystem, lift: LiftSubsystem) -> None:

        self.pivot = pivot
        self.lift = lift
        
        self._current_goal = self.Goal.DEFAULT_LOW
        self._desired_goal = self.Goal.DEFAULT_LOW
        self._last_goal = self.Goal.DEFAULT_LOW

    def periodic(self):
        if DriverStation.isDisabled():
            default_command = self.set_goal_command(self.Goal.DEFAULT_LOW)
            default_command.addRequirements(self)
            self.setDefaultCommand(default_command)
        
        self._current_goal = self._desired_goal
        self._last_goal = self._current_goal
        SmartDashboard.putString("Superstructure Current Goal", self._current_goal.name)
        
        match self._current_goal:
            case self.Goal.DEFAULT_HIGH:
                self.lift.set_desired_state(LiftSubsystem.DesiredState.RAISE)
                self.pivot.set_desired_state(PivotSubsystem.DesiredState.STOW)
            case self.Goal.DEFAULT_LOW:
                self.lift.set_desired_state(LiftSubsystem.DesiredState.LOWER)
                self.pivot.set_desired_state(PivotSubsystem.DesiredState.STOW)
            case self.Goal.INTAKE:
                self.lift.set_desired_state(LiftSubsystem.DesiredState.LOWER)
                self.pivot.set_desired_state(PivotSubsystem.DesiredState.INTAKE)
            case self.Goal.SCORE_HIGH:
                self.lift.set_desired_state(LiftSubsystem.DesiredState.RAISE)
                self.pivot.set_desired_state(PivotSubsystem.DesiredState.SCORE_DOWN)
            case self.Goal.SCORE_LOW:
                self.lift.set_desired_state(LiftSubsystem.DesiredState.AIM_AMP)
                self.pivot.set_desired_state(PivotSubsystem.DesiredState.SCORE_UP)
            case self.Goal.CLIMB:
                self.lift.set_desired_state(LiftSubsystem.DesiredState.MANUAL_CONTROL)
                self.pivot.set_desired_state(PivotSubsystem.DesiredState.STOW)
        
    def _set_goal(self, goal: Goal) -> None:
        if goal is self._desired_goal:
            return
        self._desired_goal = goal
        
    def set_goal_command(self, goal: Goal) -> Command:
        return cmd.startEnd(
            lambda: self._set_goal(goal),
            lambda: self._set_goal(self.Goal.DEFAULT_LOW),
            self
        )
