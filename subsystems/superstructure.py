from enum import auto, Enum
from pickletools import UP_TO_NEWLINE

from commands2 import Command, InstantCommand

from subsystems import StateSubsystem
from subsystems.intake import IntakeSubsystem
from subsystems.lift import LiftSubsystem
from subsystems.pivot import PivotSubsystem
from subsystems.swerve import SwerveSubsystem

class Superstructure(StateSubsystem):

    class CurrentState(Enum):
        STOPPED = auto()
        HOLD_NOTE = auto()
        NO_NOTE = auto()
        INTAKE_STATE = auto()
        FIXING_NOTE = auto()
        AMP_SCORING_NORMAL = auto()
        EJECTING_NOTE = auto()
        DEFAULT_EXTENDED = auto()
        DEFAULT_RETRACTED = auto()
        CLIMBING = auto()

    class DesiredState(Enum):
        STOPPED = auto()
        REGULAR_STATE = auto()
        INTAKE_NOTE = auto()
        FIX_NOTE = auto()
        AMP_SCORE_NORMAL = auto()
        EJECT_NOTE = auto()
        DEFAULT_EXTENDED = auto()
        DEFAULT_RETRACTED = auto()
        CLIMB = auto()

    def __init__(self, intake: IntakeSubsystem, pivot: PivotSubsystem, swerve: SwerveSubsystem, 
                 lift: LiftSubsystem) -> None:
        super().__init__("Superstructure")

        self.intake = intake
        self.pivot = pivot
        self.swerve = swerve
        self.lift = lift

    def periodic(self):
        super().periodic()

        self._current_state = self._handle_state_transition()
        self._apply_states()

    def _handle_state_transition(self):

        match self._desired_state:
            case self.DesiredState.STOPPED:
                return self.CurrentState.STOPPED
            case self.DesiredState.REGULAR_STATE:
                return self.CurrentState.HOLD_NOTE if self.intake.has_note() else self.CurrentState.NO_NOTE
            case self.DesiredState.INTAKE_NOTE:
                return self.CurrentState.INTAKE_STATE
            case self.DesiredState.FIX_NOTE:
                return self.CurrentState.FIXING_NOTE
            case self.DesiredState.AMP_SCORE_NORMAL:
                return self.CurrentState.AMP_SCORING_NORMAL
            case self.DesiredState.EJECT_NOTE:
                return self.CurrentState.EJECTING_NOTE
            case self.DesiredState.DEFAULT_EXTENDED:
                return self.CurrentState.DEFAULT_EXTENDED
            case self.DesiredState.DEFAULT_RETRACTED:
                return self.CurrentState.DEFAULT_RETRACTED
            case self.DesiredState.CLIMB:
                return self.CurrentState.CLIMBING
            case _:
                return self.CurrentState.STOPPED

    def _apply_states(self) -> None:
        match self._current_state:
            case self.CurrentState.HOLD_NOTE:
                self._hold_piece()
            case self.CurrentState.NO_NOTE:
                self._no_piece()
            case self.CurrentState.STOPPED:
                self._handle_stopped()
            case self.CurrentState.INTAKE_STATE:
                self._handle_intake_piece()
            case self.CurrentState.FIXING_NOTE:
                self._handle_fix_note()
            case self.CurrentState.AMP_SCORING_NORMAL:
                self._handle_amp_score()
            case self.CurrentState.EJECTING_NOTE:
                self._handle_eject_note()
            case self.CurrentState.DEFAULT_EXTENDED:
                self._handle_default_extended()
            case self.CurrentState.DEFAULT_RETRACTED:
                self._handle_default_retracted()
            case self.CurrentState.CLIMBING:
                self._handle_climbing()
                
    def _hold_piece(self) -> None:
        self.intake.set_desired_state(IntakeSubsystem.DesiredState.IDLE)
        self.pivot.set_desired_state(PivotSubsystem.DesiredState.STOW)
        #self.lift.set_desired_state(LiftSubsystem.DesiredState.LOWER)
    
    def _no_piece(self) -> None:
        self.intake.set_desired_state(IntakeSubsystem.DesiredState.IDLE)
        #self.pivot.set_desired_state(PivotSubsystem.DesiredState.STOW)
        #self.lift.set_desired_state(LiftSubsystem.DesiredState.LOWER)

    def _handle_stopped(self) -> None:
        self.intake.set_desired_state(IntakeSubsystem.DesiredState.OFF)
        self.pivot.set_desired_state(PivotSubsystem.DesiredState.STOW)
        self.lift.set_desired_state(LiftSubsystem.DesiredState.IDLE)

    def _handle_intake_piece(self) -> None:
        self.intake.set_desired_state(IntakeSubsystem.DesiredState.INTAKE)
        self.pivot.set_desired_state(PivotSubsystem.DesiredState.INTAKE)
        self.lift.set_desired_state(LiftSubsystem.DesiredState.LOWER)
        
    def _handle_fix_note(self) -> None:
        """Drives the note back into the intake until the 
        beam breaker is activated."""
        self.intake.set_desired_state(IntakeSubsystem.DesiredState.INTAKE)
        
    def _handle_amp_score(self) -> None:
        self.pivot.set_desired_state(PivotSubsystem.DesiredState.SCORE_DOWN)
        self.lift.set_desired_state(LiftSubsystem.DesiredState.RAISE)
        
    def _handle_eject_note(self) -> None:
        self.intake.set_desired_state(IntakeSubsystem.DesiredState.EJECT)
        
    def _handle_default_extended(self) -> None:
        self.pivot.set_desired_state(PivotSubsystem.DesiredState.STOW)
        self.intake.set_desired_state(IntakeSubsystem.DesiredState.IDLE)
        self.lift.set_desired_state(LiftSubsystem.DesiredState.RAISE)
        
    def _handle_default_retracted(self) -> None:
        self.pivot.set_desired_state(PivotSubsystem.DesiredState.STOW)
        self.intake.set_desired_state(IntakeSubsystem.DesiredState.IDLE)
        self.lift.set_desired_state(LiftSubsystem.DesiredState.LOWER)
        
    def _handle_climbing(self) -> None:
        self.pivot.set_desired_state(PivotSubsystem.DesiredState.STOW)
        self.intake.set_desired_state(IntakeSubsystem.DesiredState.IDLE)
        self.lift.set_desired_state(LiftSubsystem.DesiredState.MANUAL_CONTROL)
