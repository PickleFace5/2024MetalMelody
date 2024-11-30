from enum import Enum

from commands2 import Command, InstantCommand

from subsystems import StateSubsystem
from subsystems.intake import IntakeSubsystem
from subsystems.pivot import PivotSubsystem

class Superstructure(StateSubsystem):

    class CurrentState(Enum):
        STOPPED = 0
        INTAKE_PIECE = 1

    class DesiredState(Enum):
        STOPPED = 0
        INTAKE_PIECE = 1

    def __init__(self, intake: IntakeSubsystem, pivot: PivotSubsystem) -> None:
        super().__init__("Superstructure")

        self.intake = intake
        self.pivot = pivot

    def periodic(self):
        super().periodic()

        self._current_state = self._handle_state_transition()
        self._apply_states()

    def _handle_state_transition(self):

        match self._desired_state:

            case self.DesiredState.STOPPED:
                return self.CurrentState.STOPPED

            case self.DesiredState.INTAKE_PIECE:
                return self.CurrentState.INTAKE_PIECE

            case _:
                return self.CurrentState.STOPPED

    def _apply_states(self) -> None:
        match self._current_state:
            case self.CurrentState.STOPPED:
                self._handle_stopped()
            case self.CurrentState.INTAKE_PIECE:
                self._handle_intake_piece()

    def _handle_stopped(self) -> None:
        self.intake.set_desired_state(IntakeSubsystem.DesiredState.OFF)
        self.pivot.set_desired_state(PivotSubsystem.DesiredState.STOW)

    def _handle_intake_piece(self) -> None:
        self.intake.set_desired_state(IntakeSubsystem.DesiredState.INTAKE)
        self.pivot.set_desired_state(PivotSubsystem.DesiredState.INTAKE)

    def set_desired_state_command(self, state: DesiredState) -> Command:
        return InstantCommand(lambda: self.set_desired_state(state))
