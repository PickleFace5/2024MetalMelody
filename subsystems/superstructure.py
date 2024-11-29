from enum import Enum

from subsystems import StateSubsystem
from subsystems.intake import IntakeSubsystem

class Superstructure(StateSubsystem):

    class CurrentState(Enum):
        OFF = 0

    class DesiredState(Enum):
        OFF = 0

    def __init__(self, intake: IntakeSubsystem):
        super().__init__("Superstructure")

        self.intake = intake

    def periodic(self):
        super().periodic()

