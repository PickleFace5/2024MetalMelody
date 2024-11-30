from abc import ABC, ABCMeta, abstractmethod
from enum import Enum
from typing import Any

from commands2.subsystem import Subsystem
from ntcore import *

class StateSubsystemMeta(ABCMeta, type(Subsystem)):
    pass


class StateSubsystem(Subsystem, ABC, metaclass=StateSubsystemMeta):


    class CurrentState(Enum):
        OFF = 0

    class DesiredState(Enum):
        OFF = 0

    def __init__(self, name: str, current_state: CurrentState=CurrentState(0), 
                 desired_state: DesiredState=DesiredState(0)):
        super().__init__()
        self.setName(name.title())

        # Create NT folder for organization
        self._network_table = NetworkTableInstance.getDefault().getTable(name.title())

        self._current_state = current_state
        self._desired_state = desired_state

        self._nt_publishers = []

        self._current_state_pub = self._network_table.getStringTopic("Current State").publish()
        self._desired_state_pub = self._network_table.getStringTopic("Desired State").publish()

    def set_desired_state(self, desired_state: DesiredState) -> None: # type: ignore
        """Override this method to handle desired state handling for
        your subsystem!
        """
        self._desired_state = desired_state

    def periodic(self):
        self._current_state_pub.set(self._current_state.name.title().replace("_", " "))
        self._desired_state_pub.set(self._desired_state.name.title().replace("_", " "))

    @abstractmethod
    def _handle_state_transition(self) -> CurrentState:
        pass
