from enum import Enum

from phoenix6.configs import TalonFXConfiguration
from phoenix6.configs.config_groups import CurrentLimitsConfigs, Slot0Configs
from phoenix6.hardware import TalonFX
from phoenix6.controls import VoltageOut
from phoenix6.signals import ForwardLimitValue
from wpimath.system.plant import DCMotor

from constants import Constants
from subsystems import StateSubsystem


class IntakeSubsystem(StateSubsystem):


    IDLING_VOLTAGE = 0.0
    INTAKE_VOLTAGE = 8.0
    EJECT_VOLTAGE = -8.0

    _talon_config = TalonFXConfiguration()
    _talon_config.current_limits = (CurrentLimitsConfigs()
     .with_supply_current_limit_enable(True)
     .with_supply_current_limit(20))
    _talon_config.feedback.sensor_to_mechanism_ratio = Constants.IntakeConstants.k_gear_ratio
    _talon_config.slot0 = Slot0Configs().with_k_p(1)


    class CurrentState(Enum):
        OFF = 0
        IDLING = 1
        INTAKING = 2
        EJECTING = 3


    class DesiredState(Enum):
        OFF = 0
        IDLE = 1
        INTAKE = 2
        EJECT = 3

    def __init__(self):
        super().__init__("Intake")

        self._intake_talon = TalonFX(Constants.CanIDs.k_intake_motor)
        self._intake_talon.configurator.apply(self._talon_config)

        self._intake_request = VoltageOut(0)
        self._intake_talon.set_control(self._intake_request)
        self._add_talon_sim_model(self._intake_talon, 
                           DCMotor.falcon500FOC(), 
                           Constants.IntakeConstants.k_gear_ratio)

        ## Logging
        self._intake_voltage = self._network_table.getDoubleTopic("Output Voltage").publish()
        self._intake_rps = self._network_table.getDoubleTopic("Velocity RPS").publish()

    def periodic(self):
        super().periodic()

        self._current_state = self._handle_state_transition()

        match self._current_state:
            case self.CurrentState.OFF:
                self._intake_request.output = 0
            case self.CurrentState.IDLING:
                self._intake_request.output = self.IDLING_VOLTAGE
            case self.CurrentState.INTAKING:
                self._intake_request.output = self.INTAKE_VOLTAGE
            case self.CurrentState.EJECTING:
                self._intake_request.output = self.EJECT_VOLTAGE
            case _:
                self._intake_request.output = 0

        self._intake_talon.set_control(self._intake_request)

        self._intake_voltage.set(self._intake_talon.get_motor_voltage().value)
        self._intake_rps.set(self._intake_talon.get_velocity().value)
        
        if self.has_note:
            self.set_desired_state(self.DesiredState.IDLE)
        

    def _handle_state_transition(self) -> CurrentState:
        match self._desired_state:
            case self.DesiredState.OFF:
                return self.CurrentState.OFF
            case self.DesiredState.IDLE:
                return self.CurrentState.IDLING
            case self.DesiredState.INTAKE:
                return self.CurrentState.INTAKING
            case self.DesiredState.EJECT:
                return self.CurrentState.EJECTING
            case _:
                return self.CurrentState.OFF
            
    def has_note(self) -> bool:
        return self._intake_talon.get_forward_limit().value is ForwardLimitValue.CLOSED_TO_GROUND
