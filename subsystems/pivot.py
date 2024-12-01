from enum import Enum

from phoenix6.hardware import TalonFX
from phoenix6.configs import TalonFXConfiguration
from phoenix6.configs.config_groups import NeutralModeValue, InvertedValue
from phoenix6.controls import MotionMagicDutyCycle
from phoenix6 import unmanaged
from wpilib import DriverStation, Mechanism2d, RobotController, SmartDashboard
from wpilib.simulation import DCMotorSim
from wpimath import units
from wpimath.system.plant import DCMotor, LinearSystemId

from constants import Constants
from subsystems import StateSubsystem


class PivotSubsystem(StateSubsystem):

    class CurrentState(Enum):
        STOWED = 0
        SCORE_UP = 1
        SCORE_DOWN = 2
        INTAKE = 3

    class DesiredState(Enum):
        STOW = 0
        SCORE_UP = 1
        SCORE_DOWN = 2
        INTAKE = 3

    _pivot_config = TalonFXConfiguration()
    _pivot_config.motor_output.with_neutral_mode(NeutralModeValue.BRAKE).with_inverted(InvertedValue.CLOCKWISE_POSITIVE)
    _pivot_config.slot0 = Constants.PivotConstants.k_gains
    _pivot_config.feedback.with_sensor_to_mechanism_ratio(Constants.PivotConstants.k_gear_ratio)
    _pivot_config.motion_magic.with_motion_magic_acceleration(Constants.PivotConstants.k_acceleration).with_motion_magic_cruise_velocity(Constants.PivotConstants.k_cruise_velocity).with_motion_magic_jerk(Constants.PivotConstants.k_jerk)


    def __init__(self):
        super().__init__("Pivot")

        self.pivot_talon = TalonFX(Constants.CanIDs.k_pivot_motor)
        self.pivot_talon.configurator.apply(self._pivot_config)
        self.pivot_talon.set_position(0)

        self._pivot_request = MotionMagicDutyCycle(0)
        self.pivot_talon.set_control(self._pivot_request)

        self._add_talon_sim_model(self.pivot_talon, 
                                  DCMotor.falcon500FOC(), 
                                  Constants.PivotConstants.k_gear_ratio)

        self._position_pub = self._network_table.getDoubleTopic("Position").publish()

    def periodic(self):
        super().periodic()

        self._current_state = self._handle_state_transition()

        match self._current_state:

            case self.CurrentState.STOWED:
                self._pivot_request.position = Constants.PivotConstants.k_stow_pos
            case self.CurrentState.SCORE_UP:
                self._pivot_request.position = Constants.PivotConstants.k_score_up_pos
            case self.CurrentState.SCORE_DOWN:
                self._pivot_request.position = Constants.PivotConstants.k_score_down_pos
            case self.CurrentState.INTAKE:
                self._pivot_request.position = Constants.PivotConstants.k_intake_pos
            case _:
                raise ValueError(f'Invalid state for pivot subsystem {self._current_state}')
            
        self.pivot_talon.set_control(self._pivot_request)
            
        self._position_pub.set(self.pivot_talon.get_position().value)

    def _handle_state_transition(self) -> CurrentState:
        
        match self._desired_state:

            case self.DesiredState.STOW:
                return self.CurrentState.STOWED
            case self.DesiredState.SCORE_UP:
                return self.CurrentState.SCORE_UP
            case self.DesiredState.SCORE_DOWN:
                return self.CurrentState.SCORE_DOWN
            case self.DesiredState.INTAKE:
                return self.CurrentState.INTAKE
            case _:
                return self.CurrentState.STOWED
        