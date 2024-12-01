from phoenix6.configs import TalonFXConfiguration
from phoenix6.configs.config_groups import DifferentialSensorSourceValue, DifferentialSensorsConfigs, NeutralModeValue
from phoenix6.controls import CoastOut, DifferentialFollower, DutyCycleOut, PositionDutyCycle
from phoenix6.hardware import TalonFX
from commands2 import Subsystem
from wpilib import Timer
from constants import Constants
from enum import Enum, auto

from wpimath.system.plant import DCMotor

from subsystems import StateSubsystem

class LiftSubsystem(StateSubsystem):
    
    class CurrentState(Enum):
        IDLE = auto()
        LOWERED = auto()
        RAISED = auto()
        CONTROLLED = auto() # Manual control for climbing
        AIMED = auto() # Position for shooting upwards into the amp
    
    class DesiredState(Enum):
        IDLE = auto()
        LOWER = auto()
        RAISE = auto()
        MANUAL_CONTROL = auto()
        AIM_AMP = auto()
        
    _master_config = TalonFXConfiguration()
    _master_config.slot0 = Constants.LiftConstants.k_gains
    _master_config.motor_output.with_neutral_mode(NeutralModeValue.BRAKE)
    _master_config.current_limits.with_supply_current_limit_enable(True).with_supply_current_limit(Constants.LiftConstants.k_supply_current)
    _master_config.motion_magic.with_motion_magic_cruise_velocity(Constants.LiftConstants.k_cruise_velocity).with_motion_magic_acceleration(Constants.LiftConstants.k_acceleration)
        
    def __init__(self):
        super().__init__("Lift")
        
        self.master_talon = TalonFX(Constants.CanIDs.k_lift_right)
        self.master_talon.configurator.apply(self._master_config)
        self.master_talon.set_position(Constants.LiftConstants.k_top_pos)
        self._add_talon_sim_model(self.master_talon, 
                                  DCMotor.krakenX60FOC(), 
                                  Constants.LiftConstants.k_gear_ratio)
        
        self._follower_talon = TalonFX(Constants.CanIDs.k_lift_left)
        self._follower_talon.configurator.apply(self._master_config)
        
        self._current_state = self.CurrentState.RAISED
        
        self._postion_request = PositionDutyCycle(0)
        self._stop_request = CoastOut()
        self._climb_output = 0
        self._climb_request = DutyCycleOut(self._climb_output)
        
        self._stall_timer = Timer()
        
        self._follower_talon.set_control(self._stop_request)
        
        self._position_pub = self._network_table.getDoubleTopic("Position").publish()
        
        
    def periodic(self):
        super().periodic()
        
        self._current_state = self._handle_state_transition()
        
        match self._current_state:
            case self.CurrentState.IDLE:
                self.master_talon.set_control(self._stop_request)
            case self.CurrentState.LOWERED:
                self._postion_request.position = Constants.LiftConstants.k_bottom_pos
                self.master_talon.set_control(self._postion_request)
            case self.CurrentState.RAISED:
                self._postion_request.position = Constants.LiftConstants.k_top_pos
                self.master_talon.set_control(self._postion_request)
            case self.CurrentState.CONTROLLED:
                self._climb_request.output = self._climb_output
                self.master_talon.set_control(self._climb_request)
                self._follower_talon.set_control(self._climb_request)
                
        # If we're stalling at the bottom, start coasting in order to 
        # prevent overheating.
        if (self.master_talon.get_velocity().value == 0 
                and self.master_talon.get_closed_loop_error().value < 15
                and self._current_state is self.CurrentState.LOWERED):
            self._stall_timer.start()
            
        if self._stall_timer.get() > 0.25:
            self.set_desired_state(self.DesiredState.IDLE)
            self._stall_timer.stop()
            self._stall_timer.reset()
            
        self._position_pub.set(self.master_talon.get_position().value)
        
    def _handle_state_transition(self):
        
        match self._desired_state:
            case self.DesiredState.IDLE:
                return self.CurrentState.IDLE
            case self.DesiredState.LOWER:
                return self.CurrentState.LOWERED
            case self.DesiredState.RAISE:
                return self.CurrentState.RAISED
            case self.DesiredState.MANUAL_CONTROL:
                return self.CurrentState.CONTROLLED
            case self.DesiredState.AIM_AMP:
                return self.CurrentState.AIMED
            case _:
                return self.CurrentState.IDLE
            
    def set_climb_output(self, output: float) -> None:
        self._climb_output = output
