from enum import Enum

from phoenix6.hardware import TalonFX
from phoenix6.configs import TalonFXConfiguration
from phoenix6.configs.config_groups import NeutralModeValue, InvertedValue
from phoenix6.controls import MotionMagicDutyCycle
from phoenix6 import unmanaged
from wpilib import DriverStation, RobotController
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

        self._pivot_talon = TalonFX(Constants.CanIDs.k_pivot_motor)
        self._pivot_talon.configurator.apply(self._pivot_config)
        self._pivot_talon.set_position(0)

        self._pivot_request = MotionMagicDutyCycle(0)
        self._pivot_talon.set_control(self._pivot_request)

        gearbox = DCMotor.falcon500FOC()
        self._sim_model = DCMotorSim(
            LinearSystemId.DCMotorSystem(
                gearbox, 
                0.001,
                Constants.PivotConstants.k_gear_ratio,
            ),
            gearbox
        )

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
            
        self._pivot_talon.set_control(self._pivot_request)
            
        self._position_pub.set(self._pivot_talon.get_position().value)

    def simulationPeriodic(self):

        if DriverStation.isEnabled():
            unmanaged.feed_enable(100)
        
        pivot_sim = self._pivot_talon.sim_state
        pivot_sim.set_supply_voltage(RobotController.getBatteryVoltage())
        self._sim_model.setInputVoltage(pivot_sim.motor_voltage)
        self._sim_model.update(0.02)

        pivot_sim.set_raw_rotor_position(
            units.radiansToRotations(self._sim_model.getAngularPosition())
            * Constants.PivotConstants.k_gear_ratio)
        pivot_sim.set_rotor_velocity(
            units.radiansToRotations(self._sim_model.getAngularVelocity())
            * Constants.PivotConstants.k_gear_ratio
        )
        pivot_sim.set_rotor_acceleration(
            units.radiansToRotations(self._sim_model.getAngularAcceleration())
            * Constants.PivotConstants.k_gear_ratio)

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
        
"""
class PivotStates(Enum):
    STOWED = 0
    SCORE_UP = 1
    SCORE_DOWN = 2
    INTAKE = 3

class Pivot(Subsystem):
    
    def __init__(self) -> None:
        super().__init__()
        self.setName("Pivot")
        
        self.pivotMotor = TalonFX(Constants.CanIDs.k_pivot_motor)
        pivot_config = TalonFXConfiguration()
        pivot_config.motor_output.with_neutral_mode(NeutralModeValue.BRAKE).with_inverted(InvertedValue.CLOCKWISE_POSITIVE)
        pivot_config.slot0 = Constants.PivotConstants.k_gains
        pivot_config.feedback.with_sensor_to_mechanism_ratio(Constants.PivotConstants.k_gear_ratio)
        pivot_config.motion_magic.with_motion_magic_acceleration(Constants.PivotConstants.k_acceleration).with_motion_magic_cruise_velocity(Constants.PivotConstants.k_cruise_velocity).with_motion_magic_jerk(Constants.PivotConstants.k_jerk)
        pivot_config.current_limits.with_supply_current_limit(Constants.PivotConstants.k_supply_current).with_supply_current_limit_enable(True)
        self.pivotMotor.configurator.apply(pivot_config)
        
        self.pivotMotor.set_position(0)

        self.state = PivotStates.STOWED

    def getState(self) -> PivotStates:
        return self.state
        
    def intake(self) -> None:
        self.pivotMotor.set_control(MotionMagicDutyCycle(Constants.PivotConstants.k_intake_pos))
        self.state = PivotStates.INTAKE

    def stow(self) -> None:
        self.pivotMotor.set_control(MotionMagicDutyCycle(Constants.PivotConstants.k_stow_pos))
        self.state = PivotStates.STOWED

    def scoreUpwards(self) -> None:
        self.pivotMotor.set_control(MotionMagicDutyCycle(Constants.PivotConstants.k_score_up_pos))
        self.state = PivotStates.SCORE_UP

    def scoreDownwards(self) -> None:
        self.pivotMotor.set_control(MotionMagicDutyCycle(Constants.PivotConstants.k_score_down_pos))
        self.state = PivotStates.SCORE_DOWN
"""
