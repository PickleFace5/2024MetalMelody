import math
from ntcore import NetworkTableInstance
from phoenix6 import SignalLogger, swerve, units
from wpilib import DataLogManager, DriverStation, Field2d, Mechanism2d, SmartDashboard
from wpimath.geometry import Pose2d
from wpimath.kinematics import ChassisSpeeds, SwerveModuleState
from wpimath import units

from constants import Constants
from subsystems.lift import LiftSubsystem
from subsystems.pivot import PivotSubsystem

class RobotState:
    def __init__(self):
        DriverStation.startDataLog(DataLogManager.getLog())

        self._field = Field2d()
        SmartDashboard.putData("Field", self._field)

        # Robot speeds for general checking
        self._table = NetworkTableInstance.getDefault().getTable("Telemetry")
        self._current_pose = self._table.getStructTopic("currentPose", Pose2d).publish()
        self._chassis_speeds = self._table.getStructTopic("chassisSpeeds", ChassisSpeeds).publish()
        self._odom_freq = self._table.getDoubleTopic("Odometry Frequency").publish()

        # Additional swerve info
        self._module_states = self._table.getStructArrayTopic("moduleStates", SwerveModuleState).publish()
        self._module_targets = self._table.getStructArrayTopic("moduleTargets", SwerveModuleState).publish()

    def log_drivetrain_state(self, state: swerve.SwerveDrivetrain.SwerveDriveState):
        """
        Accept the swerve drive state and log it to NetworkTables.
        """

        self._field.setRobotPose(state.pose)
        self._current_pose.set(state.pose)

        self._odom_freq.set(1.0 / state.odometry_period)

        self._module_states.set(state.module_states)
        self._module_targets.set(state.module_targets)
        self._chassis_speeds.set(state.speeds)
    
    def create_mechanism_2d(self, lift: LiftSubsystem, pivot: PivotSubsystem):
        self.mech = Mechanism2d(0.61, 0.66)
        self.root = self.mech.getRoot("le_mechanism", 0.61/2, 0)
        
        self.lift = self.root.appendLigament(
            "lift", 0, 90
        )
        self.pivot = self.lift.appendLigament(
            "pivot", 0.18, 90
        )
        SmartDashboard.putData("The Mechanism", self.mech)
        
        self.lift_subsystem = lift
        self.pivot_subsystem = pivot
        
    def update_mechanism_2d(self) -> None:
        self.lift.setLength(self.lift_subsystem.master_talon.get_position().value / (2 * math.pi) / Constants.LiftConstants.k_gear_ratio / 3 + 0.15)
        self.pivot.setAngle(self.pivot_subsystem.pivot_talon.get_position().value * 360)
