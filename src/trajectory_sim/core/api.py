"""Public API interface for GUI consumption."""

from dataclasses import dataclass
from typing import Optional, Callable, Dict, Any
from trajectory_sim.core.state import State
from trajectory_sim.core.simulator import Simulator


@dataclass
class SimulationConfig:
    """Configuration for a simulation run."""
    
    vehicle: Any  # VehicleConfig
    mission: Any  # MissionConfig
    atmosphere: Any  # AtmosphereModel
    aero_model: Any  # AeroCoefficientModel
    thrust: Optional[Any] = None  # ThrustModel
    controller: Optional[Any] = None  # Controller
    guidance: Optional[Any] = None  # Guidance


class SimulationState:
    """Immutable state snapshot for GUI consumption."""
    
    def __init__(self, state: State, time: float):
        """Create immutable state snapshot."""
        self._pos_I = tuple(state.pos_I)
        self._vel_I = tuple(state.vel_I)
        self._quat_BI = tuple(state.quat_BI)
        self._omega_B = tuple(state.omega_B)
        self._mass = state.mass
        self._time = time
    
    @property
    def pos_I(self):
        """Position in inertial frame."""
        return self._pos_I
    
    @property
    def vel_I(self):
        """Velocity in inertial frame."""
        return self._vel_I
    
    @property
    def quat_BI(self):
        """Quaternion from inertial to body frame."""
        return self._quat_BI
    
    @property
    def omega_B(self):
        """Angular velocity in body frame."""
        return self._omega_B
    
    @property
    def mass(self):
        """Mass."""
        return self._mass
    
    @property
    def time(self):
        """Time."""
        return self._time
    
    def to_dict(self):
        """Convert to dictionary."""
        return {
            "pos_I": self._pos_I,
            "vel_I": self._vel_I,
            "quat_BI": self._quat_BI,
            "omega_B": self._omega_B,
            "mass": self._mass,
            "time": self._time,
        }


class SimulationRunner:
    """High-level interface for running simulations (headless API)."""
    
    def __init__(self, config: SimulationConfig):
        """Initialize simulation runner.
        
        Args:
            config: Simulation configuration
        """
        self.config = config
        self.simulator = Simulator(
            vehicle=config.vehicle,
            mission=config.mission,
            atmosphere=config.atmosphere,
            aero_model=config.aero_model,
            thrust=config.thrust,
            controller=config.controller,
            guidance=config.guidance,
        )
        self._results: Optional[Dict[str, Any]] = None
    
    def run(self, callback: Optional[Callable[[SimulationState, float], None]] = None):
        """Run simulation from start to end time.
        
        Args:
            callback: Optional callback function(state, time) called after each step
        """
        def wrapped_callback(state, time):
            sim_state = SimulationState(state, time)
            if callback:
                callback(sim_state, time)
        
        self.simulator.run(callback=wrapped_callback)
        self._results = self.simulator.get_history()
    
    def get_results(self) -> Dict[str, Any]:
        """Get simulation results.
        
        Returns:
            Dictionary with 'states' and 'times' keys
        """
        if self._results is None:
            self._results = self.simulator.get_history()
        return self._results
    
    def get_final_state(self) -> SimulationState:
        """Get final simulation state.
        
        Returns:
            Final state snapshot
        """
        if self._results is None:
            self.run()
        
        final_state = self._results["states"][-1]
        final_time = self._results["times"][-1]
        return SimulationState(final_state, final_time)
    
    def step(self, callback: Optional[Callable[[SimulationState, float], None]] = None):
        """Step simulation forward one time step.
        
        Args:
            callback: Optional callback function(state, time) called after step
        """
        def wrapped_callback(state, time):
            sim_state = SimulationState(state, time)
            if callback:
                callback(sim_state, time)
        
        self.simulator.step(callback=wrapped_callback)
        self._results = self.simulator.get_history()
