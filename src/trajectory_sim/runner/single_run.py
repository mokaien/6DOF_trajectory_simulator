"""Single simulation execution."""

from typing import Optional, Callable
from trajectory_sim.core.simulator import Simulator
from trajectory_sim.core.api import SimulationRunner, SimulationConfig, SimulationState


def run_simulation(
    config: SimulationConfig,
    callback: Optional[Callable[[SimulationState, float], None]] = None,
) -> dict:
    """Run a single simulation.
    
    Args:
        config: Simulation configuration
        callback: Optional callback function(state, time) called after each step
    
    Returns:
        Dictionary with simulation results (states and times)
    """
    runner = SimulationRunner(config)
    runner.run(callback=callback)
    return runner.get_results()
