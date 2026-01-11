"""Monte-Carlo batch simulation manager."""

import numpy as np
from typing import List, Dict, Callable, Optional
from trajectory_sim.core.api import SimulationRunner, SimulationConfig, SimulationState


def run_monte_carlo(
    base_config: SimulationConfig,
    n_runs: int,
    randomize_func: Callable[[SimulationConfig], SimulationConfig],
    callback: Optional[Callable[[int, SimulationState, float], None]] = None,
) -> List[Dict]:
    """Run Monte-Carlo batch simulation.
    
    Args:
        base_config: Base simulation configuration
        n_runs: Number of simulation runs
        randomize_func: Function that takes a config and returns a randomized config
        callback: Optional callback function(run_id, state, time) called after each step
    
    Returns:
        List of simulation results (one dict per run)
    """
    results = []
    
    for run_id in range(n_runs):
        # Randomize configuration
        config = randomize_func(base_config)
        
        # Create runner
        runner = SimulationRunner(config)
        
        # Wrap callback to include run_id
        if callback:
            def wrapped_callback(state, time):
                callback(run_id, state, time)
            runner.run(callback=wrapped_callback)
        else:
            runner.run()
        
        # Store results
        run_results = runner.get_results()
        run_results["run_id"] = run_id
        results.append(run_results)
    
    return results
