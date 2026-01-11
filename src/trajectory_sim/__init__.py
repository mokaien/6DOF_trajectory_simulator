"""
6-DOF Atmospheric Trajectory Simulator

A modular trajectory simulation framework for free-fall objects, rockets, and aircraft.
"""

__version__ = "1.0.0"

from trajectory_sim.core.api import SimulationRunner, SimulationConfig, SimulationState

__all__ = [
    "SimulationRunner",
    "SimulationConfig",
    "SimulationState",
]
