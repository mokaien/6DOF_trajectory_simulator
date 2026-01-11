"""Control law implementations."""

from abc import ABC, abstractmethod
from typing import Dict
from trajectory_sim.core.state import State


class Controller(ABC):
    """Abstract base class for controllers."""
    
    @abstractmethod
    def compute(self, state: State, time: float) -> Dict[str, float]:
        """Compute control inputs.
        
        Args:
            state: Current state
            time: Current time
        
        Returns:
            Dictionary with keys: p, q, r, i (control inputs)
        """
        pass


class NullController(Controller):
    """Null controller (no control inputs)."""
    
    def compute(self, state: State, time: float) -> Dict[str, float]:
        """Return zero control inputs."""
        return {"p": 0.0, "q": 0.0, "r": 0.0, "i": 0.0}
