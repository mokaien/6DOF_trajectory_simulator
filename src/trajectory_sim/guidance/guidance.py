"""Guidance logic for reference trajectory generation."""

from abc import ABC, abstractmethod
import numpy as np
from typing import Dict, Optional
from trajectory_sim.core.state import State


class Guidance(ABC):
    """Abstract base class for guidance systems."""
    
    @abstractmethod
    def get_reference(self, state: State, time: float) -> Dict[str, np.ndarray]:
        """Get reference trajectory.
        
        Args:
            state: Current state
            time: Current time
        
        Returns:
            Dictionary with reference values (e.g., position, velocity, attitude)
        """
        pass


class NullGuidance(Guidance):
    """Null guidance (no reference trajectory)."""
    
    def get_reference(self, state: State, time: float) -> Dict[str, np.ndarray]:
        """Return empty reference."""
        return {}
