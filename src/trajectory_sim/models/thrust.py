"""Thrust models."""

from abc import ABC, abstractmethod
import numpy as np
from typing import Optional, List


class ThrustModel(ABC):
    """Abstract base class for thrust models."""
    
    @abstractmethod
    def get_thrust(self, time: float) -> np.ndarray:
        """Get thrust force in body frame.
        
        Args:
            time: Current time
        
        Returns:
            Thrust vector in body frame
        """
        pass
    
    @abstractmethod
    def get_mass_flow_rate(self, time: float) -> float:
        """Get mass flow rate.
        
        Args:
            time: Current time
        
        Returns:
            Mass flow rate (kg/s, negative for mass loss)
        """
        pass
    
    @abstractmethod
    def is_active(self, time: float) -> bool:
        """Check if thrust is active.
        
        Args:
            time: Current time
        
        Returns:
            True if thrust is active
        """
        pass


class NoThrust(ThrustModel):
    """No thrust model."""
    
    def get_thrust(self, time: float) -> np.ndarray:
        """Get thrust (zero)."""
        return np.array([0.0, 0.0, 0.0])
    
    def get_mass_flow_rate(self, time: float) -> float:
        """Get mass flow rate (zero)."""
        return 0.0
    
    def is_active(self, time: float) -> bool:
        """Thrust is not active."""
        return False


class ConstantThrust(ThrustModel):
    """Constant thrust model."""
    
    def __init__(
        self,
        thrust_vector: tuple[float, float, float],
        mass_flow: float = 0.0,
    ):
        """Initialize constant thrust.
        
        Args:
            thrust_vector: Thrust vector in body frame [Tx, Ty, Tz]
            mass_flow: Mass flow rate (kg/s, negative for mass loss)
        """
        self.thrust_vector = np.array(thrust_vector)
        self.mass_flow = mass_flow
    
    def get_thrust(self, time: float) -> np.ndarray:
        """Get thrust."""
        return self.thrust_vector
    
    def get_mass_flow_rate(self, time: float) -> float:
        """Get mass flow rate."""
        return self.mass_flow
    
    def is_active(self, time: float) -> bool:
        """Thrust is always active."""
        return True


class CSVThrust(ThrustModel):
    """Time-varying thrust from CSV data (interpolated)."""
    
    def __init__(
        self,
        times: List[float],
        thrusts: List[float],
        mass_flows: List[float],
        direction: tuple[float, float, float] = (1.0, 0.0, 0.0),
        duration: Optional[float] = None,
    ):
        """Initialize CSV thrust model.
        
        Args:
            times: Time points (s)
            thrusts: Thrust magnitudes at each time point (N)
            mass_flows: Mass flow rates at each time point (kg/s)
            direction: Thrust direction in body frame (normalized)
            duration: Burn duration (s, uses max time if not provided)
        """
        if len(times) != len(thrusts) or len(times) != len(mass_flows):
            raise ValueError("Times, thrusts, and mass_flows must have same length")
        
        self.times = np.array(times)
        self.thrusts = np.array(thrusts)
        self.mass_flows = np.array(mass_flows)
        self.direction = np.array(direction)
        self.direction = self.direction / np.linalg.norm(self.direction)
        self.duration = duration if duration is not None else max(times)
    
    def _interpolate(self, time: float, values: np.ndarray) -> float:
        """Linear interpolation."""
        if time <= self.times[0]:
            return values[0]
        if time >= self.times[-1]:
            return values[-1]
        
        idx = np.searchsorted(self.times, time)
        if idx == 0:
            return values[0]
        
        t0 = self.times[idx - 1]
        t1 = self.times[idx]
        v0 = values[idx - 1]
        v1 = values[idx]
        
        return v0 + (v1 - v0) * (time - t0) / (t1 - t0)
    
    def get_thrust(self, time: float) -> np.ndarray:
        """Get thrust."""
        if time < 0 or time > self.duration:
            return np.array([0.0, 0.0, 0.0])
        
        magnitude = self._interpolate(time, self.thrusts)
        return self.direction * magnitude
    
    def get_mass_flow_rate(self, time: float) -> float:
        """Get mass flow rate."""
        if time < 0 or time > self.duration:
            return 0.0
        
        return -self._interpolate(time, self.mass_flows)  # Negative for mass loss
    
    def is_active(self, time: float) -> bool:
        """Check if thrust is active."""
        return 0 <= time <= self.duration
