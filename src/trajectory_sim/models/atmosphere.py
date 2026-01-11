"""Atmosphere and wind models."""

from abc import ABC, abstractmethod
import numpy as np
from typing import Tuple


class AtmosphereModel(ABC):
    """Abstract base class for atmosphere models."""
    
    @abstractmethod
    def get_atmosphere(self, altitude: float) -> dict:
        """Get atmosphere properties at given altitude.
        
        Args:
            altitude: Altitude above sea level (m)
        
        Returns:
            Dictionary with keys: density, pressure, temperature, speed_of_sound
        """
        pass
    
    @abstractmethod
    def get_wind(self, pos_I: np.ndarray, time: float) -> np.ndarray:
        """Get wind velocity in inertial frame.
        
        Args:
            pos_I: Position in inertial frame
            time: Current time
        
        Returns:
            Wind velocity vector in inertial frame
        """
        pass


class WindModel(ABC):
    """Abstract base class for wind models."""
    
    @abstractmethod
    def get_wind(self, pos_I: np.ndarray, time: float) -> np.ndarray:
        """Get wind velocity in inertial frame."""
        pass


class ExponentialAtmosphere(AtmosphereModel):
    """Simple exponential atmosphere model (ISA-like)."""
    
    def __init__(
        self,
        rho0: float = 1.225,
        H: float = 8400.0,
        T0: float = 288.15,
        P0: float = 101325.0,
        gamma: float = 1.4,
        R: float = 287.0,
        wind_model: WindModel = None,
    ):
        """Initialize exponential atmosphere model.
        
        Args:
            rho0: Sea level density (kg/m^3)
            H: Scale height (m)
            T0: Sea level temperature (K)
            P0: Sea level pressure (Pa)
            gamma: Specific heat ratio
            R: Gas constant (J/(kg*K))
            wind_model: Wind model (optional)
        """
        self.rho0 = rho0
        self.H = H
        self.T0 = T0
        self.P0 = P0
        self.gamma = gamma
        self.R = R
        self.wind_model = wind_model if wind_model is not None else NoWind()
    
    def get_atmosphere(self, altitude: float) -> dict:
        """Get atmosphere properties."""
        # Exponential density
        density = self.rho0 * np.exp(-altitude / self.H)
        
        # Temperature (linear lapse rate)
        lapse_rate = 0.0065  # K/m
        temperature = self.T0 - lapse_rate * altitude
        if temperature < 216.65:
            temperature = 216.65  # Tropopause limit
        
        # Pressure from ideal gas law
        pressure = density * self.R * temperature
        
        # Speed of sound
        speed_of_sound = np.sqrt(self.gamma * self.R * temperature)
        
        return {
            "density": density,
            "pressure": pressure,
            "temperature": temperature,
            "speed_of_sound": speed_of_sound,
        }
    
    def get_wind(self, pos_I: np.ndarray, time: float) -> np.ndarray:
        """Get wind velocity."""
        return self.wind_model.get_wind(pos_I, time)


class ConstantWind(WindModel):
    """Constant wind model."""
    
    def __init__(self, wind_velocity: Tuple[float, float, float]):
        """Initialize constant wind.
        
        Args:
            wind_velocity: Wind velocity vector in inertial frame [wx, wy, wz]
        """
        self.wind_velocity = np.array(wind_velocity)
    
    def get_wind(self, pos_I: np.ndarray, time: float) -> np.ndarray:
        """Get wind velocity."""
        return self.wind_velocity


class NoWind(WindModel):
    """No wind model."""
    
    def get_wind(self, pos_I: np.ndarray, time: float) -> np.ndarray:
        """Get wind velocity (zero)."""
        return np.array([0.0, 0.0, 0.0])
