"""Vehicle configuration and physical properties."""

from dataclasses import dataclass
from typing import Optional


@dataclass
class VehicleConfig:
    """Vehicle physical properties configuration."""
    
    # Mass properties
    mass: float  # Initial mass (kg)
    
    # Inertia tensor (principal moments)
    Ixx: float  # Roll moment of inertia (kg*m^2)
    Iyy: float  # Pitch moment of inertia (kg*m^2)
    Izz: float  # Yaw moment of inertia (kg*m^2)
    
    # Reference dimensions for aerodynamics
    ref_area: float  # Reference area (m^2)
    ref_length: float  # Reference length for moments (m)
    
    # Products of inertia (optional, default to zero)
    Ixy: float = 0.0  # Product of inertia (kg*m^2)
    Ixz: float = 0.0  # Product of inertia (kg*m^2)
    Iyz: float = 0.0  # Product of inertia (kg*m^2)
    
    def to_c_struct(self):
        """Convert to C API VehicleParams structure."""
        try:
            import ctypes
            from ctypes import Structure, c_double
            
            class VehicleParams(Structure):
                _fields_ = [
                    ("mass", c_double),
                    ("Ixx", c_double),
                    ("Iyy", c_double),
                    ("Izz", c_double),
                    ("Ixy", c_double),
                    ("Ixz", c_double),
                    ("Iyz", c_double),
                    ("ref_area", c_double),
                    ("ref_length", c_double),
                ]
            
            params = VehicleParams()
            params.mass = self.mass
            params.Ixx = self.Ixx
            params.Iyy = self.Iyy
            params.Izz = self.Izz
            params.Ixy = self.Ixy
            params.Ixz = self.Ixz
            params.Iyz = self.Iyz
            params.ref_area = self.ref_area
            params.ref_length = self.ref_length
            return params
        except ImportError:
            # Return dict if ctypes not available
            return {
                "mass": self.mass,
                "Ixx": self.Ixx,
                "Iyy": self.Iyy,
                "Izz": self.Izz,
                "Ixy": self.Ixy,
                "Ixz": self.Ixz,
                "Iyz": self.Iyz,
                "ref_area": self.ref_area,
                "ref_length": self.ref_length,
            }
