"""Mission configuration and initial conditions."""

from dataclasses import dataclass
from typing import Optional
import numpy as np


@dataclass
class MissionConfig:
    """Mission configuration and initial conditions."""
    
    # Initial position in inertial frame (m)
    pos_I: tuple[float, float, float]  # [x, y, z]
    
    # Initial velocity in inertial frame (m/s)
    vel_I: tuple[float, float, float]  # [vx, vy, vz]
    
    # Initial attitude (quaternion from inertial to body frame)
    # Can specify as quaternion [w, x, y, z] or Euler angles [roll, pitch, yaw] in radians
    quat_BI: Optional[tuple[float, float, float, float]] = None  # [w, x, y, z]
    euler_angles: Optional[tuple[float, float, float]] = None  # [roll, pitch, yaw] in radians
    
    # Initial angular velocity in body frame (rad/s)
    omega_B: tuple[float, float, float] = (0.0, 0.0, 0.0)  # [p, q, r]
    
    # Time settings
    t_start: float = 0.0  # Start time (s)
    t_end: float = 10.0  # End time (s)
    dt: float = 0.01  # Time step (s)
    
    # Gravity vector in inertial frame (m/s^2)
    gravity: tuple[float, float, float] = (0.0, 0.0, -9.81)  # [gx, gy, gz]
    
    def get_quaternion(self):
        """Get quaternion from either quat_BI or euler_angles."""
        if self.quat_BI is not None:
            return np.array(self.quat_BI, dtype=np.float64)
        elif self.euler_angles is not None:
            # Convert Euler angles to quaternion
            roll, pitch, yaw = self.euler_angles
            
            # Quaternion from Euler angles (ZYX convention)
            cy = np.cos(yaw * 0.5)
            sy = np.sin(yaw * 0.5)
            cp = np.cos(pitch * 0.5)
            sp = np.sin(pitch * 0.5)
            cr = np.cos(roll * 0.5)
            sr = np.sin(roll * 0.5)
            
            w = cr * cp * cy + sr * sp * sy
            x = sr * cp * cy - cr * sp * sy
            y = cr * sp * cy + sr * cp * sy
            z = cr * cp * sy - sr * sp * cy
            
            return np.array([w, x, y, z], dtype=np.float64)
        else:
            # Default: identity quaternion (no rotation)
            return np.array([1.0, 0.0, 0.0, 0.0], dtype=np.float64)
    
    def to_c_struct(self):
        """Convert to C API State structure (initial state)."""
        try:
            import ctypes
            from ctypes import Structure, c_double, POINTER
            
            class State(Structure):
                _fields_ = [
                    ("pos_I", c_double * 3),
                    ("vel_I", c_double * 3),
                    ("quat_BI", c_double * 4),
                    ("omega_B", c_double * 3),
                    ("mass", c_double),
                ]
            
            state = State()
            state.pos_I = (c_double * 3)(*self.pos_I)
            state.vel_I = (c_double * 3)(*self.vel_I)
            quat = self.get_quaternion()
            state.quat_BI = (c_double * 4)(*quat)
            state.omega_B = (c_double * 3)(*self.omega_B)
            # Mass will be set from vehicle config
            state.mass = 0.0
            
            return state
        except ImportError:
            # Return dict if ctypes not available
            quat = self.get_quaternion()
            return {
                "pos_I": list(self.pos_I),
                "vel_I": list(self.vel_I),
                "quat_BI": list(quat),
                "omega_B": list(self.omega_B),
                "mass": 0.0,
            }
