"""State vector management."""

import numpy as np
from typing import Optional
from dataclasses import dataclass


@dataclass
class State:
    """6-DOF state vector."""
    
    # Position in inertial frame (m)
    pos_I: np.ndarray  # [x, y, z]
    
    # Velocity in inertial frame (m/s)
    vel_I: np.ndarray  # [vx, vy, vz]
    
    # Quaternion from inertial to body frame [w, x, y, z]
    quat_BI: np.ndarray
    
    # Angular velocity in body frame (rad/s)
    omega_B: np.ndarray  # [p, q, r]
    
    # Mass (kg)
    mass: float
    
    def __post_init__(self):
        """Validate and normalize state."""
        # Ensure arrays are numpy arrays
        self.pos_I = np.asarray(self.pos_I, dtype=np.float64)
        self.vel_I = np.asarray(self.vel_I, dtype=np.float64)
        self.quat_BI = np.asarray(self.quat_BI, dtype=np.float64)
        self.omega_B = np.asarray(self.omega_B, dtype=np.float64)
        
        # Normalize quaternion
        quat_norm = np.linalg.norm(self.quat_BI)
        if quat_norm > 1e-10:
            self.quat_BI = self.quat_BI / quat_norm
        else:
            self.quat_BI = np.array([1.0, 0.0, 0.0, 0.0])
    
    def copy(self):
        """Create a deep copy of the state."""
        return State(
            pos_I=self.pos_I.copy(),
            vel_I=self.vel_I.copy(),
            quat_BI=self.quat_BI.copy(),
            omega_B=self.omega_B.copy(),
            mass=self.mass,
        )
    
    def to_dict(self):
        """Convert state to dictionary."""
        return {
            "pos_I": self.pos_I.tolist(),
            "vel_I": self.vel_I.tolist(),
            "quat_BI": self.quat_BI.tolist(),
            "omega_B": self.omega_B.tolist(),
            "mass": self.mass,
        }
    
    @classmethod
    def from_dict(cls, data):
        """Create state from dictionary."""
        return cls(
            pos_I=np.array(data["pos_I"]),
            vel_I=np.array(data["vel_I"]),
            quat_BI=np.array(data["quat_BI"]),
            omega_B=np.array(data["omega_B"]),
            mass=data["mass"],
        )
    
    def to_c_struct(self):
        """Convert to C API State structure."""
        try:
            import ctypes
            from ctypes import Structure, c_double
            
            class CState(Structure):
                _fields_ = [
                    ("pos_I", c_double * 3),
                    ("vel_I", c_double * 3),
                    ("quat_BI", c_double * 4),
                    ("omega_B", c_double * 3),
                    ("mass", c_double),
                ]
            
            state = CState()
            state.pos_I = (c_double * 3)(*self.pos_I)
            state.vel_I = (c_double * 3)(*self.vel_I)
            state.quat_BI = (c_double * 4)(*self.quat_BI)
            state.omega_B = (c_double * 3)(*self.omega_B)
            state.mass = self.mass
            return state
        except ImportError:
            return self.to_dict()
    
    def from_c_struct(self, c_state):
        """Update state from C API State structure."""
        if hasattr(c_state, 'pos_I'):
            # C structure
            self.pos_I = np.array([c_state.pos_I[i] for i in range(3)])
            self.vel_I = np.array([c_state.vel_I[i] for i in range(3)])
            self.quat_BI = np.array([c_state.quat_BI[i] for i in range(4)])
            self.omega_B = np.array([c_state.omega_B[i] for i in range(3)])
            self.mass = c_state.mass
        else:
            # Dictionary
            self.pos_I = np.array(c_state["pos_I"])
            self.vel_I = np.array(c_state["vel_I"])
            self.quat_BI = np.array(c_state["quat_BI"])
            self.omega_B = np.array(c_state["omega_B"])
            self.mass = c_state["mass"]
        self.__post_init__()
    
    def get_euler_angles(self):
        """Convert quaternion to Euler angles (ZYX convention)."""
        w, x, y, z = self.quat_BI
        
        # Roll (x-axis rotation)
        sinr_cosp = 2 * (w * x + y * z)
        cosr_cosp = 1 - 2 * (x * x + y * y)
        roll = np.arctan2(sinr_cosp, cosr_cosp)
        
        # Pitch (y-axis rotation)
        sinp = 2 * (w * y - z * x)
        if abs(sinp) >= 1:
            pitch = np.copysign(np.pi / 2, sinp)  # Use 90 degrees if out of range
        else:
            pitch = np.arcsin(sinp)
        
        # Yaw (z-axis rotation)
        siny_cosp = 2 * (w * z + x * y)
        cosy_cosp = 1 - 2 * (y * y + z * z)
        yaw = np.arctan2(siny_cosp, cosy_cosp)
        
        return np.array([roll, pitch, yaw])
    
    def get_speed(self):
        """Get speed magnitude."""
        return np.linalg.norm(self.vel_I)
