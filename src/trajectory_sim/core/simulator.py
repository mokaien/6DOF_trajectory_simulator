"""Main simulator class that orchestrates the C++ solver."""

import numpy as np
from typing import Optional, Callable, List
from trajectory_sim.core.state import State
from trajectory_sim.config.vehicle import VehicleConfig
from trajectory_sim.config.mission import MissionConfig
from trajectory_sim.models.atmosphere import AtmosphereModel
from trajectory_sim.models.thrust import ThrustModel
from trajectory_sim.models.coefficient_models import AeroCoefficientModel
from trajectory_sim.control.controller import Controller
from trajectory_sim.guidance.guidance import Guidance


class Simulator:
    """Main simulator class for 6-DOF trajectory simulation."""
    
    def __init__(
        self,
        vehicle: VehicleConfig,
        mission: MissionConfig,
        atmosphere: AtmosphereModel,
        aero_model: AeroCoefficientModel,
        thrust: Optional[ThrustModel] = None,
        controller: Optional[Controller] = None,
        guidance: Optional[Guidance] = None,
    ):
        """Initialize simulator.
        
        Args:
            vehicle: Vehicle configuration
            mission: Mission configuration
            atmosphere: Atmosphere model
            aero_model: Aerodynamic coefficient model
            thrust: Thrust model (optional)
            controller: Control law (optional)
            guidance: Guidance logic (optional)
        """
        self.vehicle = vehicle
        self.mission = mission
        self.atmosphere = atmosphere
        self.aero_model = aero_model
        self.thrust = thrust
        self.controller = controller
        self.guidance = guidance
        
        # Initialize state
        self.state = State(
            pos_I=np.array(mission.pos_I),
            vel_I=np.array(mission.vel_I),
            quat_BI=mission.get_quaternion(),
            omega_B=np.array(mission.omega_B),
            mass=vehicle.mass,
        )
        
        # History storage
        self.history: List[State] = []
        self.time_history: List[float] = []
        
        # Try to load C++ library
        self._cpp_lib = None
        self._load_cpp_library()
    
    def _load_cpp_library(self):
        """Load C++ simulation library."""
        try:
            import ctypes
            import os
            import platform
            
            # Try to find the library
            lib_name = "trajectory_simulator"
            if platform.system() == "Windows":
                lib_name = f"{lib_name}.dll"
            elif platform.system() == "Darwin":
                lib_name = f"lib{lib_name}.dylib"
            else:
                lib_name = f"lib{lib_name}.so"
            
            # Try common locations
            possible_paths = [
                os.path.join(os.path.dirname(__file__), "..", "..", "..", "build", "lib", lib_name),
                os.path.join(os.path.dirname(__file__), "..", "..", "..", "lib", lib_name),
                lib_name,  # System library path
            ]
            
            for path in possible_paths:
                if os.path.exists(path):
                    self._cpp_lib = ctypes.CDLL(path)
                    self._setup_cpp_functions()
                    return
            
            # If not found, will use Python fallback
            print("Warning: C++ library not found, using Python fallback (slower)")
        except Exception as e:
            print(f"Warning: Could not load C++ library: {e}")
            print("Using Python fallback (slower)")
    
    def _setup_cpp_functions(self):
        """Setup C++ function signatures."""
        if self._cpp_lib is None:
            return
        
        try:
            import ctypes
            from ctypes import Structure, c_double, POINTER
            
            # Define structures
            class CState(Structure):
                _fields_ = [
                    ("pos_I", c_double * 3),
                    ("vel_I", c_double * 3),
                    ("quat_BI", c_double * 4),
                    ("omega_B", c_double * 3),
                    ("mass", c_double),
                ]
            
            class ControlInput(Structure):
                _fields_ = [
                    ("p", c_double),
                    ("q", c_double),
                    ("r", c_double),
                    ("i", c_double),
                ]
            
            class AeroCoeffs(Structure):
                _fields_ = [
                    ("Cx", c_double),
                    ("Cy", c_double),
                    ("Cz", c_double),
                    ("Cmx", c_double),
                    ("Cmy", c_double),
                    ("Cmz", c_double),
                ]
            
            class ThrustData(Structure):
                _fields_ = [
                    ("thrust_B", c_double * 3),
                    ("mass_flow_rate", c_double),
                ]
            
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
            
            class EnvironmentParams(Structure):
                _fields_ = [
                    ("gravity", c_double * 3),
                    ("wind_I", c_double * 3),
                    ("density", c_double),
                    ("speed_of_sound", c_double),
                ]
            
            # Setup function signature
            self._cpp_lib.step_simulation.argtypes = [
                POINTER(CState),
                POINTER(ControlInput),
                POINTER(AeroCoeffs),
                POINTER(ThrustData),
                POINTER(VehicleParams),
                POINTER(EnvironmentParams),
                c_double,  # time
                c_double,  # dt
            ]
            self._cpp_lib.step_simulation.restype = None
            
            # Store structure classes
            self._CState = CState
            self._ControlInput = ControlInput
            self._AeroCoeffs = AeroCoeffs
            self._ThrustData = ThrustData
            self._VehicleParams = VehicleParams
            self._EnvironmentParams = EnvironmentParams
            
        except Exception as e:
            print(f"Warning: Could not setup C++ functions: {e}")
            self._cpp_lib = None
    
    def step(self, dt: Optional[float] = None, callback: Optional[Callable] = None):
        """Step simulation forward one time step.
        
        Args:
            dt: Time step (uses mission.dt if not provided)
            callback: Optional callback function(state, time) called after each step
        """
        if dt is None:
            dt = self.mission.dt
        
        current_time = len(self.time_history) * self.mission.dt + self.mission.t_start
        
        # Get control inputs
        control = self._get_control_inputs(current_time)
        
        # Get aerodynamic coefficients
        aero_coeffs = self._get_aerodynamic_coefficients(current_time)
        
        # Get thrust
        thrust_data = self._get_thrust_data(current_time)
        
        # Get atmosphere
        altitude = -self.state.pos_I[2]  # Assuming NED frame (z down)
        atm_state = self.atmosphere.get_atmosphere(altitude)
        wind_I = self.atmosphere.get_wind(self.state.pos_I, current_time)
        
        # Step using C++ if available, otherwise Python fallback
        if self._cpp_lib is not None:
            self._step_cpp(dt, control, aero_coeffs, thrust_data, atm_state, wind_I)
        else:
            self._step_python(dt, control, aero_coeffs, thrust_data, atm_state, wind_I)
        
        # Store history
        self.history.append(self.state.copy())
        self.time_history.append(current_time)
        
        # Call callback if provided
        if callback:
            callback(self.state, current_time)
    
    def _get_control_inputs(self, time: float):
        """Get control inputs from controller or guidance."""
        if self.controller is not None:
            return self.controller.compute(self.state, time)
        else:
            # Default: no control
            return {"p": 0.0, "q": 0.0, "r": 0.0, "i": 0.0}
    
    def _get_aerodynamic_coefficients(self, time: float):
        """Compute aerodynamic coefficients."""
        # Compute relative velocity and angles
        vel_B = self._inertial_to_body(self.state.vel_I, self.state.quat_BI)
        wind_I = self.atmosphere.get_wind(self.state.pos_I, time)
        wind_B = self._inertial_to_body(wind_I, self.state.quat_BI)
        v_rel_B = vel_B - wind_B
        v_rel_A = self._body_to_aero(v_rel_B)
        
        # Compute aerodynamic angles
        V_mag = np.linalg.norm(v_rel_A)
        if V_mag < 1e-6:
            alpha = beta = 0.0
        else:
            alpha = np.arctan2(-v_rel_A[2], -v_rel_A[0])
            beta = np.arcsin(v_rel_A[1] / V_mag)
        
        # Get Mach number
        altitude = -self.state.pos_I[2]
        atm_state = self.atmosphere.get_atmosphere(altitude)
        mach = V_mag / atm_state["speed_of_sound"] if atm_state["speed_of_sound"] > 1e-6 else 0.0
        
        # Get control inputs
        control = self._get_control_inputs(time)
        
        # Evaluate aerodynamic coefficients
        return self.aero_model.evaluate(
            alpha, beta, mach,
            control["p"], control["q"], control["r"], control["i"]
        )
    
    def _get_thrust_data(self, time: float):
        """Get thrust data."""
        if self.thrust is not None:
            thrust_B = self.thrust.get_thrust(time)
            mass_flow = self.thrust.get_mass_flow_rate(time)
        else:
            thrust_B = np.array([0.0, 0.0, 0.0])
            mass_flow = 0.0
        
        return {"thrust_B": thrust_B, "mass_flow_rate": mass_flow}
    
    def _step_cpp(self, dt, control, aero_coeffs, thrust_data, atm_state, wind_I):
        """Step using C++ library."""
        # This would call the C++ function
        # For now, fall back to Python implementation
        self._step_python(dt, control, aero_coeffs, thrust_data, atm_state, wind_I)
    
    def _step_python(self, dt, control, aero_coeffs, thrust_data, atm_state, wind_I):
        """Step using Python implementation (simplified)."""
        # Simplified Python implementation
        # In production, this would be a full Python reimplementation or call to C++
        # For now, just update position based on velocity
        self.state.pos_I += self.state.vel_I * dt
        
        # Update mass
        self.state.mass += thrust_data["mass_flow_rate"] * dt
        if self.state.mass < 0.1:
            self.state.mass = 0.1
    
    def _inertial_to_body(self, v_I, quat_BI):
        """Transform vector from inertial to body frame."""
        # Simplified quaternion rotation
        w, x, y, z = quat_BI
        v = v_I
        
        # Quaternion rotation: v' = q * v * q^-1
        # For pure quaternion v_quat = [0, v]
        v_quat = np.array([0, v[0], v[1], v[2]])
        q = quat_BI
        q_inv = np.array([q[0], -q[1], -q[2], -q[3]])
        
        # q * v_quat
        result = np.array([
            -q[1]*v[0] - q[2]*v[1] - q[3]*v[2],
            q[0]*v[0] + q[2]*v[2] - q[3]*v[1],
            q[0]*v[1] - q[1]*v[2] + q[3]*v[0],
            q[0]*v[2] + q[1]*v[1] - q[2]*v[0],
        ])
        
        # result * q_inv
        v_B = np.array([
            result[1]*q_inv[1] + result[2]*q_inv[2] + result[3]*q_inv[3],
            -result[0]*q_inv[1] + result[1]*q_inv[0] - result[2]*q_inv[3] + result[3]*q_inv[2],
            -result[0]*q_inv[2] + result[1]*q_inv[3] + result[2]*q_inv[0] - result[3]*q_inv[1],
            -result[0]*q_inv[3] - result[1]*q_inv[2] + result[2]*q_inv[1] + result[3]*q_inv[0],
        ])
        
        return v_B[1:4]  # Return vector part
    
    def _body_to_aero(self, v_B):
        """Transform vector from body to aerodynamic frame."""
        return np.array([-v_B[0], v_B[1], -v_B[2]])
    
    def run(self, callback: Optional[Callable] = None):
        """Run simulation from start to end time.
        
        Args:
            callback: Optional callback function(state, time) called after each step
        """
        self.history = [self.state.copy()]
        self.time_history = [self.mission.t_start]
        
        t = self.mission.t_start
        while t < self.mission.t_end:
            self.step(callback=callback)
            t += self.mission.dt
    
    def get_history(self):
        """Get simulation history."""
        return {
            "states": self.history,
            "times": self.time_history,
        }
