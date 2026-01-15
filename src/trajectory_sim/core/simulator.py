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
            
            # Get project root directory (assuming we're in src/trajectory_sim/core/)
            current_file = os.path.abspath(__file__)
            project_root = os.path.dirname(os.path.dirname(os.path.dirname(current_file)))
            
            # Try common locations
            possible_paths = [
                # Windows Visual Studio build locations (most common)
                os.path.join(project_root, "build", "bin", "Debug", lib_name),
                os.path.join(project_root, "build", "bin", "Release", lib_name),
                os.path.join(project_root, "build", "bin", lib_name),
                # Standard build locations
                os.path.join(project_root, "build", "lib", lib_name),
                os.path.join(project_root, "build", "Debug", lib_name),
                os.path.join(project_root, "build", "Release", lib_name),
                os.path.join(project_root, "build", lib_name),
                # Alternative locations
                os.path.join(project_root, "lib", lib_name),
                # Relative to current working directory
                os.path.join(os.getcwd(), "build", "bin", "Debug", lib_name),
                os.path.join(os.getcwd(), "build", "bin", "Release", lib_name),
                os.path.join(os.getcwd(), "build", "bin", lib_name),
                os.path.join(os.getcwd(), "build", "lib", lib_name),
                os.path.join(os.getcwd(), "build", "Debug", lib_name),
                os.path.join(os.getcwd(), "build", "Release", lib_name),
                os.path.join(os.getcwd(), "lib", lib_name),
                # System library path
                lib_name,
            ]
            
            for path in possible_paths:
                abs_path = os.path.abspath(path)
                if os.path.exists(abs_path):
                    try:
                        self._cpp_lib = ctypes.CDLL(abs_path)
                        self._setup_cpp_functions()
                        print(f"Loaded C++ library from: {abs_path}")
                        return
                    except Exception as e:
                        # Try next path if this one fails
                        continue
            
            # If not found, will use Python fallback
            print("Warning: C++ library not found, using Python fallback (slower)")
            print(f"  Searched in: {project_root}/build/...")
            print(f"  Library name: {lib_name}")
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
        
        # Check if object has hit ground (altitude <= 0)
        altitude = -self.state.pos_I[2]  # Assuming NED frame (z down)
        if altitude <= 0:
            # Stop simulation - object has hit ground
            # Set position to ground level and zero vertical velocity
            self.state.pos_I[2] = 0.0
            if self.state.vel_I[2] < 0:  # Only stop downward velocity
                self.state.vel_I[2] = 0.0
            # Store final state
            self.history.append(self.state.copy())
            self.time_history.append(current_time)
            if callback:
                callback(self.state, current_time)
            return
        
        # Get control inputs
        control = self._get_control_inputs(current_time)
        
        # Get aerodynamic coefficients
        aero_coeffs = self._get_aerodynamic_coefficients(current_time)
        
        # Get thrust
        thrust_data = self._get_thrust_data(current_time)
        
        # Get atmosphere
        atm_state = self.atmosphere.get_atmosphere(altitude)
        wind_I = self.atmosphere.get_wind(self.state.pos_I, current_time)
        
        # Step using C++ if available, otherwise Python fallback
        if self._cpp_lib is not None:
            self._step_cpp(dt, control, aero_coeffs, thrust_data, atm_state, wind_I)
        else:
            self._step_python(dt, control, aero_coeffs, thrust_data, atm_state, wind_I)
        
        # Check again after step to prevent going below ground
        altitude_after = -self.state.pos_I[2]
        if altitude_after < 0:
            self.state.pos_I[2] = 0.0
            if self.state.vel_I[2] < 0:
                self.state.vel_I[2] = 0.0
        
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
        """Step using Python implementation."""
        # Compute relative velocity in body frame
        vel_B = self._inertial_to_body(self.state.vel_I, self.state.quat_BI)
        wind_B = self._inertial_to_body(wind_I, self.state.quat_BI)
        v_rel_B = vel_B - wind_B
        
        # Transform to aerodynamic frame
        v_rel_A = self._body_to_aero(v_rel_B)
        
        # Compute aerodynamic angles and dynamic pressure
        V_mag = np.linalg.norm(v_rel_A)
        if V_mag < 1e-6:
            alpha = beta = 0.0
        else:
            alpha = np.arctan2(-v_rel_A[2], -v_rel_A[0])
            beta = np.arcsin(v_rel_A[1] / V_mag)
        
        dynamic_pressure = 0.5 * atm_state["density"] * V_mag * V_mag
        
        # Compute aerodynamic forces in aerodynamic frame
        force_A = np.array([
            -aero_coeffs["Cx"] * dynamic_pressure * self.vehicle.ref_area,
            aero_coeffs["Cy"] * dynamic_pressure * self.vehicle.ref_area,
            -aero_coeffs["Cz"] * dynamic_pressure * self.vehicle.ref_area,
        ])
        
        # Transform to body frame
        force_B = self._aero_to_body(force_A)
        
        # Add thrust
        force_B += thrust_data["thrust_B"]
        
        # Transform force to inertial frame and add gravity
        force_I = self._body_to_inertial(force_B, self.state.quat_BI)
        gravity = np.array(self.mission.gravity)
        force_I += gravity * self.state.mass
        
        # Compute acceleration
        if self.state.mass > 1e-6:
            accel_I = force_I / self.state.mass
        else:
            accel_I = np.zeros(3)
        
        # Update velocity and position (Euler integration)
        self.state.vel_I += accel_I * dt
        
        # Update position, but check for ground collision
        new_pos = self.state.pos_I + self.state.vel_I * dt
        new_altitude = -new_pos[2]
        
        if new_altitude < 0:
            # Object would go below ground, stop at ground level
            self.state.pos_I[2] = 0.0
            # Zero out downward velocity component
            if self.state.vel_I[2] < 0:
                self.state.vel_I[2] = 0.0
        else:
            self.state.pos_I = new_pos
        
        # Compute aerodynamic moments in aerodynamic frame
        moment_A = np.array([
            aero_coeffs["Cmx"] * dynamic_pressure * self.vehicle.ref_area * self.vehicle.ref_length,
            aero_coeffs["Cmy"] * dynamic_pressure * self.vehicle.ref_area * self.vehicle.ref_length,
            aero_coeffs["Cmz"] * dynamic_pressure * self.vehicle.ref_area * self.vehicle.ref_length,
        ])
        
        # Transform to body frame
        moment_B = self._aero_to_body(moment_A)
        
        # Compute angular acceleration (simplified - assuming diagonal inertia)
        I = np.array([
            [self.vehicle.Ixx, self.vehicle.Ixy, self.vehicle.Ixz],
            [self.vehicle.Ixy, self.vehicle.Iyy, self.vehicle.Iyz],
            [self.vehicle.Ixz, self.vehicle.Iyz, self.vehicle.Izz],
        ])
        
        # I * omega
        I_omega = I @ self.state.omega_B
        
        # omega x (I * omega)
        omega_cross = np.cross(self.state.omega_B, I_omega)
        
        # M - omega x (I * omega)
        rhs = moment_B - omega_cross
        
        # Solve for angular acceleration (simplified - assume diagonal)
        if abs(self.vehicle.Ixx) > 1e-6:
            alpha_B = np.array([
                rhs[0] / self.vehicle.Ixx,
                rhs[1] / self.vehicle.Iyy,
                rhs[2] / self.vehicle.Izz,
            ])
        else:
            alpha_B = np.zeros(3)
        
        # Update angular velocity
        self.state.omega_B += alpha_B * dt
        
        # Update quaternion (simplified integration)
        w, x, y, z = self.state.quat_BI
        p, q, r = self.state.omega_B
        
        # Quaternion derivative: q_dot = 0.5 * Omega(omega) * q
        q_dot = 0.5 * np.array([
            -p*x - q*y - r*z,
            p*w + q*z - r*y,
            q*w - p*z + r*x,
            r*w + p*y - q*x,
        ])
        
        self.state.quat_BI += q_dot * dt
        # Normalize quaternion
        quat_norm = np.linalg.norm(self.state.quat_BI)
        if quat_norm > 1e-10:
            self.state.quat_BI = self.state.quat_BI / quat_norm
        else:
            self.state.quat_BI = np.array([1.0, 0.0, 0.0, 0.0])
        
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
    
    def _aero_to_body(self, v_A):
        """Transform vector from aerodynamic to body frame."""
        return np.array([-v_A[0], v_A[1], -v_A[2]])
    
    def _body_to_inertial(self, v_B, quat_BI):
        """Transform vector from body to inertial frame."""
        # Inverse of inertial_to_body
        # v_I = q^-1 * v_B * q
        w, x, y, z = quat_BI
        q_inv = np.array([w, -x, -y, -z])  # Conjugate
        
        # Convert to quaternion rotation
        v_quat = np.array([0, v_B[0], v_B[1], v_B[2]])
        
        # q_inv * v_quat
        result = np.array([
            -q_inv[1]*v_B[0] - q_inv[2]*v_B[1] - q_inv[3]*v_B[2],
            q_inv[0]*v_B[0] + q_inv[2]*v_B[2] - q_inv[3]*v_B[1],
            q_inv[0]*v_B[1] - q_inv[1]*v_B[2] + q_inv[3]*v_B[0],
            q_inv[0]*v_B[2] + q_inv[1]*v_B[1] - q_inv[2]*v_B[0],
        ])
        
        # result * q
        v_I = np.array([
            result[1]*x + result[2]*y + result[3]*z,
            -result[0]*x + result[1]*w - result[2]*z + result[3]*y,
            -result[0]*y + result[1]*z + result[2]*w - result[3]*x,
            -result[0]*z - result[1]*y + result[2]*x + result[3]*w,
        ])
        
        return v_I[1:4]  # Return vector part
    
    def calculate_terminal_velocity(self, altitude: Optional[float] = None) -> float:
        """Calculate terminal velocity at given altitude.
        
        Terminal velocity formula: v_t = sqrt(2 * m * g / (rho * A * C_d))
        
        Args:
            altitude: Altitude in meters (uses current altitude if not provided)
        
        Returns:
            Terminal velocity in m/s
        """
        if altitude is None:
            altitude = -self.state.pos_I[2]
        
        # Get atmosphere at altitude
        atm_state = self.atmosphere.get_atmosphere(altitude)
        rho = atm_state["density"]
        
        # Get gravity magnitude
        gravity = np.array(self.mission.gravity)
        g = np.linalg.norm(gravity)
        
        # Get drag coefficient (Cx in aerodynamic frame)
        # For terminal velocity, we need the drag coefficient at zero angle of attack
        # Use a simple evaluation with alpha=0, beta=0
        aero_coeffs = self.aero_model.evaluate(
            alpha=0.0, beta=0.0, mach=0.0,
            control_p=0.0, control_q=0.0, control_r=0.0, control_i=0.0
        )
        C_d = aero_coeffs["Cx"]  # Drag coefficient
        
        # Terminal velocity: v_t = sqrt(2 * m * g / (rho * A * C_d))
        if rho > 1e-10 and self.vehicle.ref_area > 1e-10 and C_d > 1e-10:
            v_t = np.sqrt(2 * self.state.mass * g / (rho * self.vehicle.ref_area * C_d))
        else:
            v_t = 0.0
        
        return v_t
    
    def run(self, callback: Optional[Callable] = None):
        """Run simulation from start to end time.
        
        Args:
            callback: Optional callback function(state, time) called after each step
        """
        self.history = [self.state.copy()]
        self.time_history = [self.mission.t_start]
        
        t = self.mission.t_start
        while t < self.mission.t_end:
            # Check if object has hit ground before stepping
            altitude = -self.state.pos_I[2]
            if altitude <= 0:
                # Object has hit ground, stop simulation
                break
            
            self.step(callback=callback)
            t += self.mission.dt
            
            # Check again after step
            altitude = -self.state.pos_I[2]
            if altitude <= 0:
                # Object hit ground during step, stop
                break
    
    def get_history(self):
        """Get simulation history."""
        return {
            "states": self.history,
            "times": self.time_history,
        }
