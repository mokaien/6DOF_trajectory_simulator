#ifndef TRAJECTORY_SIMULATOR_API_HPP
#define TRAJECTORY_SIMULATOR_API_HPP

#ifdef __cplusplus
extern "C" {
#endif

/**
 * C-style API for Python bindings
 * 
 * This API provides a simple C interface that can be easily called from Python
 * using ctypes or pybind11
 */

/**
 * State structure (matches specification)
 */
struct State {
    double pos_I[3];      // Position in inertial frame [x, y, z]
    double vel_I[3];      // Velocity in inertial frame [vx, vy, vz]
    double quat_BI[4];    // Quaternion from inertial to body [w, x, y, z]
    double omega_B[3];    // Angular velocity in body frame [p, q, r]
    double mass;          // Mass (kg)
};

/**
 * Control input structure (matches specification)
 */
struct ControlInput {
    double p;  // Roll control
    double q;  // Pitch control
    double r;  // Yaw control
    double i;  // Brake/drag control
};

/**
 * Vehicle parameters structure
 */
struct VehicleParams {
    double mass;           // Initial mass (kg)
    double Ixx, Iyy, Izz;  // Principal moments of inertia (kg*m^2)
    double Ixy, Ixz, Iyz;  // Products of inertia (kg*m^2)
    double ref_area;       // Reference area for aerodynamics (m^2)
    double ref_length;     // Reference length for moments (m)
};

/**
 * Environment parameters structure
 */
struct EnvironmentParams {
    double gravity[3];     // Gravity vector in inertial frame [gx, gy, gz]
    double wind_I[3];      // Wind velocity in inertial frame [wx, wy, wz]
    double density;        // Atmospheric density (kg/m^3)
    double speed_of_sound; // Speed of sound (m/s)
};

/**
 * Aerodynamic coefficients structure
 */
struct AeroCoeffs {
    double Cx, Cy, Cz;     // Force coefficients in aero frame
    double Cmx, Cmy, Cmz;  // Moment coefficients in aero frame
};

/**
 * Thrust structure
 */
struct ThrustData {
    double thrust_B[3];    // Thrust vector in body frame [Tx, Ty, Tz]
    double mass_flow_rate;  // Mass flow rate (kg/s, negative for mass loss)
};

/**
 * Step simulation forward one time step
 * 
 * This function integrates the 6-DOF equations of motion for one time step.
 * It expects the caller to provide:
 * - Current state
 * - Control inputs
 * - Aerodynamic coefficients
 * - Thrust data
 * - Vehicle and environment parameters
 * 
 * The state is updated in place.
 * 
 * @param state Current state (updated in place)
 * @param control Control inputs
 * @param aero_coeffs Aerodynamic coefficients
 * @param thrust Thrust data
 * @param vehicle Vehicle parameters
 * @param env Environment parameters
 * @param time Current time
 * @param dt Time step
 */
void step_simulation(
    State* state,
    const ControlInput* control,
    const AeroCoeffs* aero_coeffs,
    const ThrustData* thrust,
    const VehicleParams* vehicle,
    const EnvironmentParams* env,
    double time,
    double dt
);

#ifdef __cplusplus
}
#endif

#endif // TRAJECTORY_SIMULATOR_API_HPP
