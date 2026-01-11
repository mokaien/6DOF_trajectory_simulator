#include "simulator_api.hpp"
#include "../math/vector.hpp"
#include "../math/quaternion.hpp"
#include "../frames/transform.hpp"
#include "../dynamics/rigid_body.hpp"
#include "../integrators/rk4.hpp"
#include "../models/aero_model.hpp"
#include <cmath>
#include <utility>

using namespace trajectory;
using namespace trajectory::math;
using namespace trajectory::frames;
using namespace trajectory::dynamics;
using namespace trajectory::integrators;
using namespace trajectory::models;

// Internal helper functions
static Vector3 array_to_vector(const double arr[3]) {
    return Vector3(arr[0], arr[1], arr[2]);
}

static void vector_to_array(const Vector3& vec, double arr[3]) {
    arr[0] = vec.x;
    arr[1] = vec.y;
    arr[2] = vec.z;
}

static Quaternion array_to_quaternion(const double arr[4]) {
    return Quaternion(arr[0], arr[1], arr[2], arr[3]);
}

static void quaternion_to_array(const Quaternion& q, double arr[4]) {
    arr[0] = q.w;
    arr[1] = q.x;
    arr[2] = q.y;
    arr[3] = q.z;
}

// C API implementation
extern "C" {

void step_simulation(
    State* state,
    const ControlInput* control,
    const AeroCoeffs* aero_coeffs,
    const ThrustData* thrust,
    const VehicleParams* vehicle,
    const EnvironmentParams* env,
    double time,
    double dt
) {

    // Convert C structures to C++ types
    Vector3 pos_I = array_to_vector(state->pos_I);
    Vector3 vel_I = array_to_vector(state->vel_I);
    Quaternion quat_BI = array_to_quaternion(state->quat_BI);
    quat_BI.normalize();  // Ensure normalized
    Vector3 omega_B = array_to_vector(state->omega_B);
    double mass = state->mass;

    // Vehicle parameters
    InertiaTensor I(vehicle->Ixx, vehicle->Iyy, vehicle->Izz,
                    vehicle->Ixy, vehicle->Ixz, vehicle->Iyz);

    // Environment
    Vector3 gravity = array_to_vector(env->gravity);
    Vector3 wind_I = array_to_vector(env->wind_I);

    // Compute relative wind velocity in body frame
    Vector3 vel_B = inertial_to_body(vel_I, quat_BI);
    Vector3 wind_B = inertial_to_body(wind_I, quat_BI);
    Vector3 v_rel_B = vel_B - wind_B;

    // Transform to aerodynamic frame
    Vector3 v_rel_A = body_to_aero(v_rel_B);

    // Compute aerodynamic angles and dynamic pressure
    auto aero_angles = compute_aero_angles(v_rel_A);
    double alpha = aero_angles[0];
    double beta = aero_angles[1];
    double V_mag = aero_angles[2];
    double mach = V_mag / env->speed_of_sound;
    if (env->speed_of_sound < 1e-6) mach = 0.0;

    // Get aerodynamic coefficients (provided by caller)
    AeroCoefficients coeffs;
    coeffs.Cx = aero_coeffs->Cx;
    coeffs.Cy = aero_coeffs->Cy;
    coeffs.Cz = aero_coeffs->Cz;
    coeffs.Cmx = aero_coeffs->Cmx;
    coeffs.Cmy = aero_coeffs->Cmy;
    coeffs.Cmz = aero_coeffs->Cmz;

    // Compute dynamic pressure
    double dynamic_pressure = 0.5 * env->density * V_mag * V_mag;

    // Compute aerodynamic forces and moments in aerodynamic frame
    std::pair<Vector3, Vector3> aero_forces_moments = compute_aero_forces_moments(
        coeffs, dynamic_pressure, vehicle->ref_area, vehicle->ref_length
    );
    Vector3 force_A = aero_forces_moments.first;
    Vector3 moment_A = aero_forces_moments.second;

    // Transform forces and moments to body frame
    Vector3 force_B = aero_to_body(force_A);
    Vector3 moment_B = aero_to_body(moment_A);

    // Add thrust in body frame
    Vector3 thrust_B = array_to_vector(thrust->thrust_B);
    force_B += thrust_B;

    // Add gravity in inertial frame
    Vector3 force_I = body_to_inertial(force_B, quat_BI) + gravity * mass;

    // Define derivative function for RK4
    auto compute_deriv = [&](const Vector3& p, const Vector3& v, const Quaternion& q,
                             const Vector3& om, double m, double t) -> StateDerivative {
        // Recompute forces at this state (simplified - using same forces)
        // In a more sophisticated implementation, we'd recompute everything
        Vector3 v_B_local = inertial_to_body(v, q);
        Vector3 v_rel_B_local = v_B_local - wind_B;
        Vector3 v_rel_A_local = body_to_aero(v_rel_B_local);
        auto angles_local = compute_aero_angles(v_rel_A_local);
        double V_local = angles_local[2];
        double q_dyn_local = 0.5 * env->density * V_local * V_local;
        
        // Use same coefficients (could be improved with state-dependent coefficients)
        std::pair<Vector3, Vector3> aero_fm_local = compute_aero_forces_moments(
            coeffs, q_dyn_local, vehicle->ref_area, vehicle->ref_length
        );
        Vector3 f_A_local = aero_fm_local.first;
        Vector3 m_A_local = aero_fm_local.second;
        Vector3 f_B_local = aero_to_body(f_A_local) + thrust_B;
        Vector3 f_I_local = body_to_inertial(f_B_local, q) + gravity * m;
        Vector3 m_B_local = aero_to_body(m_A_local);

        return compute_state_derivatives(
            p, v, q, om, m, f_I_local, m_B_local, I
        );
    };

    // Integrate one step using RK4
    RK4Integrator::integrate(
        pos_I, vel_I, quat_BI, omega_B, mass,
        time, dt, compute_deriv
    );

    // Update mass from thrust model
    mass += thrust->mass_flow_rate * dt;
    if (mass < 0.1) mass = 0.1;  // Prevent negative mass

    // Convert back to C structures
    vector_to_array(pos_I, state->pos_I);
    vector_to_array(vel_I, state->vel_I);
    quaternion_to_array(quat_BI, state->quat_BI);
    vector_to_array(omega_B, state->omega_B);
    state->mass = mass;
}

} // extern "C"
