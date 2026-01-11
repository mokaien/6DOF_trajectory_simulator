#ifndef TRAJECTORY_RK4_HPP
#define TRAJECTORY_RK4_HPP

#include "../dynamics/rigid_body.hpp"
#include <functional>

namespace trajectory {
namespace integrators {

using namespace trajectory::math;
using namespace trajectory::dynamics;

/**
 * Function type for computing state derivatives
 * Takes state and time, returns derivatives
 */
using DerivativeFunction = std::function<StateDerivative(
    const Vector3& pos_I,
    const Vector3& vel_I,
    const Quaternion& quat_BI,
    const Vector3& omega_B,
    double mass,
    double time
)>;

/**
 * 4th-order Runge-Kutta integrator
 */
class RK4Integrator {
public:
    /**
     * Integrate one time step using RK4
     * 
     * @param pos_I Current position in inertial frame
     * @param vel_I Current velocity in inertial frame
     * @param quat_BI Current quaternion (inertial to body)
     * @param omega_B Current angular velocity in body frame
     * @param mass Current mass
     * @param time Current time
     * @param dt Time step
     * @param compute_deriv Function to compute derivatives
     * @return New state after integration
     */
    static void integrate(
        Vector3& pos_I,
        Vector3& vel_I,
        Quaternion& quat_BI,
        Vector3& omega_B,
        double& mass,
        double time,
        double dt,
        const DerivativeFunction& compute_deriv
    ) {
        // k1 = f(t, y)
        StateDerivative k1 = compute_deriv(pos_I, vel_I, quat_BI, omega_B, mass, time);
        
        // k2 = f(t + dt/2, y + dt*k1/2)
        Vector3 pos_k2 = pos_I + k1.d_pos_I * (dt / 2.0);
        Vector3 vel_k2 = vel_I + k1.d_vel_I * (dt / 2.0);
        Quaternion quat_k2 = quat_BI;
        // Approximate quaternion update: q_new ≈ q + dt/2 * q_dot
        quat_k2.w += k1.d_quat_BI.w * (dt / 2.0);
        quat_k2.x += k1.d_quat_BI.x * (dt / 2.0);
        quat_k2.y += k1.d_quat_BI.y * (dt / 2.0);
        quat_k2.z += k1.d_quat_BI.z * (dt / 2.0);
        quat_k2.normalize();
        Vector3 omega_k2 = omega_B + k1.d_omega_B * (dt / 2.0);
        double mass_k2 = mass + k1.d_mass * (dt / 2.0);
        
        StateDerivative k2 = compute_deriv(pos_k2, vel_k2, quat_k2, omega_k2, mass_k2, time + dt / 2.0);
        
        // k3 = f(t + dt/2, y + dt*k2/2)
        Vector3 pos_k3 = pos_I + k2.d_pos_I * (dt / 2.0);
        Vector3 vel_k3 = vel_I + k2.d_vel_I * (dt / 2.0);
        Quaternion quat_k3 = quat_BI;
        quat_k3.w += k2.d_quat_BI.w * (dt / 2.0);
        quat_k3.x += k2.d_quat_BI.x * (dt / 2.0);
        quat_k3.y += k2.d_quat_BI.y * (dt / 2.0);
        quat_k3.z += k2.d_quat_BI.z * (dt / 2.0);
        quat_k3.normalize();
        Vector3 omega_k3 = omega_B + k2.d_omega_B * (dt / 2.0);
        double mass_k3 = mass + k2.d_mass * (dt / 2.0);
        
        StateDerivative k3 = compute_deriv(pos_k3, vel_k3, quat_k3, omega_k3, mass_k3, time + dt / 2.0);
        
        // k4 = f(t + dt, y + dt*k3)
        Vector3 pos_k4 = pos_I + k3.d_pos_I * dt;
        Vector3 vel_k4 = vel_I + k3.d_vel_I * dt;
        Quaternion quat_k4 = quat_BI;
        quat_k4.w += k3.d_quat_BI.w * dt;
        quat_k4.x += k3.d_quat_BI.x * dt;
        quat_k4.y += k3.d_quat_BI.y * dt;
        quat_k4.z += k3.d_quat_BI.z * dt;
        quat_k4.normalize();
        Vector3 omega_k4 = omega_B + k3.d_omega_B * dt;
        double mass_k4 = mass + k3.d_mass * dt;
        
        StateDerivative k4 = compute_deriv(pos_k4, vel_k4, quat_k4, omega_k4, mass_k4, time + dt);
        
        // Combine: y_new = y + dt/6 * (k1 + 2*k2 + 2*k3 + k4)
        pos_I += (k1.d_pos_I + k2.d_pos_I * 2.0 + k3.d_pos_I * 2.0 + k4.d_pos_I) * (dt / 6.0);
        vel_I += (k1.d_vel_I + k2.d_vel_I * 2.0 + k3.d_vel_I * 2.0 + k4.d_vel_I) * (dt / 6.0);
        
        // For quaternion, use proper quaternion integration
        // More accurate: use exponential map or quaternion multiplication
        // Simplified: linear combination with normalization
        quat_BI.w += (k1.d_quat_BI.w + 2.0 * k2.d_quat_BI.w + 2.0 * k3.d_quat_BI.w + k4.d_quat_BI.w) * (dt / 6.0);
        quat_BI.x += (k1.d_quat_BI.x + 2.0 * k2.d_quat_BI.x + 2.0 * k3.d_quat_BI.x + k4.d_quat_BI.x) * (dt / 6.0);
        quat_BI.y += (k1.d_quat_BI.y + 2.0 * k2.d_quat_BI.y + 2.0 * k3.d_quat_BI.y + k4.d_quat_BI.y) * (dt / 6.0);
        quat_BI.z += (k1.d_quat_BI.z + 2.0 * k2.d_quat_BI.z + 2.0 * k3.d_quat_BI.z + k4.d_quat_BI.z) * (dt / 6.0);
        quat_BI.normalize();
        
        omega_B += (k1.d_omega_B + k2.d_omega_B * 2.0 + k3.d_omega_B * 2.0 + k4.d_omega_B) * (dt / 6.0);
        mass += (k1.d_mass + 2.0 * k2.d_mass + 2.0 * k3.d_mass + k4.d_mass) * (dt / 6.0);
    }
};

} // namespace integrators
} // namespace trajectory

#endif // TRAJECTORY_RK4_HPP
