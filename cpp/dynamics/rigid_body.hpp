#ifndef TRAJECTORY_RIGID_BODY_HPP
#define TRAJECTORY_RIGID_BODY_HPP

#include "../math/vector.hpp"
#include "../math/quaternion.hpp"
#include <array>

namespace trajectory {
namespace dynamics {

using namespace trajectory::math;

/**
 * Inertia tensor (3x3 symmetric matrix)
 */
struct InertiaTensor {
    std::array<std::array<double, 3>, 3> I;

    InertiaTensor() {
        // Initialize to identity
        for (int i = 0; i < 3; i++) {
            for (int j = 0; j < 3; j++) {
                I[i][j] = (i == j) ? 1.0 : 0.0;
            }
        }
    }

    InertiaTensor(double Ixx, double Iyy, double Izz,
                  double Ixy = 0.0, double Ixz = 0.0, double Iyz = 0.0) {
        I[0][0] = Ixx; I[0][1] = Ixy; I[0][2] = Ixz;
        I[1][0] = Ixy; I[1][1] = Iyy; I[1][2] = Iyz;
        I[2][0] = Ixz; I[2][1] = Iyz; I[2][2] = Izz;
    }

    // Matrix-vector multiplication: I * omega
    Vector3 multiply(const Vector3& omega) const {
        return Vector3(
            I[0][0] * omega.x + I[0][1] * omega.y + I[0][2] * omega.z,
            I[1][0] * omega.x + I[1][1] * omega.y + I[1][2] * omega.z,
            I[2][0] * omega.x + I[2][1] * omega.y + I[2][2] * omega.z
        );
    }
};

/**
 * State derivative structure
 */
struct StateDerivative {
    Vector3 d_pos_I;      // Position derivative (velocity)
    Vector3 d_vel_I;      // Velocity derivative (acceleration)
    Quaternion d_quat_BI; // Quaternion derivative
    Vector3 d_omega_B;    // Angular velocity derivative
    double d_mass;        // Mass derivative

    StateDerivative() : d_pos_I(), d_vel_I(), d_quat_BI(), d_omega_B(), d_mass(0.0) {}
};

/**
 * Compute quaternion derivative matrix Omega(omega)
 * q_dot = 0.5 * Omega(omega) * q
 */
inline std::array<std::array<double, 4>, 4> quaternion_derivative_matrix(const Vector3& omega) {
    std::array<std::array<double, 4>, 4> Omega;
    
    // Omega matrix for quaternion derivative
    Omega[0][0] = 0.0;     Omega[0][1] = -omega.x; Omega[0][2] = -omega.y; Omega[0][3] = -omega.z;
    Omega[1][0] = omega.x; Omega[1][1] = 0.0;      Omega[1][2] = omega.z;  Omega[1][3] = -omega.y;
    Omega[2][0] = omega.y; Omega[2][1] = -omega.z; Omega[2][2] = 0.0;       Omega[2][3] = omega.x;
    Omega[3][0] = omega.z; Omega[3][1] = omega.y;  Omega[3][2] = -omega.x;  Omega[3][3] = 0.0;
    
    return Omega;
}

/**
 * Compute quaternion derivative
 * q_dot = 0.5 * Omega(omega) * q
 */
inline Quaternion compute_quaternion_derivative(const Quaternion& q, const Vector3& omega_B) {
    // q_dot = 0.5 * [w*omega, omega x q_vec + w*omega]
    // More efficient direct computation
    double w = q.w;
    double x = q.x;
    double y = q.y;
    double z = q.z;
    
    return Quaternion(
        0.5 * (-omega_B.x * x - omega_B.y * y - omega_B.z * z),
        0.5 * (omega_B.x * w + omega_B.y * z - omega_B.z * y),
        0.5 * (omega_B.y * w - omega_B.x * z + omega_B.z * x),
        0.5 * (omega_B.z * w + omega_B.x * y - omega_B.y * x)
    );
}

/**
 * Compute angular acceleration from Euler's equations
 * I * omega_dot + omega x (I * omega) = M
 * omega_dot = I^-1 * (M - omega x (I * omega))
 */
inline Vector3 compute_angular_acceleration(
    const Vector3& omega_B,
    const Vector3& moment_B,
    const InertiaTensor& I
) {
    // Compute I * omega
    Vector3 I_omega = I.multiply(omega_B);
    
    // Compute omega x (I * omega)
    Vector3 omega_cross_I_omega = omega_B.cross(I_omega);
    
    // Compute M - omega x (I * omega)
    Vector3 rhs = moment_B - omega_cross_I_omega;
    
    // Solve I * omega_dot = rhs
    // For now, assume diagonal inertia (simplified)
    // TODO: Implement general matrix inversion for full inertia tensor
    double Ixx = I.I[0][0];
    double Iyy = I.I[1][1];
    double Izz = I.I[2][2];
    
    // Simplified: assume diagonal inertia tensor
    // For full 3x3, need to solve linear system
    return Vector3(
        rhs.x / Ixx,
        rhs.y / Iyy,
        rhs.z / Izz
    );
}

/**
 * Compute state derivatives for 6-DOF rigid body
 * 
 * @param pos_I Position in inertial frame
 * @param vel_I Velocity in inertial frame
 * @param quat_BI Quaternion from inertial to body frame
 * @param omega_B Angular velocity in body frame
 * @param mass Current mass
 * @param force_I Total force in inertial frame
 * @param moment_B Total moment in body frame
 * @param I Inertia tensor
 * @return State derivatives
 */
inline StateDerivative compute_state_derivatives(
    const Vector3& pos_I,
    const Vector3& vel_I,
    const Quaternion& quat_BI,
    const Vector3& omega_B,
    double mass,
    const Vector3& force_I,
    const Vector3& moment_B,
    const InertiaTensor& I
) {
    StateDerivative deriv;
    
    // Translational dynamics: r_dot = v, v_dot = F/m
    deriv.d_pos_I = vel_I;
    if (mass > 1e-6) {
        deriv.d_vel_I = force_I / mass;
    } else {
        deriv.d_vel_I = Vector3(0, 0, 0);
    }
    
    // Rotational dynamics: q_dot = 0.5 * Omega(omega) * q
    deriv.d_quat_BI = compute_quaternion_derivative(quat_BI, omega_B);
    
    // Angular acceleration: I * omega_dot + omega x (I * omega) = M
    deriv.d_omega_B = compute_angular_acceleration(omega_B, moment_B, I);
    
    // Mass derivative (default to zero, can be set by thrust model)
    deriv.d_mass = 0.0;
    
    return deriv;
}

} // namespace dynamics
} // namespace trajectory

#endif // TRAJECTORY_RIGID_BODY_HPP
