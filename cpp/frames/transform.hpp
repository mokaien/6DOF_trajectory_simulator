#ifndef TRAJECTORY_TRANSFORM_HPP
#define TRAJECTORY_TRANSFORM_HPP

#include "../math/vector.hpp"
#include "../math/quaternion.hpp"
#include <array>

namespace trajectory {
namespace frames {

using namespace trajectory::math;

/**
 * Coordinate frame transformations
 * 
 * Frames:
 * - I: Inertial frame (Earth-fixed NED or Cartesian)
 * - B: Body frame (origin at CoM, +X forward, +Y right, +Z up)
 * - A: Aerodynamic frame (origin at nose, +X backward, +Y right, +Z down)
 */

/**
 * Transform vector from inertial frame to body frame
 * v_B = q_BI * v_I * q_BI^-1
 */
inline Vector3 inertial_to_body(const Vector3& v_I, const Quaternion& q_BI) {
    return q_BI.rotate(v_I);
}

/**
 * Transform vector from body frame to inertial frame
 * v_I = q_BI^-1 * v_B * q_BI
 */
inline Vector3 body_to_inertial(const Vector3& v_B, const Quaternion& q_BI) {
    return q_BI.conjugate().rotate(v_B);
}

/**
 * Transform vector from body frame to aerodynamic frame
 * 
 * Body frame: +X forward, +Y right, +Z up
 * Aero frame: +X backward, +Y right, +Z down
 * 
 * Transformation: rotate 180° about Y axis
 */
inline Vector3 body_to_aero(const Vector3& v_B) {
    // Rotation matrix for 180° about Y: [X, Y, Z] -> [-X, Y, -Z]
    return Vector3(-v_B.x, v_B.y, -v_B.z);
}

/**
 * Transform vector from aerodynamic frame to body frame
 */
inline Vector3 aero_to_body(const Vector3& v_A) {
    // Inverse of body_to_aero (same transformation, it's symmetric)
    return Vector3(-v_A.x, v_A.y, -v_A.z);
}

/**
 * Transform vector from inertial frame to aerodynamic frame
 * v_A = body_to_aero(inertial_to_body(v_I, q_BI))
 */
inline Vector3 inertial_to_aero(const Vector3& v_I, const Quaternion& q_BI) {
    Vector3 v_B = inertial_to_body(v_I, q_BI);
    return body_to_aero(v_B);
}

/**
 * Transform vector from aerodynamic frame to inertial frame
 * v_I = body_to_inertial(aero_to_body(v_A), q_BI)
 */
inline Vector3 aero_to_inertial(const Vector3& v_A, const Quaternion& q_BI) {
    Vector3 v_B = aero_to_body(v_A);
    return body_to_inertial(v_B, q_BI);
}

/**
 * Get rotation matrix from inertial to body frame
 */
inline std::array<std::array<double, 3>, 3> get_rotation_matrix_IB(const Quaternion& q_BI) {
    return q_BI.to_rotation_matrix();
}

/**
 * Get rotation matrix from body to inertial frame
 */
inline std::array<std::array<double, 3>, 3> get_rotation_matrix_BI(const Quaternion& q_BI) {
    return q_BI.conjugate().to_rotation_matrix();
}

/**
 * Get rotation matrix from body to aerodynamic frame
 */
inline std::array<std::array<double, 3>, 3> get_rotation_matrix_BA() {
    // 180° rotation about Y axis
    std::array<std::array<double, 3>, 3> R;
    R[0][0] = -1.0; R[0][1] = 0.0;  R[0][2] = 0.0;
    R[1][0] = 0.0;  R[1][1] = 1.0;  R[1][2] = 0.0;
    R[2][0] = 0.0;  R[2][1] = 0.0;  R[2][2] = -1.0;
    return R;
}

/**
 * Compute relative wind velocity in body frame
 * v_rel_B = v_B - v_wind_B
 * where v_wind_B is wind velocity transformed to body frame
 */
inline Vector3 compute_relative_wind_body(
    const Vector3& velocity_body,
    const Vector3& wind_inertial,
    const Quaternion& q_BI
) {
    Vector3 wind_body = inertial_to_body(wind_inertial, q_BI);
    return velocity_body - wind_body;
}

/**
 * Compute aerodynamic angles (angle of attack, sideslip)
 * from relative velocity in aerodynamic frame
 * 
 * Returns: [alpha (AoA), beta (sideslip), V_mag]
 */
inline std::array<double, 3> compute_aero_angles(const Vector3& v_rel_A) {
    double V_mag = v_rel_A.magnitude();
    
    if (V_mag < 1e-6) {
        return {0.0, 0.0, 0.0};
    }
    
    // Angle of attack: alpha = atan2(w, u) where u is -X (backward) and w is -Z (down)
    // In aero frame: +X is backward, so v_rel_A.x is negative of forward velocity
    double alpha = std::atan2(-v_rel_A.z, -v_rel_A.x);
    
    // Sideslip: beta = asin(v / V) where v is Y component
    double beta = std::asin(v_rel_A.y / V_mag);
    
    return {alpha, beta, V_mag};
}

} // namespace frames
} // namespace trajectory

#endif // TRAJECTORY_TRANSFORM_HPP
