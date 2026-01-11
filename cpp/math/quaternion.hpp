#ifndef TRAJECTORY_QUATERNION_HPP
#define TRAJECTORY_QUATERNION_HPP

#include "vector.hpp"
#include <array>
#include <cmath>

namespace trajectory {
namespace math {

/**
 * Quaternion representation: q = [w, x, y, z]
 * where w is the scalar part and (x, y, z) is the vector part
 */
struct Quaternion {
    double w, x, y, z;

    Quaternion() : w(1.0), x(0.0), y(0.0), z(0.0) {}
    Quaternion(double w, double x, double y, double z) : w(w), x(x), y(y), z(z) {}

    // Array access
    double& operator[](int i) {
        double* arr = &w;
        return arr[i];
    }
    const double& operator[](int i) const {
        const double* arr = &w;
        return arr[i];
    }

    // Quaternion multiplication
    Quaternion operator*(const Quaternion& other) const {
        return Quaternion(
            w * other.w - x * other.x - y * other.y - z * other.z,
            w * other.x + x * other.w + y * other.z - z * other.y,
            w * other.y - x * other.z + y * other.w + z * other.x,
            w * other.z + x * other.y - y * other.x + z * other.w
        );
    }

    // Quaternion conjugate
    Quaternion conjugate() const {
        return Quaternion(w, -x, -y, -z);
    }

    // Quaternion inverse (for unit quaternions, inverse = conjugate)
    Quaternion inverse() const {
        double norm_sq = w * w + x * x + y * y + z * z;
        if (norm_sq > 1e-10) {
            double inv_norm_sq = 1.0 / norm_sq;
            return Quaternion(w * inv_norm_sq, -x * inv_norm_sq, -y * inv_norm_sq, -z * inv_norm_sq);
        }
        return Quaternion(1.0, 0.0, 0.0, 0.0);
    }

    // Magnitude
    double magnitude() const {
        return std::sqrt(w * w + x * x + y * y + z * z);
    }

    // Magnitude squared
    double magnitude_squared() const {
        return w * w + x * x + y * y + z * z;
    }

    // Normalize
    Quaternion normalized() const {
        double mag = magnitude();
        if (mag > 1e-10) {
            return Quaternion(w / mag, x / mag, y / mag, z / mag);
        }
        return Quaternion(1.0, 0.0, 0.0, 0.0);
    }

    // Normalize in place
    void normalize() {
        double mag = magnitude();
        if (mag > 1e-10) {
            w /= mag;
            x /= mag;
            y /= mag;
            z /= mag;
        } else {
            w = 1.0;
            x = y = z = 0.0;
        }
    }

    // Rotate a vector by this quaternion
    // v_rotated = q * v * q^-1
    Vector3 rotate(const Vector3& v) const {
        // Convert vector to quaternion (pure quaternion)
        Quaternion v_quat(0.0, v.x, v.y, v.z);
        
        // q * v * q^-1
        Quaternion result = (*this) * v_quat * this->inverse();
        
        return Vector3(result.x, result.y, result.z);
    }

    // Create rotation matrix from quaternion
    // Returns 3x3 rotation matrix as array of arrays
    std::array<std::array<double, 3>, 3> to_rotation_matrix() const {
        double w2 = w * w;
        double x2 = x * x;
        double y2 = y * y;
        double z2 = z * z;
        
        std::array<std::array<double, 3>, 3> R;
        
        R[0][0] = w2 + x2 - y2 - z2;
        R[0][1] = 2.0 * (x * y - w * z);
        R[0][2] = 2.0 * (x * z + w * y);
        
        R[1][0] = 2.0 * (x * y + w * z);
        R[1][1] = w2 - x2 + y2 - z2;
        R[1][2] = 2.0 * (y * z - w * x);
        
        R[2][0] = 2.0 * (x * z - w * y);
        R[2][1] = 2.0 * (y * z + w * x);
        R[2][2] = w2 - x2 - y2 + z2;
        
        return R;
    }

    // Create quaternion from rotation matrix
    static Quaternion from_rotation_matrix(const std::array<std::array<double, 3>, 3>& R) {
        double trace = R[0][0] + R[1][1] + R[2][2];
        double w, x, y, z;
        
        if (trace > 0) {
            double s = 0.5 / std::sqrt(trace + 1.0);
            w = 0.25 / s;
            x = (R[2][1] - R[1][2]) * s;
            y = (R[0][2] - R[2][0]) * s;
            z = (R[1][0] - R[0][1]) * s;
        } else {
            if (R[0][0] > R[1][1] && R[0][0] > R[2][2]) {
                double s = 2.0 * std::sqrt(1.0 + R[0][0] - R[1][1] - R[2][2]);
                w = (R[2][1] - R[1][2]) / s;
                x = 0.25 * s;
                y = (R[0][1] + R[1][0]) / s;
                z = (R[0][2] + R[2][0]) / s;
            } else if (R[1][1] > R[2][2]) {
                double s = 2.0 * std::sqrt(1.0 + R[1][1] - R[0][0] - R[2][2]);
                w = (R[0][2] - R[2][0]) / s;
                x = (R[0][1] + R[1][0]) / s;
                y = 0.25 * s;
                z = (R[1][2] + R[2][1]) / s;
            } else {
                double s = 2.0 * std::sqrt(1.0 + R[2][2] - R[0][0] - R[1][1]);
                w = (R[1][0] - R[0][1]) / s;
                x = (R[0][2] + R[2][0]) / s;
                y = (R[1][2] + R[2][1]) / s;
                z = 0.25 * s;
            }
        }
        
        return Quaternion(w, x, y, z).normalized();
    }

    // Create quaternion from axis-angle representation
    static Quaternion from_axis_angle(const Vector3& axis, double angle) {
        double half_angle = 0.5 * angle;
        double s = std::sin(half_angle);
        Vector3 normalized_axis = axis.normalized();
        
        return Quaternion(
            std::cos(half_angle),
            normalized_axis.x * s,
            normalized_axis.y * s,
            normalized_axis.z * s
        );
    }

    // Convert to array [w, x, y, z]
    std::array<double, 4> to_array() const {
        return {w, x, y, z};
    }

    // Create from array [w, x, y, z]
    static Quaternion from_array(const std::array<double, 4>& arr) {
        return Quaternion(arr[0], arr[1], arr[2], arr[3]);
    }
};

} // namespace math
} // namespace trajectory

#endif // TRAJECTORY_QUATERNION_HPP
