#ifndef TRAJECTORY_VECTOR_HPP
#define TRAJECTORY_VECTOR_HPP

#include <array>
#include <cmath>

namespace trajectory {
namespace math {

/**
 * 3D vector operations
 */
struct Vector3 {
    double x, y, z;

    Vector3() : x(0.0), y(0.0), z(0.0) {}
    Vector3(double x, double y, double z) : x(x), y(y), z(z) {}

    // Array access
    double& operator[](int i) { return (&x)[i]; }
    const double& operator[](int i) const { return (&x)[i]; }

    // Arithmetic operations
    Vector3 operator+(const Vector3& other) const {
        return Vector3(x + other.x, y + other.y, z + other.z);
    }

    Vector3 operator-(const Vector3& other) const {
        return Vector3(x - other.x, y - other.y, z - other.z);
    }

    Vector3 operator*(double scalar) const {
        return Vector3(x * scalar, y * scalar, z * scalar);
    }

    Vector3 operator/(double scalar) const {
        return Vector3(x / scalar, y / scalar, z / scalar);
    }

    Vector3& operator+=(const Vector3& other) {
        x += other.x;
        y += other.y;
        z += other.z;
        return *this;
    }

    Vector3& operator-=(const Vector3& other) {
        x -= other.x;
        y -= other.y;
        z -= other.z;
        return *this;
    }

    Vector3& operator*=(double scalar) {
        x *= scalar;
        y *= scalar;
        z *= scalar;
        return *this;
    }

    // Dot product
    double dot(const Vector3& other) const {
        return x * other.x + y * other.y + z * other.z;
    }

    // Cross product
    Vector3 cross(const Vector3& other) const {
        return Vector3(
            y * other.z - z * other.y,
            z * other.x - x * other.z,
            x * other.y - y * other.x
        );
    }

    // Magnitude
    double magnitude() const {
        return std::sqrt(x * x + y * y + z * z);
    }

    // Magnitude squared (faster, no sqrt)
    double magnitude_squared() const {
        return x * x + y * y + z * z;
    }

    // Normalize (returns normalized vector)
    Vector3 normalized() const {
        double mag = magnitude();
        if (mag > 1e-10) {
            return Vector3(x / mag, y / mag, z / mag);
        }
        return Vector3(0, 0, 0);
    }

    // Normalize in place
    void normalize() {
        double mag = magnitude();
        if (mag > 1e-10) {
            x /= mag;
            y /= mag;
            z /= mag;
        } else {
            x = y = z = 0.0;
        }
    }

    // Convert to array
    std::array<double, 3> to_array() const {
        return {x, y, z};
    }

    // Create from array
    static Vector3 from_array(const std::array<double, 3>& arr) {
        return Vector3(arr[0], arr[1], arr[2]);
    }
};

// Scalar multiplication (left side)
inline Vector3 operator*(double scalar, const Vector3& vec) {
    return vec * scalar;
}

} // namespace math
} // namespace trajectory

#endif // TRAJECTORY_VECTOR_HPP
