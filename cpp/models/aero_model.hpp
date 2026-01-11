#ifndef TRAJECTORY_AERO_MODEL_HPP
#define TRAJECTORY_AERO_MODEL_HPP

#include "../math/vector.hpp"
#include "../frames/transform.hpp"
#include <functional>

namespace trajectory {
namespace models {

using namespace trajectory::math;
using namespace trajectory::frames;

/**
 * Aerodynamic coefficients structure
 */
struct AeroCoefficients {
    double Cx;  // Force coefficient in aero X (drag direction)
    double Cy;  // Force coefficient in aero Y
    double Cz;  // Force coefficient in aero Z
    double Cmx; // Moment coefficient about aero X
    double Cmy; // Moment coefficient about aero Y
    double Cmz; // Moment coefficient about aero Z
};

/**
 * Abstract interface for aerodynamic coefficient models
 * 
 * Models should implement this interface to provide aerodynamic coefficients
 * based on aerodynamic angles, Mach number, control inputs, etc.
 */
class AeroModel {
public:
    virtual ~AeroModel() = default;

    /**
     * Evaluate aerodynamic coefficients
     * 
     * @param alpha Angle of attack (rad)
     * @param beta Sideslip angle (rad)
     * @param mach Mach number
     * @param control_p Roll control input
     * @param control_q Pitch control input
     * @param control_r Yaw control input
     * @param control_i Brake/drag control input
     * @return Aerodynamic coefficients in aerodynamic frame
     */
    virtual AeroCoefficients evaluate(
        double alpha,
        double beta,
        double mach,
        double control_p,
        double control_q,
        double control_r,
        double control_i
    ) = 0;
};

/**
 * Simple linear aerodynamic model (for testing/default)
 */
class SimpleAeroModel : public AeroModel {
private:
    double Cx0;      // Base drag coefficient
    double Cz_alpha; // Lift curve slope
    double Cm_alpha; // Pitch moment slope

public:
    SimpleAeroModel(double Cx0 = 0.5, double Cz_alpha = 5.0, double Cm_alpha = -2.0)
        : Cx0(Cx0), Cz_alpha(Cz_alpha), Cm_alpha(Cm_alpha) {}

    AeroCoefficients evaluate(
        double alpha,
        double beta,
        double mach,
        double control_p,
        double control_q,
        double control_r,
        double control_i
    ) override {
        AeroCoefficients coeffs;
        
        // Simple linear model
        coeffs.Cx = Cx0 + control_i * 0.5;  // Drag increases with brake
        coeffs.Cy = 0.0;  // No side force
        coeffs.Cz = Cz_alpha * alpha;  // Linear lift
        coeffs.Cmx = 0.0;  // No roll moment
        coeffs.Cmy = Cm_alpha * alpha;  // Pitch moment
        coeffs.Cmz = 0.0;  // No yaw moment
        
        return coeffs;
    }
};

/**
 * Compute aerodynamic forces and moments from coefficients
 * 
 * @param coeffs Aerodynamic coefficients
 * @param dynamic_pressure Dynamic pressure (0.5 * rho * V^2)
 * @param ref_area Reference area
 * @param ref_length Reference length (for moments)
 * @return Forces in aerodynamic frame, moments in aerodynamic frame
 */
inline std::pair<Vector3, Vector3> compute_aero_forces_moments(
    const AeroCoefficients& coeffs,
    double dynamic_pressure,
    double ref_area,
    double ref_length
) {
    // Forces in aerodynamic frame
    Vector3 force_A(
        -coeffs.Cx * dynamic_pressure * ref_area,  // Drag (negative X in aero frame)
        coeffs.Cy * dynamic_pressure * ref_area,
        -coeffs.Cz * dynamic_pressure * ref_area   // Lift (negative Z in aero frame)
    );
    
    // Moments in aerodynamic frame
    Vector3 moment_A(
        coeffs.Cmx * dynamic_pressure * ref_area * ref_length,
        coeffs.Cmy * dynamic_pressure * ref_area * ref_length,
        coeffs.Cmz * dynamic_pressure * ref_area * ref_length
    );
    
    return {force_A, moment_A};
}

} // namespace models
} // namespace trajectory

#endif // TRAJECTORY_AERO_MODEL_HPP
