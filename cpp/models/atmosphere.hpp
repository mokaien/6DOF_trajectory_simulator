#ifndef TRAJECTORY_ATMOSPHERE_HPP
#define TRAJECTORY_ATMOSPHERE_HPP

#include "../math/vector.hpp"
#include <functional>
#include <vector>
#include <array>

namespace trajectory {
namespace models {

using namespace trajectory::math;

/**
 * Atmosphere properties
 */
struct AtmosphereState {
    double density;      // kg/m^3
    double pressure;     // Pa
    double temperature;  // K
    double speed_of_sound; // m/s
};

/**
 * Abstract interface for atmosphere models
 */
class AtmosphereModel {
public:
    virtual ~AtmosphereModel() = default;

    /**
     * Get atmosphere properties at given altitude
     * 
     * @param altitude Altitude above sea level (m)
     * @return Atmosphere state
     */
    virtual AtmosphereState get_atmosphere(double altitude) = 0;
};

/**
 * Simple exponential atmosphere model (ISA-like)
 * rho = rho0 * exp(-h / H)
 */
class ExponentialAtmosphere : public AtmosphereModel {
private:
    double rho0;  // Sea level density (kg/m^3)
    double H;     // Scale height (m)
    double T0;    // Sea level temperature (K)
    double P0;    // Sea level pressure (Pa)
    double gamma; // Specific heat ratio
    double R;     // Gas constant (J/(kg*K))

public:
    ExponentialAtmosphere(
        double rho0 = 1.225,
        double H = 8400.0,
        double T0 = 288.15,
        double P0 = 101325.0,
        double gamma = 1.4,
        double R = 287.0
    ) : rho0(rho0), H(H), T0(T0), P0(P0), gamma(gamma), R(R) {}

    AtmosphereState get_atmosphere(double altitude) override {
        AtmosphereState state;
        
        // Exponential density model
        state.density = rho0 * std::exp(-altitude / H);
        
        // Temperature (linear lapse rate)
        double lapse_rate = 0.0065;  // K/m
        state.temperature = T0 - lapse_rate * altitude;
        if (state.temperature < 216.65) {
            state.temperature = 216.65;  // Tropopause limit
        }
        
        // Pressure from ideal gas law
        state.pressure = state.density * R * state.temperature;
        
        // Speed of sound
        state.speed_of_sound = std::sqrt(gamma * R * state.temperature);
        
        return state;
    }
};

/**
 * Wind model interface
 */
class WindModel {
public:
    virtual ~WindModel() = default;

    /**
     * Get wind velocity in inertial frame at given position and time
     * 
     * @param pos_I Position in inertial frame
     * @param time Current time
     * @return Wind velocity vector in inertial frame
     */
    virtual Vector3 get_wind(const Vector3& pos_I, double time) = 0;
};

/**
 * Constant wind model
 */
class ConstantWind : public WindModel {
private:
    Vector3 wind_velocity;

public:
    ConstantWind(const Vector3& wind) : wind_velocity(wind) {}

    Vector3 get_wind(const Vector3& pos_I, double time) override {
        return wind_velocity;
    }
};

/**
 * No wind model
 */
class NoWind : public WindModel {
public:
    Vector3 get_wind(const Vector3& pos_I, double time) override {
        return Vector3(0, 0, 0);
    }
};

} // namespace models
} // namespace trajectory

#endif // TRAJECTORY_ATMOSPHERE_HPP
