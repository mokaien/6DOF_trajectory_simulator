#ifndef TRAJECTORY_THRUST_HPP
#define TRAJECTORY_THRUST_HPP

#include "../math/vector.hpp"
#include <vector>
#include <algorithm>
#include <stdexcept>

namespace trajectory {
namespace models {

using namespace trajectory::math;

/**
 * Thrust model interface
 */
class ThrustModel {
public:
    virtual ~ThrustModel() = default;

    /**
     * Get thrust force in body frame at given time
     * 
     * @param time Current time
     * @return Thrust vector in body frame (typically +X direction)
     */
    virtual Vector3 get_thrust(double time) = 0;

    /**
     * Get mass flow rate (negative for mass loss)
     * 
     * @param time Current time
     * @return Mass flow rate (kg/s)
     */
    virtual double get_mass_flow_rate(double time) = 0;

    /**
     * Check if thrust is active at given time
     * 
     * @param time Current time
     * @return True if thrust is active
     */
    virtual bool is_active(double time) = 0;
};

/**
 * No thrust model
 */
class NoThrust : public ThrustModel {
public:
    Vector3 get_thrust(double time) override {
        return Vector3(0, 0, 0);
    }

    double get_mass_flow_rate(double time) override {
        return 0.0;
    }

    bool is_active(double time) override {
        return false;
    }
};

/**
 * Constant thrust model
 */
class ConstantThrust : public ThrustModel {
private:
    Vector3 thrust_vector;
    double mass_flow;

public:
    ConstantThrust(const Vector3& thrust, double mass_flow = 0.0)
        : thrust_vector(thrust), mass_flow(mass_flow) {}

    Vector3 get_thrust(double time) override {
        return thrust_vector;
    }

    double get_mass_flow_rate(double time) override {
        return mass_flow;
    }

    bool is_active(double time) override {
        return true;
    }
};

/**
 * Time-varying thrust from CSV data (interpolated)
 */
class CSVThrust : public ThrustModel {
private:
    std::vector<double> time_points;
    std::vector<double> thrust_magnitudes;
    std::vector<double> mass_flow_rates;
    Vector3 thrust_direction;
    double burn_duration;

    double interpolate(double t, const std::vector<double>& times, const std::vector<double>& values) {
        if (times.empty()) return 0.0;
        if (t <= times[0]) return values[0];
        if (t >= times.back()) return values.back();

        // Find interpolation interval
        auto it = std::lower_bound(times.begin(), times.end(), t);
        size_t idx = std::distance(times.begin(), it);
        
        if (idx == 0) return values[0];
        if (idx >= times.size()) return values.back();

        // Linear interpolation
        double t0 = times[idx - 1];
        double t1 = times[idx];
        double v0 = values[idx - 1];
        double v1 = values[idx];
        
        return v0 + (v1 - v0) * (t - t0) / (t1 - t0);
    }

public:
    CSVThrust(
        const std::vector<double>& times,
        const std::vector<double>& thrusts,
        const std::vector<double>& mass_flows,
        const Vector3& direction = Vector3(1, 0, 0),
        double duration = 0.0
    ) : time_points(times), thrust_magnitudes(thrusts), 
        mass_flow_rates(mass_flows), thrust_direction(direction.normalized()),
        burn_duration(duration > 0 ? duration : (times.empty() ? 0.0 : times.back()))
    {
        if (times.size() != thrusts.size() || times.size() != mass_flows.size()) {
            throw std::invalid_argument("Time, thrust, and mass flow vectors must have same size");
        }
    }

    Vector3 get_thrust(double time) override {
        if (time < 0 || time > burn_duration) {
            return Vector3(0, 0, 0);
        }
        double magnitude = interpolate(time, time_points, thrust_magnitudes);
        return thrust_direction * magnitude;
    }

    double get_mass_flow_rate(double time) override {
        if (time < 0 || time > burn_duration) {
            return 0.0;
        }
        return -interpolate(time, time_points, mass_flow_rates);  // Negative for mass loss
    }

    bool is_active(double time) override {
        return time >= 0 && time <= burn_duration;
    }
};

} // namespace models
} // namespace trajectory

#endif // TRAJECTORY_THRUST_HPP
