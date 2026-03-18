#pragma once
#include "Satellite.h"
#include "GravityModel.h"
#include <functional>

namespace orbit {

// Function type: given state, return acceleration
using AccelerationFunc = std::function<Vector3D(const SatelliteState&)>;

class Integrator {
public:
    // Euler method (1st order — baseline)
    static SatelliteState euler(const SatelliteState& state,
                                const AccelerationFunc& accel,
                                double dt);

    // Classical 4th-order Runge-Kutta
    static SatelliteState rk4(const SatelliteState& state,
                              const AccelerationFunc& accel,
                              double dt);

    // Runge-Kutta-Fehlberg (RK45) with adaptive step size
    static SatelliteState rkf45(const SatelliteState& state,
                                const AccelerationFunc& accel,
                                double dt,
                                double tolerance = 1e-9);
};

} // namespace orbit
