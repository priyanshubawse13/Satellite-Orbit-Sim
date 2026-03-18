#include "Integrator.h"

namespace orbit {

SatelliteState Integrator::euler(const SatelliteState& state,
                                  const AccelerationFunc& accel,
                                  double dt) {
    Vector3D a = accel(state);
    return {
        state.position + state.velocity * dt,
        state.velocity + a * dt,
        state.time + dt
    };
}

SatelliteState Integrator::rk4(const SatelliteState& state,
                                const AccelerationFunc& accel,
                                double dt) {
    // k1
    Vector3D a1 = accel(state);
    Vector3D v1 = state.velocity;

    // k2
    SatelliteState s2 = {
        state.position + v1 * (dt / 2.0),
        state.velocity + a1 * (dt / 2.0),
        state.time + dt / 2.0
    };
    Vector3D a2 = accel(s2);
    Vector3D v2 = s2.velocity;

    // k3
    SatelliteState s3 = {
        state.position + v2 * (dt / 2.0),
        state.velocity + a2 * (dt / 2.0),
        state.time + dt / 2.0
    };
    Vector3D a3 = accel(s3);
    Vector3D v3 = s3.velocity;

    // k4
    SatelliteState s4 = {
        state.position + v3 * dt,
        state.velocity + a3 * dt,
        state.time + dt
    };
    Vector3D a4 = accel(s4);
    Vector3D v4 = s4.velocity;

    // Combine
    Vector3D newPos = state.position +
        (v1 + 2.0 * v2 + 2.0 * v3 + v4) * (dt / 6.0);
    Vector3D newVel = state.velocity +
        (a1 + 2.0 * a2 + 2.0 * a3 + a4) * (dt / 6.0);

    return {newPos, newVel, state.time + dt};
}

SatelliteState Integrator::rkf45(const SatelliteState& state,
                                  const AccelerationFunc& accel,
                                  double dt,
                                  double tolerance) {
    // Runge-Kutta-Fehlberg coefficients
    // For simplicity, use RK4 with half-step error estimation
    SatelliteState full = rk4(state, accel, dt);

    // Two half steps
    SatelliteState half1 = rk4(state, accel, dt / 2.0);
    SatelliteState half2 = rk4(half1, accel, dt / 2.0);

    // Error estimate (Richardson extrapolation)
    double error = (half2.position - full.position).magnitude();

    if (error > tolerance && dt > 1.0) {
        // Recursively try smaller step
        SatelliteState mid = rkf45(state, accel, dt / 2.0, tolerance);
        return rkf45(mid, accel, dt / 2.0, tolerance);
    }

    // Use the more accurate half-step result
    return half2;
}

} // namespace orbit
