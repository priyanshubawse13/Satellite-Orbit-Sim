#include "Simulation.h"
#include <iostream>
#include <iomanip>
#include <algorithm>

namespace orbit {

Simulation::Simulation(const CelestialBody& body, SimulationConfig config)
    : body_(body),
      gravity_(body, true, false),
      config_(std::move(config)) {}

void Simulation::addSatellite(std::shared_ptr<Satellite> sat) {
    satellites_.push_back(std::move(sat));
}

void Simulation::run() {
    trajectories_.clear();
    trajectories_.reserve(satellites_.size());

    for (auto& sat : satellites_) {
        trajectories_.push_back(runSingle(sat));
    }
}

std::vector<TrajectoryPoint> Simulation::runSingle(std::shared_ptr<Satellite> sat) {
    std::vector<TrajectoryPoint> trajectory;
    int totalSteps = static_cast<int>(config_.totalTime / config_.dt);
    trajectory.reserve(totalSteps / config_.outputInterval + 1);

    SatelliteState state = sat->getState();

    // Acceleration function (captures gravity model)
    AccelerationFunc accelFunc = [this](const SatelliteState& s) {
        return gravity_.computeAcceleration(s);
    };

    // Select integrator
    auto integrate = [&](const SatelliteState& s, double dt) -> SatelliteState {
        if (config_.method == "euler") return Integrator::euler(s, accelFunc, dt);
        if (config_.method == "rkf45") return Integrator::rkf45(s, accelFunc, dt);
        return Integrator::rk4(s, accelFunc, dt); // default
    };

    // Record initial state
    OrbitalElements oe = OrbitalMechanics::stateToElements(
        state.position, state.velocity, body_.getMu());
    trajectory.push_back({
        state.time, state.position, state.velocity,
        state.position.magnitude() - body_.getRadius(),
        state.velocity.magnitude(), oe
    });

    // Main loop
    for (int step = 1; step <= totalSteps; ++step) {
        state = integrate(state, config_.dt);

        if (step % config_.outputInterval == 0) {
            oe = OrbitalMechanics::stateToElements(
                state.position, state.velocity, body_.getMu());
            trajectory.push_back({
                state.time, state.position, state.velocity,
                state.position.magnitude() - body_.getRadius(),
                state.velocity.magnitude(), oe
            });
        }

        // Check for crash
        if (state.position.magnitude() < body_.getRadius()) {
            std::cout << "[WARNING] " << sat->getName()
                      << " has impacted the surface at t="
                      << state.time << "s\n";
            break;
        }
    }

    // Update satellite with final state
    sat->setState(state);

    return trajectory;
}

const std::vector<std::vector<TrajectoryPoint>>& Simulation::getTrajectories() const {
    return trajectories_;
}

const std::vector<std::shared_ptr<Satellite>>& Simulation::getSatellites() const {
    return satellites_;
}

void Simulation::printSummary() const {
    std::cout << "\n========================================\n";
    std::cout << "  SIMULATION SUMMARY\n";
    std::cout << "  Central Body: " << body_.getName() << "\n";
    std::cout << "  Method: " << config_.method << "\n";
    std::cout << "  dt = " << config_.dt << "s, Total = "
              << config_.totalTime << "s\n";
    std::cout << "========================================\n\n";

    for (size_t i = 0; i < satellites_.size(); ++i) {
        const auto& sat = satellites_[i];
        const auto& traj = trajectories_[i];

        if (traj.empty()) continue;

        const auto& first = traj.front();
        const auto& last = traj.back();

        // Find min/max altitude
        double minAlt = 1e20, maxAlt = -1e20;
        for (const auto& pt : traj) {
            minAlt = std::min(minAlt, pt.altitude);
            maxAlt = std::max(maxAlt, pt.altitude);
        }

        std::cout << "--- " << sat->getName() << " ---\n";
        std::cout << std::fixed << std::setprecision(2);
        std::cout << "  Initial altitude: " << first.altitude / 1000.0 << " km\n";
        std::cout << "  Final altitude:   " << last.altitude / 1000.0 << " km\n";
        std::cout << "  Min altitude:     " << minAlt / 1000.0 << " km\n";
        std::cout << "  Max altitude:     " << maxAlt / 1000.0 << " km\n";
        std::cout << "  Initial speed:    " << first.speed << " m/s\n";
        std::cout << "  Final speed:      " << last.speed << " m/s\n";
        std::cout << "  Eccentricity:     " << std::setprecision(6) << first.elements.eccentricity << "\n";
        std::cout << "  Semi-major axis:  " << std::setprecision(2) << first.elements.semiMajorAxis / 1000.0 << " km\n";
        std::cout << "  Period:           " << first.elements.period / 60.0 << " min\n";
        std::cout << "  Data points:      " << traj.size() << "\n\n";
    }
}

} // namespace orbit
