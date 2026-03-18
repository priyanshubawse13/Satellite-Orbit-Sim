#pragma once
#include "Satellite.h"
#include "GravityModel.h"
#include "Integrator.h"
#include "OrbitalElements.h"
#include <vector>
#include <memory>
#include <string>
#include <functional>

namespace orbit {

struct SimulationConfig {
    double dt = 10.0;              // time step (seconds)
    double totalTime = 86400.0;    // total sim time (seconds) — default 1 day
    int outputInterval = 1;        // record every N steps
    std::string method = "rk4";    // "euler", "rk4", "rkf45"
};

struct TrajectoryPoint {
    double time;
    Vector3D position;
    Vector3D velocity;
    double altitude;
    double speed;
    OrbitalElements elements;
};

class Simulation {
public:
    Simulation(const CelestialBody& body, SimulationConfig config);

    // Add satellites to simulation
    void addSatellite(std::shared_ptr<Satellite> sat);

    // Run simulation for all satellites
    void run();

    // Get results
    const std::vector<std::vector<TrajectoryPoint>>& getTrajectories() const;
    const std::vector<std::shared_ptr<Satellite>>& getSatellites() const;

    // Print summary
    void printSummary() const;

private:
    CelestialBody body_;
    GravityModel gravity_;
    SimulationConfig config_;
    std::vector<std::shared_ptr<Satellite>> satellites_;
    std::vector<std::vector<TrajectoryPoint>> trajectories_;

    // Run single satellite
    std::vector<TrajectoryPoint> runSingle(std::shared_ptr<Satellite> sat);
};

} // namespace orbit
