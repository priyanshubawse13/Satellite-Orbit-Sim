#include "CelestialBody.h"
#include "Satellite.h"
#include "Simulation.h"
#include "CSVExporter.h"
#include "OrbitalElements.h"
#include <iostream>
#include <memory>
#include <cmath>

int main() {
    using namespace orbit;

    std::cout << "=== Satellite Orbit Simulation Tool ===\n\n";

    // Central body
    auto earth = CelestialBody::Earth();
    double mu = earth.getMu();
    double Re = earth.getRadius();

    // --- Satellite 1: ISS-like (circular LEO) ---
    double altISS = 408e3; // 408 km
    double rISS = Re + altISS;
    double vISS = std::sqrt(mu / rISS); // circular velocity

    auto iss = std::make_shared<Satellite>(
        "ISS", Vector3D(rISS, 0, 0), Vector3D(0, vISS, 0));

    // --- Satellite 2: Elliptical orbit (Molniya-like) ---
    double perigee = 500e3;
    double apogee = 39873e3;
    double rP = Re + perigee;
    double a = (rP + Re + apogee) / 2.0;
    double vP = std::sqrt(mu * (2.0 / rP - 1.0 / a)); // vis-viva

    auto molniya = std::make_shared<Satellite>(
        "Molniya", Vector3D(rP, 0, 0), Vector3D(0, vP * std::cos(0.1), vP * std::sin(0.1)));

    // --- Satellite 3: GEO ---
    double altGEO = 35786e3;
    double rGEO = Re + altGEO;
    double vGEO = std::sqrt(mu / rGEO);

    auto geo = std::make_shared<Satellite>(
        "GEO", Vector3D(rGEO, 0, 0), Vector3D(0, vGEO, 0));

    // Configuration
    SimulationConfig config;
    config.dt = 10.0;                  // 10 second time step
    config.totalTime = 86400.0;        // 24 hours
    config.outputInterval = 6;         // record every 60 seconds
    config.method = "rk4";

    // Run simulation
    Simulation sim(earth, config);
    sim.addSatellite(iss);
    sim.addSatellite(molniya);
    sim.addSatellite(geo);

    std::cout << "Running simulation...\n";
    sim.run();
    sim.printSummary();

    // Export data
    const auto& trajectories = sim.getTrajectories();
    const auto& satellites = sim.getSatellites();

    for (size_t i = 0; i < satellites.size(); ++i) {
        std::string name = satellites[i]->getName();
        CSVExporter::exportTrajectory(trajectories[i],
            "data/" + name + "_trajectory.csv", name);
        CSVExporter::exportOrbitalElements(trajectories[i],
            "data/" + name + "_elements.csv", name);
    }

    CSVExporter::exportSummary(satellites, trajectories,
        "data/simulation_summary.csv", mu, Re);

    std::cout << "\nSimulation complete! Check the data/ directory for CSV outputs.\n";

    return 0;
}
