#include <iomanip>
#include "CelestialBody.h"
#include "Satellite.h"
#include "Simulation.h"
#include "CSVExporter.h"
#include "OrbitalElements.h"
#include <iostream>
#include <memory>
#include <cmath>
#include <thread>
#include <vector>
#include <mutex>
#include <chrono>

int main() {
    using namespace orbit;

    std::cout << "=== Multi-Threaded Satellite Orbit Simulation ===\n";
    std::cout << "Hardware threads available: "
              << std::thread::hardware_concurrency() << "\n\n";

    auto earth = CelestialBody::Earth();
    double mu = earth.getMu();
    double Re = earth.getRadius();

    // Create a constellation of satellites at different altitudes
    struct SatConfig {
        std::string name;
        double altitude; // km
        double inclination; // radians
    };

    std::vector<SatConfig> satConfigs = {
        {"LEO-200km",  200e3,  0.0},
        {"LEO-400km",  400e3,  0.9},     // ~51.6 deg (ISS-like)
        {"LEO-600km",  600e3,  1.7},     // ~97.4 deg (sun-sync)
        {"LEO-800km",  800e3,  1.7},
        {"MEO-2000km", 2000e3, 0.96},    // ~55 deg (GPS-like)
        {"MEO-20200km",20200e3, 0.96},   // GPS altitude
        {"GEO-35786km",35786e3, 0.0},
        {"HEO-Molniya",500e3,  1.1},     // Molniya perigee
    };

    // Build satellites
    std::vector<std::shared_ptr<Satellite>> satellites;
    for (const auto& cfg : satConfigs) {
        double r = Re + cfg.altitude;
        double v = std::sqrt(mu / r);

        // Apply inclination via velocity components
        Vector3D pos(r, 0, 0);
        Vector3D vel(0, v * std::cos(cfg.inclination), v * std::sin(cfg.inclination));

        // Special case: Molniya (elliptical)
        if (cfg.name == "HEO-Molniya") {
            double apogee = 39873e3;
            double a = (r + Re + apogee) / 2.0;
            double vP = std::sqrt(mu * (2.0 / r - 1.0 / a));
            vel = {0, vP * std::cos(cfg.inclination), vP * std::sin(cfg.inclination)};
        }

        satellites.push_back(std::make_shared<Satellite>(cfg.name, pos, vel));
    }

    // Simulation config
    SimulationConfig config;
    config.dt = 10.0;
    config.totalTime = 86400.0;
    config.outputInterval = 6;
    config.method = "rk4";

    // --- Multi-threaded simulation ---
    GravityModel gravity(earth, true, false);

    std::vector<std::vector<TrajectoryPoint>> allTrajectories(satellites.size());
    std::mutex coutMutex;

    auto start = std::chrono::high_resolution_clock::now();

    // Launch threads
    std::vector<std::thread> threads;
    for (size_t i = 0; i < satellites.size(); ++i) {
        threads.emplace_back([&, i]() {
            // Each thread runs its own simulation
            Simulation sim(earth, config);
            sim.addSatellite(satellites[i]);
            sim.run();

            allTrajectories[i] = sim.getTrajectories()[0];

            std::lock_guard<std::mutex> lock(coutMutex);
            std::cout << "  [Thread " << i << "] " << satellites[i]->getName()
                      << " complete — " << allTrajectories[i].size()
                      << " data points\n";
        });
    }

    // Wait for all threads
    for (auto& t : threads) {
        t.join();
    }

    auto end = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);

    std::cout << "\nAll " << satellites.size() << " satellites simulated in "
              << duration.count() << " ms using "
              << satellites.size() << " threads.\n";

    // Export results
    for (size_t i = 0; i < satellites.size(); ++i) {
        std::string name = satellites[i]->getName();
        CSVExporter::exportTrajectory(allTrajectories[i],
            "data/" + name + "_trajectory.csv", name);
        CSVExporter::exportOrbitalElements(allTrajectories[i],
            "data/" + name + "_elements.csv", name);
    }

    CSVExporter::exportSummary(satellites, allTrajectories,
        "data/mt_simulation_summary.csv", mu, Re);

    // Print summary
    std::cout << "\n========================================\n";
    std::cout << "  CONSTELLATION SUMMARY\n";
    std::cout << "========================================\n\n";

    for (size_t i = 0; i < satellites.size(); ++i) {
        const auto& traj = allTrajectories[i];
        if (traj.empty()) continue;

        double minAlt = 1e20, maxAlt = -1e20;
        for (const auto& pt : traj) {
            if (pt.altitude < minAlt) minAlt = pt.altitude;
            if (pt.altitude > maxAlt) maxAlt = pt.altitude;
        }

        std::cout << satellites[i]->getName() << ":\n"
                  << "  Altitude: " << std::fixed << std::setprecision(1)
                  << minAlt / 1000.0 << " - " << maxAlt / 1000.0 << " km\n"
                  << "  Period:   " << traj.front().elements.period / 60.0 << " min\n"
                  << "  Ecc:      " << std::setprecision(6)
                  << traj.front().elements.eccentricity << "\n\n";
    }

    std::cout << "CSV files exported to data/ directory.\n";

    return 0;
}
