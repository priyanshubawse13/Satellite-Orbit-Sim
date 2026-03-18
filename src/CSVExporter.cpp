#include "CSVExporter.h"
#include <fstream>
#include <iomanip>
#include <iostream>

namespace orbit {

void CSVExporter::exportTrajectory(const std::vector<TrajectoryPoint>& trajectory,
                                    const std::string& filename,
                                    const std::string& satelliteName) {
    std::ofstream file(filename);
    if (!file.is_open()) {
        std::cerr << "Error: Could not open " << filename << "\n";
        return;
    }

    file << "satellite,time_s,pos_x_m,pos_y_m,pos_z_m,vel_x_ms,vel_y_ms,vel_z_ms,altitude_km,speed_ms\n";
    file << std::fixed << std::setprecision(6);

    for (const auto& pt : trajectory) {
        file << satelliteName << ","
             << pt.time << ","
             << pt.position.x << "," << pt.position.y << "," << pt.position.z << ","
             << pt.velocity.x << "," << pt.velocity.y << "," << pt.velocity.z << ","
             << pt.altitude / 1000.0 << ","
             << pt.speed << "\n";
    }

    std::cout << "Exported trajectory to " << filename
              << " (" << trajectory.size() << " points)\n";
}

void CSVExporter::exportOrbitalElements(const std::vector<TrajectoryPoint>& trajectory,
                                          const std::string& filename,
                                          const std::string& satelliteName) {
    std::ofstream file(filename);
    if (!file.is_open()) {
        std::cerr << "Error: Could not open " << filename << "\n";
        return;
    }

    file << "satellite,time_s,semi_major_axis_km,eccentricity,inclination_deg,"
         << "raan_deg,arg_periapsis_deg,true_anomaly_deg,period_min,energy_Jkg\n";
    file << std::fixed << std::setprecision(6);

    constexpr double RAD2DEG = 180.0 / M_PI;

    for (const auto& pt : trajectory) {
        const auto& oe = pt.elements;
        file << satelliteName << ","
             << pt.time << ","
             << oe.semiMajorAxis / 1000.0 << ","
             << oe.eccentricity << ","
             << oe.inclination * RAD2DEG << ","
             << oe.raan * RAD2DEG << ","
             << oe.argOfPeriapsis * RAD2DEG << ","
             << oe.trueAnomaly * RAD2DEG << ","
             << oe.period / 60.0 << ","
             << oe.specificEnergy << "\n";
    }

    std::cout << "Exported orbital elements to " << filename << "\n";
}

void CSVExporter::exportSummary(const std::vector<std::shared_ptr<Satellite>>& satellites,
                                 const std::vector<std::vector<TrajectoryPoint>>& trajectories,
                                 const std::string& filename,
                                 double mu, double bodyRadius) {
    std::ofstream file(filename);
    if (!file.is_open()) return;

    file << "satellite,initial_alt_km,final_alt_km,min_alt_km,max_alt_km,"
         << "eccentricity,semi_major_axis_km,period_min\n";
    file << std::fixed << std::setprecision(4);

    for (size_t i = 0; i < satellites.size(); ++i) {
        const auto& traj = trajectories[i];
        if (traj.empty()) continue;

        double minAlt = 1e20, maxAlt = -1e20;
        for (const auto& pt : traj) {
            if (pt.altitude < minAlt) minAlt = pt.altitude;
            if (pt.altitude > maxAlt) maxAlt = pt.altitude;
        }

        file << satellites[i]->getName() << ","
             << traj.front().altitude / 1000.0 << ","
             << traj.back().altitude / 1000.0 << ","
             << minAlt / 1000.0 << ","
             << maxAlt / 1000.0 << ","
             << traj.front().elements.eccentricity << ","
             << traj.front().elements.semiMajorAxis / 1000.0 << ","
             << traj.front().elements.period / 60.0 << "\n";
    }

    std::cout << "Exported summary to " << filename << "\n";
}

} // namespace orbit
