#pragma once
#include "Simulation.h"
#include <string>

namespace orbit {

class CSVExporter {
public:
    static void exportTrajectory(const std::vector<TrajectoryPoint>& trajectory,
                                  const std::string& filename,
                                  const std::string& satelliteName);

    static void exportOrbitalElements(const std::vector<TrajectoryPoint>& trajectory,
                                       const std::string& filename,
                                       const std::string& satelliteName);

    static void exportSummary(const std::vector<std::shared_ptr<Satellite>>& satellites,
                               const std::vector<std::vector<TrajectoryPoint>>& trajectories,
                               const std::string& filename,
                               double mu, double bodyRadius);
};

} // namespace orbit
