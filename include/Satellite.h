#pragma once
#include "Vector3D.h"
#include <string>

namespace orbit {

struct SatelliteState {
    Vector3D position;   // meters
    Vector3D velocity;   // m/s
    double time;         // seconds since epoch
};

class Satellite {
public:
    Satellite(std::string name, Vector3D position, Vector3D velocity,
              double mass = 100.0, double dragCoeff = 2.2, double crossSection = 1.0);

    const std::string& getName() const;
    const SatelliteState& getState() const;
    void setState(const SatelliteState& state);

    double getMass() const;
    double getDragCoefficient() const;
    double getCrossSection() const;

    // Compute orbital speed
    double speed() const;
    // Compute altitude above a body of given radius
    double altitude(double bodyRadius) const;

private:
    std::string name_;
    SatelliteState state_;
    double mass_;          // kg
    double dragCoeff_;     // dimensionless
    double crossSection_;  // m^2
};

} // namespace orbit
