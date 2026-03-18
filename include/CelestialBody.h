#pragma once
#include "Vector3D.h"
#include <string>

namespace orbit {

class CelestialBody {
public:
    CelestialBody(std::string name, double mass, double radius);

    const std::string& getName() const;
    double getMass() const;
    double getRadius() const;
    double getMu() const; // gravitational parameter (G * M)

    // Factory for common bodies
    static CelestialBody Earth();
    static CelestialBody Moon();
    static CelestialBody Mars();

private:
    std::string name_;
    double mass_;
    double radius_;
    double mu_; // precomputed G * M

    static constexpr double G = 6.67430e-11; // m^3 kg^-1 s^-2
};

} // namespace orbit
