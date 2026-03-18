#pragma once
#include "Vector3D.h"
#include <cmath>

namespace orbit {

struct OrbitalElements {
    double semiMajorAxis;      // a (meters)
    double eccentricity;       // e (dimensionless)
    double inclination;        // i (radians)
    double raan;               // Ω — right ascension of ascending node (radians)
    double argOfPeriapsis;     // ω (radians)
    double trueAnomaly;        // ν (radians)
    double period;             // T (seconds)
    double specificEnergy;     // ε (J/kg)
    double apoapsis;           // highest point (meters from center)
    double periapsis;          // lowest point (meters from center)
};

class OrbitalMechanics {
public:
    // Convert state vector to Keplerian elements
    static OrbitalElements stateToElements(const Vector3D& position,
                                            const Vector3D& velocity,
                                            double mu);

    // Convert Keplerian elements to state vector
    static void elementsToState(const OrbitalElements& elements,
                                double mu,
                                Vector3D& position,
                                Vector3D& velocity);

    // Compute orbital period from semi-major axis
    static double computePeriod(double semiMajorAxis, double mu);

    // Compute specific orbital energy
    static double computeEnergy(const Vector3D& position,
                                 const Vector3D& velocity,
                                 double mu);
};

} // namespace orbit
