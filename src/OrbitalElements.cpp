#include <algorithm>
#include "OrbitalElements.h"

namespace orbit {

OrbitalElements OrbitalMechanics::stateToElements(const Vector3D& position,
                                                    const Vector3D& velocity,
                                                    double mu) {
    OrbitalElements oe{};

    double r = position.magnitude();
    double v = velocity.magnitude();

    // Specific angular momentum
    Vector3D h = position.cross(velocity);
    double hMag = h.magnitude();

    // Node vector (z_hat x h)
    Vector3D n = Vector3D(0, 0, 1).cross(h);
    double nMag = n.magnitude();

    // Eccentricity vector
    Vector3D eVec = ((v * v - mu / r) * position - position.dot(velocity) * velocity) / mu;
    oe.eccentricity = eVec.magnitude();

    // Specific energy
    oe.specificEnergy = (v * v / 2.0) - (mu / r);

    // Semi-major axis
    if (std::abs(oe.eccentricity - 1.0) > 1e-10) {
        oe.semiMajorAxis = -mu / (2.0 * oe.specificEnergy);
    } else {
        oe.semiMajorAxis = std::numeric_limits<double>::infinity(); // parabolic
    }

    // Inclination
    oe.inclination = std::acos(std::clamp(h.z / hMag, -1.0, 1.0));

    // RAAN (Right Ascension of Ascending Node)
    if (nMag > 1e-10) {
        oe.raan = std::acos(std::clamp(n.x / nMag, -1.0, 1.0));
        if (n.y < 0) oe.raan = 2.0 * M_PI - oe.raan;
    } else {
        oe.raan = 0.0;
    }

    // Argument of periapsis
    if (nMag > 1e-10 && oe.eccentricity > 1e-10) {
        oe.argOfPeriapsis = std::acos(
            std::clamp(n.dot(eVec) / (nMag * oe.eccentricity), -1.0, 1.0));
        if (eVec.z < 0) oe.argOfPeriapsis = 2.0 * M_PI - oe.argOfPeriapsis;
    } else {
        oe.argOfPeriapsis = 0.0;
    }

    // True anomaly
    if (oe.eccentricity > 1e-10) {
        oe.trueAnomaly = std::acos(
            std::clamp(eVec.dot(position) / (oe.eccentricity * r), -1.0, 1.0));
        if (position.dot(velocity) < 0) {
            oe.trueAnomaly = 2.0 * M_PI - oe.trueAnomaly;
        }
    } else {
        oe.trueAnomaly = 0.0;
    }

    // Period
    oe.period = computePeriod(oe.semiMajorAxis, mu);

    // Apoapsis and periapsis
    oe.apoapsis = oe.semiMajorAxis * (1.0 + oe.eccentricity);
    oe.periapsis = oe.semiMajorAxis * (1.0 - oe.eccentricity);

    return oe;
}

void OrbitalMechanics::elementsToState(const OrbitalElements& elements,
                                        double mu,
                                        Vector3D& position,
                                        Vector3D& velocity) {
    double a = elements.semiMajorAxis;
    double e = elements.eccentricity;
    double i = elements.inclination;
    double omega = elements.raan;
    double w = elements.argOfPeriapsis;
    double nu = elements.trueAnomaly;

    // Semi-latus rectum
    double p = a * (1.0 - e * e);

    // Position in perifocal frame
    double rMag = p / (1.0 + e * std::cos(nu));
    double cosNu = std::cos(nu);
    double sinNu = std::sin(nu);

    Vector3D rPF = {rMag * cosNu, rMag * sinNu, 0.0};
    double sqrtMuP = std::sqrt(mu / p);
    Vector3D vPF = {-sqrtMuP * sinNu, sqrtMuP * (e + cosNu), 0.0};

    // Rotation matrices (perifocal -> ECI)
    double cosO = std::cos(omega), sinO = std::sin(omega);
    double cosW = std::cos(w), sinW = std::sin(w);
    double cosI = std::cos(i), sinI = std::sin(i);

    auto rotate = [&](const Vector3D& v) -> Vector3D {
        double x = (cosO * cosW - sinO * sinW * cosI) * v.x +
                   (-cosO * sinW - sinO * cosW * cosI) * v.y;
        double y = (sinO * cosW + cosO * sinW * cosI) * v.x +
                   (-sinO * sinW + cosO * cosW * cosI) * v.y;
        double z = (sinW * sinI) * v.x + (cosW * sinI) * v.y;
        return {x, y, z};
    };

    position = rotate(rPF);
    velocity = rotate(vPF);
}

double OrbitalMechanics::computePeriod(double semiMajorAxis, double mu) {
    if (semiMajorAxis <= 0) return 0.0;
    return 2.0 * M_PI * std::sqrt(semiMajorAxis * semiMajorAxis * semiMajorAxis / mu);
}

double OrbitalMechanics::computeEnergy(const Vector3D& position,
                                         const Vector3D& velocity,
                                         double mu) {
    return 0.5 * velocity.magnitudeSquared() - mu / position.magnitude();
}

} // namespace orbit
