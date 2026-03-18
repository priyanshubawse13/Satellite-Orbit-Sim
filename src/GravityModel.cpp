#include "GravityModel.h"
#include <cmath>

namespace orbit {

GravityModel::GravityModel(const CelestialBody& body, bool enableJ2, bool enableDrag)
    : mu_(body.getMu()),
      bodyRadius_(body.getRadius()),
      enableJ2_(enableJ2),
      enableDrag_(enableDrag) {}

Vector3D GravityModel::computeAcceleration(const SatelliteState& state) const {
    Vector3D totalAccel = pointMassGravity(state.position);

    if (enableJ2_) {
        totalAccel += j2Perturbation(state.position);
    }

    if (enableDrag_) {
        totalAccel += atmosphericDrag(state);
    }

    return totalAccel;
}

Vector3D GravityModel::pointMassGravity(const Vector3D& position) const {
    double r = position.magnitude();
    double r3 = r * r * r;
    // a = -mu / r^3 * r_vec
    return position * (-mu_ / r3);
}

Vector3D GravityModel::j2Perturbation(const Vector3D& position) const {
    double r = position.magnitude();
    double r2 = r * r;
    double r5 = r2 * r2 * r;
    double Re2 = bodyRadius_ * bodyRadius_;
    double z2_r2 = (position.z * position.z) / r2;

    double factor = -1.5 * J2 * mu_ * Re2 / r5;

    double ax = factor * position.x * (1.0 - 5.0 * z2_r2);
    double ay = factor * position.y * (1.0 - 5.0 * z2_r2);
    double az = factor * position.z * (3.0 - 5.0 * z2_r2);

    return {ax, ay, az};
}

Vector3D GravityModel::atmosphericDrag(const SatelliteState& state) const {
    double alt = state.position.magnitude() - bodyRadius_;
    if (alt > 1000e3 || alt < 0) return {0.0, 0.0, 0.0}; // No drag above 1000km

    double rho = atmosphericDensity(alt);
    double v = state.velocity.magnitude();
    if (v < 1e-10) return {0.0, 0.0, 0.0};

    // Simplified: assume Cd=2.2, A=1.0 m^2, m=100 kg
    double Cd = 2.2;
    double A = 1.0;
    double m = 100.0;

    // a_drag = -0.5 * (Cd * A / m) * rho * v * v_hat
    double dragMag = 0.5 * (Cd * A / m) * rho * v * v;
    Vector3D vHat = state.velocity.normalized();

    return vHat * (-dragMag);
}

double GravityModel::atmosphericDensity(double altitude) const {
    // Exponential atmosphere model
    // rho = rho0 * exp(-altitude / scaleHeight)
    // Using approximate values for Earth
    struct AtmLayer {
        double altBase;   // meters
        double rhoBase;   // kg/m^3
        double scaleH;    // meters
    };

    // Simplified multi-layer model
    static const AtmLayer layers[] = {
        {0.0,     1.225,     8500.0},
        {100e3,   5.297e-7,  27000.0},
        {200e3,   2.789e-10, 37000.0},
        {300e3,   7.248e-12, 45000.0},
        {400e3,   3.725e-13, 53000.0},
        {500e3,   2.418e-14, 62000.0},
        {600e3,   1.454e-15, 72000.0},
        {700e3,   7.615e-17, 83000.0},
        {800e3,   3.614e-18, 95000.0},
    };

    // Find the right layer
    const AtmLayer* layer = &layers[0];
    for (const auto& l : layers) {
        if (altitude >= l.altBase) layer = &l;
        else break;
    }

    return layer->rhoBase * std::exp(-(altitude - layer->altBase) / layer->scaleH);
}

void GravityModel::setDragEnabled(bool enabled) { enableDrag_ = enabled; }
void GravityModel::setJ2Enabled(bool enabled) { enableJ2_ = enabled; }

} // namespace orbit
