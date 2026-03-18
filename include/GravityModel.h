#pragma once
#include "Vector3D.h"
#include "CelestialBody.h"
#include "Satellite.h"

namespace orbit {

class GravityModel {
public:
    explicit GravityModel(const CelestialBody& body, bool enableJ2 = true,
                          bool enableDrag = false);

    // Compute total acceleration on satellite
    Vector3D computeAcceleration(const SatelliteState& state) const;

    // Individual force components
    Vector3D pointMassGravity(const Vector3D& position) const;
    Vector3D j2Perturbation(const Vector3D& position) const;
    Vector3D atmosphericDrag(const SatelliteState& state) const;

    void setDragEnabled(bool enabled);
    void setJ2Enabled(bool enabled);

private:
    double mu_;
    double bodyRadius_;
    bool enableJ2_;
    bool enableDrag_;

    // J2 coefficient for Earth oblateness
    static constexpr double J2 = 1.08263e-3;

    // Simple exponential atmosphere model
    double atmosphericDensity(double altitude) const;
};

} // namespace orbit
