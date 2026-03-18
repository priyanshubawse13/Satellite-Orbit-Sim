#include "CelestialBody.h"

namespace orbit {

CelestialBody::CelestialBody(std::string name, double mass, double radius)
    : name_(std::move(name)), mass_(mass), radius_(radius), mu_(G * mass) {}

const std::string& CelestialBody::getName() const { return name_; }
double CelestialBody::getMass() const { return mass_; }
double CelestialBody::getRadius() const { return radius_; }
double CelestialBody::getMu() const { return mu_; }

CelestialBody CelestialBody::Earth() {
    return {"Earth", 5.972e24, 6.371e6};
}

CelestialBody CelestialBody::Moon() {
    return {"Moon", 7.342e22, 1.737e6};
}

CelestialBody CelestialBody::Mars() {
    return {"Mars", 6.417e23, 3.3895e6};
}

} // namespace orbit
