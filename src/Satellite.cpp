#include "Satellite.h"

namespace orbit {

Satellite::Satellite(std::string name, Vector3D position, Vector3D velocity,
                     double mass, double dragCoeff, double crossSection)
    : name_(std::move(name)),
      state_{position, velocity, 0.0},
      mass_(mass),
      dragCoeff_(dragCoeff),
      crossSection_(crossSection) {}

const std::string& Satellite::getName() const { return name_; }
const SatelliteState& Satellite::getState() const { return state_; }
void Satellite::setState(const SatelliteState& state) { state_ = state; }

double Satellite::getMass() const { return mass_; }
double Satellite::getDragCoefficient() const { return dragCoeff_; }
double Satellite::getCrossSection() const { return crossSection_; }

double Satellite::speed() const {
    return state_.velocity.magnitude();
}

double Satellite::altitude(double bodyRadius) const {
    return state_.position.magnitude() - bodyRadius;
}

} // namespace orbit
