#include "Vector3D.h"
#include "CelestialBody.h"
#include "Satellite.h"
#include "GravityModel.h"
#include "Integrator.h"
#include "OrbitalElements.h"
#include <iostream>
#include <cassert>
#include <cmath>

using namespace orbit;

constexpr double TOLERANCE = 1e-6;

bool approxEqual(double a, double b, double tol = TOLERANCE) {
    return std::abs(a - b) < tol;
}

void testVector3D() {
    std::cout << "Testing Vector3D... ";

    Vector3D a(1, 2, 3);
    Vector3D b(4, 5, 6);

    // Addition
    auto c = a + b;
    assert(approxEqual(c.x, 5) && approxEqual(c.y, 7) && approxEqual(c.z, 9));

    // Subtraction
    auto d = b - a;
    assert(approxEqual(d.x, 3) && approxEqual(d.y, 3) && approxEqual(d.z, 3));

    // Scalar multiply
    auto e = a * 2.0;
    assert(approxEqual(e.x, 2) && approxEqual(e.y, 4) && approxEqual(e.z, 6));

    // Magnitude
    Vector3D v(3, 4, 0);
    assert(approxEqual(v.magnitude(), 5.0));

    // Dot product
    assert(approxEqual(a.dot(b), 32.0));

    // Cross product
    Vector3D i(1, 0, 0), j(0, 1, 0);
    auto k = i.cross(j);
    assert(approxEqual(k.x, 0) && approxEqual(k.y, 0) && approxEqual(k.z, 1));

    // Normalized
    auto norm = v.normalized();
    assert(approxEqual(norm.magnitude(), 1.0));

    std::cout << "PASSED\n";
}

void testCelestialBody() {
    std::cout << "Testing CelestialBody... ";

    auto earth = CelestialBody::Earth();
    assert(earth.getName() == "Earth");
    assert(earth.getMass() > 5.9e24);
    assert(earth.getRadius() > 6.3e6);
    assert(earth.getMu() > 3.9e14);

    std::cout << "PASSED\n";
}

void testCircularOrbit() {
    std::cout << "Testing circular orbit conservation... ";

    auto earth = CelestialBody::Earth();
    double mu = earth.getMu();
    double Re = earth.getRadius();

    // ISS-like orbit
    double alt = 400e3;
    double r = Re + alt;
    double v = std::sqrt(mu / r);

    SatelliteState state = {{r, 0, 0}, {0, v, 0}, 0.0};

    GravityModel gravity(earth, false, false); // point mass only
    auto accelFunc = [&](const SatelliteState& s) {
        return gravity.computeAcceleration(s);
    };

    // Run for one full orbit
    double period = 2.0 * M_PI * std::sqrt(r * r * r / mu);
    double dt = 10.0;
    int steps = static_cast<int>(period / dt);

    double initialEnergy = 0.5 * v * v - mu / r;

    for (int i = 0; i < steps; ++i) {
        state = Integrator::rk4(state, accelFunc, dt);
    }

    double finalR = state.position.magnitude();
    double finalV = state.velocity.magnitude();
    double finalEnergy = 0.5 * finalV * finalV - mu / finalR;

    // Energy should be conserved within ~0.01%
    double energyError = std::abs((finalEnergy - initialEnergy) / initialEnergy);
    assert(energyError < 1e-4);

    // Radius should return close to initial
    double radiusError = std::abs(finalR - r) / r;
    assert(radiusError < 1e-4);

    std::cout << "PASSED (energy error: " << energyError
              << ", radius error: " << radiusError << ")\n";
}

void testOrbitalElements() {
    std::cout << "Testing orbital elements conversion... ";

    auto earth = CelestialBody::Earth();
    double mu = earth.getMu();
    double Re = earth.getRadius();

    // Circular orbit
    double r = Re + 400e3;
    double v = std::sqrt(mu / r);

    auto oe = OrbitalMechanics::stateToElements({r, 0, 0}, {0, v, 0}, mu);

    // Should be nearly circular
    assert(oe.eccentricity < 1e-6);
    // Semi-major axis should match radius
    assert(approxEqual(oe.semiMajorAxis, r, 1.0)); // within 1 meter
    // Inclination should be ~0 (equatorial)
    assert(oe.inclination < 1e-6);

    std::cout << "PASSED (e=" << oe.eccentricity
              << ", a=" << oe.semiMajorAxis / 1000.0 << " km)\n";
}

void testEulerVsRK4() {
    std::cout << "Testing Euler vs RK4 accuracy... ";

    auto earth = CelestialBody::Earth();
    double mu = earth.getMu();
    double Re = earth.getRadius();

    double r = Re + 400e3;
    double v = std::sqrt(mu / r);

    SatelliteState state = {{r, 0, 0}, {0, v, 0}, 0.0};

    GravityModel gravity(earth, false, false);
    auto accelFunc = [&](const SatelliteState& s) {
        return gravity.computeAcceleration(s);
    };

    // Run 1000 steps with both methods
    SatelliteState eulerState = state;
    SatelliteState rk4State = state;
    double dt = 10.0;

    for (int i = 0; i < 1000; ++i) {
        eulerState = Integrator::euler(eulerState, accelFunc, dt);
        rk4State = Integrator::rk4(rk4State, accelFunc, dt);
    }

    // RK4 should conserve radius better than Euler
    double eulerError = std::abs(eulerState.position.magnitude() - r) / r;
    double rk4Error = std::abs(rk4State.position.magnitude() - r) / r;

    assert(rk4Error < eulerError);

    std::cout << "PASSED (Euler err: " << eulerError
              << ", RK4 err: " << rk4Error << ")\n";
}

int main() {
    std::cout << "\n=== Running Tests ===\n\n";

    testVector3D();
    testCelestialBody();
    testCircularOrbit();
    testOrbitalElements();
    testEulerVsRK4();

    std::cout << "\n=== All Tests Passed! ===\n\n";
    return 0;
}
