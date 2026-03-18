#pragma once
#include <cmath>
#include <iostream>
#include <array>

namespace orbit {

class Vector3D {
public:
    double x, y, z;

    // Constructors
    Vector3D();
    Vector3D(double x, double y, double z);

    // Arithmetic operators
    Vector3D operator+(const Vector3D& other) const;
    Vector3D operator-(const Vector3D& other) const;
    Vector3D operator*(double scalar) const;
    Vector3D operator/(double scalar) const;
    Vector3D& operator+=(const Vector3D& other);
    Vector3D& operator*=(double scalar);

    // Vector operations
    double magnitude() const;
    double magnitudeSquared() const;
    Vector3D normalized() const;
    double dot(const Vector3D& other) const;
    Vector3D cross(const Vector3D& other) const;
    double distanceTo(const Vector3D& other) const;

    // Output
    friend std::ostream& operator<<(std::ostream& os, const Vector3D& v);
};

// Scalar * Vector
Vector3D operator*(double scalar, const Vector3D& v);

} // namespace orbit
