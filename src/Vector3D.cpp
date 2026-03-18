#include "Vector3D.h"
#include <iomanip>

namespace orbit {

Vector3D::Vector3D() : x(0.0), y(0.0), z(0.0) {}

Vector3D::Vector3D(double x, double y, double z) : x(x), y(y), z(z) {}

Vector3D Vector3D::operator+(const Vector3D& other) const {
    return {x + other.x, y + other.y, z + other.z};
}

Vector3D Vector3D::operator-(const Vector3D& other) const {
    return {x - other.x, y - other.y, z - other.z};
}

Vector3D Vector3D::operator*(double scalar) const {
    return {x * scalar, y * scalar, z * scalar};
}

Vector3D Vector3D::operator/(double scalar) const {
    return {x / scalar, y / scalar, z / scalar};
}

Vector3D& Vector3D::operator+=(const Vector3D& other) {
    x += other.x; y += other.y; z += other.z;
    return *this;
}

Vector3D& Vector3D::operator*=(double scalar) {
    x *= scalar; y *= scalar; z *= scalar;
    return *this;
}

double Vector3D::magnitude() const {
    return std::sqrt(x * x + y * y + z * z);
}

double Vector3D::magnitudeSquared() const {
    return x * x + y * y + z * z;
}

Vector3D Vector3D::normalized() const {
    double mag = magnitude();
    if (mag < 1e-15) return {0.0, 0.0, 0.0};
    return *this / mag;
}

double Vector3D::dot(const Vector3D& other) const {
    return x * other.x + y * other.y + z * other.z;
}

Vector3D Vector3D::cross(const Vector3D& other) const {
    return {
        y * other.z - z * other.y,
        z * other.x - x * other.z,
        x * other.y - y * other.x
    };
}

double Vector3D::distanceTo(const Vector3D& other) const {
    return (*this - other).magnitude();
}

std::ostream& operator<<(std::ostream& os, const Vector3D& v) {
    os << std::fixed << std::setprecision(3)
       << "(" << v.x << ", " << v.y << ", " << v.z << ")";
    return os;
}

Vector3D operator*(double scalar, const Vector3D& v) {
    return v * scalar;
}

} // namespace orbit
