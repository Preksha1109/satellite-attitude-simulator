#pragma once
// Unit quaternions, Hamilton convention, scalar-first (w, x, y, z).
//
// Convention used throughout the library:
//   q rotates a vector from the BODY frame to the INERTIAL frame:
//       v_inertial = q (x) (0, v_body) (x) q*
//   Angular velocity omega is expressed in the BODY frame.

#include <cmath>

#include "attitude/vec3.hpp"

namespace attitude {

struct Quat {
    double w{1.0}, x{0.0}, y{0.0}, z{0.0};

    constexpr Quat() = default;
    constexpr Quat(double w_, double x_, double y_, double z_) : w(w_), x(x_), y(y_), z(z_) {}

    static Quat fromAxisAngle(const Vec3& axis, double angle_rad) {
        const Vec3 u = axis / norm(axis);
        const double s = std::sin(0.5 * angle_rad);
        return {std::cos(0.5 * angle_rad), s * u.x, s * u.y, s * u.z};
    }

    constexpr Vec3 vec() const { return {x, y, z}; }
    constexpr Quat conjugate() const { return {w, -x, -y, -z}; }
    double length() const { return std::sqrt(w * w + x * x + y * y + z * z); }
    Quat normalized() const {
        const double n = length();
        return {w / n, x / n, y / n, z / n};
    }

    /// Rotate a vector: body -> inertial (for a unit quaternion).
    Vec3 rotate(const Vec3& v) const {
        const Vec3 u = vec();
        const Vec3 t = 2.0 * cross(u, v);
        return v + w * t + cross(u, t);
    }
    /// Rotate a vector: inertial -> body.
    Vec3 rotateInverse(const Vec3& v) const { return conjugate().rotate(v); }
};

constexpr Quat operator*(const Quat& a, const Quat& b) {  // Hamilton product
    return {a.w * b.w - a.x * b.x - a.y * b.y - a.z * b.z,
            a.w * b.x + a.x * b.w + a.y * b.z - a.z * b.y,
            a.w * b.y - a.x * b.z + a.y * b.w + a.z * b.x,
            a.w * b.z + a.x * b.y - a.y * b.x + a.z * b.w};
}
constexpr Quat operator+(const Quat& a, const Quat& b) { return {a.w + b.w, a.x + b.x, a.y + b.y, a.z + b.z}; }
constexpr Quat operator*(double s, const Quat& a) { return {s * a.w, s * a.x, s * a.y, s * a.z}; }

/// Smallest rotation angle [rad] taking orientation a to orientation b.
inline double angleBetween(const Quat& a, const Quat& b) {
    const Quat e = a.conjugate() * b;
    return 2.0 * std::atan2(norm(e.vec()), std::abs(e.w));
}

}  // namespace attitude
