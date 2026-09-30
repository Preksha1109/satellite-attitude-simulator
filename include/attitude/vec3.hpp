#pragma once
// Minimal 3-vector and 3x3 matrix types (no external dependencies).

#include <array>
#include <cmath>
#include <stdexcept>

namespace attitude {

constexpr double kPi = 3.14159265358979323846;

struct Vec3 {
    double x{0.0}, y{0.0}, z{0.0};
    constexpr Vec3() = default;
    constexpr Vec3(double x_, double y_, double z_) : x(x_), y(y_), z(z_) {}
};

constexpr Vec3 operator+(const Vec3& a, const Vec3& b) { return {a.x + b.x, a.y + b.y, a.z + b.z}; }
constexpr Vec3 operator-(const Vec3& a, const Vec3& b) { return {a.x - b.x, a.y - b.y, a.z - b.z}; }
constexpr Vec3 operator-(const Vec3& a) { return {-a.x, -a.y, -a.z}; }
constexpr Vec3 operator*(double s, const Vec3& a) { return {s * a.x, s * a.y, s * a.z}; }
constexpr Vec3 operator*(const Vec3& a, double s) { return s * a; }
constexpr Vec3 operator/(const Vec3& a, double s) { return {a.x / s, a.y / s, a.z / s}; }

constexpr double dot(const Vec3& a, const Vec3& b) { return a.x * b.x + a.y * b.y + a.z * b.z; }
constexpr Vec3 cross(const Vec3& a, const Vec3& b) {
    return {a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x};
}
inline double norm(const Vec3& a) { return std::sqrt(dot(a, a)); }

/// Row-major 3x3 matrix.
struct Mat3 {
    std::array<double, 9> m{};

    static constexpr Mat3 diag(double a, double b, double c) {
        Mat3 r;
        r.m = {a, 0, 0, 0, b, 0, 0, 0, c};
        return r;
    }
    static constexpr Mat3 identity() { return diag(1.0, 1.0, 1.0); }

    constexpr double operator()(int r, int c) const { return m[3 * r + c]; }
    constexpr double& operator()(int r, int c) { return m[3 * r + c]; }
};

constexpr Vec3 operator*(const Mat3& A, const Vec3& v) {
    return {A(0, 0) * v.x + A(0, 1) * v.y + A(0, 2) * v.z,
            A(1, 0) * v.x + A(1, 1) * v.y + A(1, 2) * v.z,
            A(2, 0) * v.x + A(2, 1) * v.y + A(2, 2) * v.z};
}

constexpr Mat3 operator*(const Mat3& A, const Mat3& B) {
    Mat3 R;
    for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 3; ++j)
            R(i, j) = A(i, 0) * B(0, j) + A(i, 1) * B(1, j) + A(i, 2) * B(2, j);
    return R;
}

constexpr double det(const Mat3& A) {
    return A(0, 0) * (A(1, 1) * A(2, 2) - A(1, 2) * A(2, 1)) -
           A(0, 1) * (A(1, 0) * A(2, 2) - A(1, 2) * A(2, 0)) +
           A(0, 2) * (A(1, 0) * A(2, 1) - A(1, 1) * A(2, 0));
}

/// Inverse via the adjugate. Throws std::domain_error for singular matrices.
inline Mat3 inverse(const Mat3& A) {
    const double d = det(A);
    if (std::abs(d) < 1e-300) throw std::domain_error("inverse(): singular matrix");
    Mat3 R;
    R(0, 0) = (A(1, 1) * A(2, 2) - A(1, 2) * A(2, 1)) / d;
    R(0, 1) = (A(0, 2) * A(2, 1) - A(0, 1) * A(2, 2)) / d;
    R(0, 2) = (A(0, 1) * A(1, 2) - A(0, 2) * A(1, 1)) / d;
    R(1, 0) = (A(1, 2) * A(2, 0) - A(1, 0) * A(2, 2)) / d;
    R(1, 1) = (A(0, 0) * A(2, 2) - A(0, 2) * A(2, 0)) / d;
    R(1, 2) = (A(0, 2) * A(1, 0) - A(0, 0) * A(1, 2)) / d;
    R(2, 0) = (A(1, 0) * A(2, 1) - A(1, 1) * A(2, 0)) / d;
    R(2, 1) = (A(0, 1) * A(2, 0) - A(0, 0) * A(2, 1)) / d;
    R(2, 2) = (A(0, 0) * A(1, 1) - A(0, 1) * A(1, 0)) / d;
    return R;
}

}  // namespace attitude
