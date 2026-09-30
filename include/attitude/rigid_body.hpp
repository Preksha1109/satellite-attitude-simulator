#pragma once

#include <algorithm>
#include <cmath>
#include <stdexcept>

#include "attitude/vec3.hpp"

namespace attitude {

/// Rigid body described by its inertia tensor about the centre of mass (body frame).
class RigidBody {
public:
    /// Throws std::invalid_argument unless `inertia` is symmetric positive-definite.
    explicit RigidBody(const Mat3& inertia) : I_(inertia), Iinv_(Mat3::identity()) {
        const double scale = std::max({std::abs(I_(0, 0)), std::abs(I_(1, 1)), std::abs(I_(2, 2)), 1e-300});
        for (int i = 0; i < 3; ++i)
            for (int j = i + 1; j < 3; ++j)
                if (std::abs(I_(i, j) - I_(j, i)) > 1e-12 * scale)
                    throw std::invalid_argument("RigidBody: inertia tensor must be symmetric");
        const double m1 = I_(0, 0);
        const double m2 = I_(0, 0) * I_(1, 1) - I_(0, 1) * I_(1, 0);
        const double m3 = det(I_);
        if (!(m1 > 0.0 && m2 > 0.0 && m3 > 0.0))
            throw std::invalid_argument("RigidBody: inertia tensor must be positive-definite");
        Iinv_ = inverse(I_);
    }

    static RigidBody fromPrincipalMoments(double Ixx, double Iyy, double Izz) {
        return RigidBody(Mat3::diag(Ixx, Iyy, Izz));
    }

    const Mat3& inertia() const { return I_; }
    const Mat3& inverseInertia() const { return Iinv_; }

    Vec3 angularMomentum(const Vec3& w) const { return I_ * w; }                  // body frame
    double kineticEnergy(const Vec3& w) const { return 0.5 * dot(w, I_ * w); }    // rotational

private:
    Mat3 I_;
    Mat3 Iinv_;
};

}  // namespace attitude
