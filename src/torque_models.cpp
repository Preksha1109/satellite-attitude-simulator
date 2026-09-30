#include "attitude/torque_model.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace attitude {

GravityGradient::GravityGradient(double mu, double r) {
    if (!(mu > 0.0) || !(r > 0.0)) throw std::invalid_argument("GravityGradient: mu and r must be > 0");
    n_ = std::sqrt(mu / (r * r * r));
}

Vec3 GravityGradient::torque(double t, const State& s, const RigidBody& body) const {
    const Vec3 r_hat_i{std::cos(n_ * t), std::sin(n_ * t), 0.0};
    const Vec3 r_hat_b = s.q.normalized().rotateInverse(r_hat_i);
    return 3.0 * n_ * n_ * cross(r_hat_b, body.inertia() * r_hat_b);
}

PDController::PDController(const Quat& target, double kp, double kd, double max_torque_nm)
    : target_(target.normalized()), kp_(kp), kd_(kd), max_torque_(max_torque_nm) {
    if (kp < 0.0 || kd < 0.0) throw std::invalid_argument("PDController: gains must be >= 0");
}

Vec3 PDController::torque(double, const State& s, const RigidBody&) const {
    Quat qe = target_.conjugate() * s.q.normalized();
    if (qe.w < 0.0) qe = {-qe.w, -qe.x, -qe.y, -qe.z};
    Vec3 tau = -kp_ * qe.vec() - kd_ * s.w;
    if (max_torque_ > 0.0) {
        tau.x = std::clamp(tau.x, -max_torque_, max_torque_);
        tau.y = std::clamp(tau.y, -max_torque_, max_torque_);
        tau.z = std::clamp(tau.z, -max_torque_, max_torque_);
    }
    return tau;
}

}  // namespace attitude
