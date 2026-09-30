#pragma once
// Torque models: anything that produces an external torque on the satellite.
//
// To add a new disturbance or controller, derive from TorqueModel and implement
// torque(). The simulation engine never needs to change (see examples/).

#include <memory>
#include <vector>

#include "attitude/quaternion.hpp"
#include "attitude/rigid_body.hpp"
#include "attitude/state.hpp"

namespace attitude {

class TorqueModel {
public:
    virtual ~TorqueModel() = default;
    /// External torque [N m] in the BODY frame at time t [s] for the given state.
    virtual Vec3 torque(double t, const State& s, const RigidBody& body) const = 0;
};

/// tau = 0 (torque-free motion).
class NoTorque final : public TorqueModel {
public:
    Vec3 torque(double, const State&, const RigidBody&) const override { return {}; }
};

/// Constant body-fixed torque.
class ConstantTorque final : public TorqueModel {
public:
    explicit ConstantTorque(const Vec3& tau) : tau_(tau) {}
    Vec3 torque(double, const State&, const RigidBody&) const override { return tau_; }

private:
    Vec3 tau_;
};

/// Sum of several torque models (e.g. disturbance + controller).
class CompositeTorque final : public TorqueModel {
public:
    CompositeTorque& add(std::shared_ptr<const TorqueModel> m) {
        models_.push_back(std::move(m));
        return *this;
    }
    Vec3 torque(double t, const State& s, const RigidBody& b) const override {
        Vec3 sum;
        for (const auto& m : models_) sum = sum + m->torque(t, s, b);
        return sum;
    }

private:
    std::vector<std::shared_ptr<const TorqueModel>> models_;
};

/// Gravity-gradient disturbance for a circular orbit in the inertial x-y plane.
///
///   tau = 3 (mu / r^3) * (rhat_b x I rhat_b)
///
/// where rhat_b is the unit vector from Earth's centre to the satellite, in the body
/// frame. The orbit position is rhat_i(t) = (cos nt, sin nt, 0) with n = sqrt(mu / r^3).
class GravityGradient final : public TorqueModel {
public:
    GravityGradient(double mu_m3_s2, double orbit_radius_m);
    double meanMotion() const { return n_; }
    Vec3 torque(double t, const State& s, const RigidBody& body) const override;

private:
    double n_;
};

/// Quaternion-feedback PD attitude controller with per-axis torque saturation.
///
///   q_e = q_target* (x) q      (sign chosen so that q_e.w >= 0: shortest path)
///   tau = -kp * vec(q_e) - kd * omega,   each axis clamped to +/- max_torque
///
/// The clamp is a simple actuator model (e.g. reaction-wheel torque limit).
/// Pass max_torque <= 0 to disable saturation.
class PDController final : public TorqueModel {
public:
    PDController(const Quat& target, double kp, double kd, double max_torque_nm = 0.0);
    Vec3 torque(double t, const State& s, const RigidBody& body) const override;
    const Quat& target() const { return target_; }

private:
    Quat target_;
    double kp_, kd_, max_torque_;
};

}  // namespace attitude
