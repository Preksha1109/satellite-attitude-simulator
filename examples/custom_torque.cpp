// Example: add a new disturbance model WITHOUT touching the simulation core.
//
// A body-fixed torque with a sinusoidal time dependence, combined with a rate damper.

#include <cmath>
#include <iostream>

#include "attitude/simulator.hpp"

using namespace attitude;

class SinusoidalDisturbance final : public TorqueModel {
public:
    SinusoidalDisturbance(const Vec3& amplitude, double omega_rad_s) : a_(amplitude), w_(omega_rad_s) {}
    Vec3 torque(double t, const State&, const RigidBody&) const override { return std::sin(w_ * t) * a_; }

private:
    Vec3 a_;
    double w_;
};

class RateDamper final : public TorqueModel {
public:
    explicit RateDamper(double kd) : kd_(kd) {}
    Vec3 torque(double, const State& s, const RigidBody&) const override { return -kd_ * s.w; }

private:
    double kd_;
};

int main() {
    const RigidBody body = RigidBody::fromPrincipalMoments(0.185, 0.144, 0.061);

    CompositeTorque total;
    total.add(std::make_shared<SinusoidalDisturbance>(Vec3{1e-4, 0.0, 2e-4}, 0.5));
    total.add(std::make_shared<RateDamper>(0.02));

    SimConfig cfg;
    cfg.dt = 0.01;
    cfg.t_end = 60.0;

    const auto samples = simulate(body, total, State{}, cfg);
    const auto& last = samples.back();
    std::cout << "after " << last.t << " s: |omega| = " << norm(last.state.w) << " rad/s\n";
    return 0;
}
