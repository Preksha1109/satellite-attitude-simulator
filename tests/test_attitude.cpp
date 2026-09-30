// Self-contained tests (no external framework). Run with `ctest` or ./attitude_tests.

#include <cmath>
#include <functional>
#include <iostream>
#include <memory>
#include <string>
#include <vector>

#include "attitude/simulator.hpp"

using namespace attitude;

namespace {

int g_failures = 0;
int g_checks = 0;

void check(bool ok, const std::string& what) {
    ++g_checks;
    if (!ok) {
        ++g_failures;
        std::cerr << "    FAIL: " << what << "\n";
    }
}
void checkLess(double value, double limit, const std::string& what) {
    std::cout << "    " << what << ": " << value << " (limit " << limit << ")\n";
    check(value < limit, what);
}

struct TestCase {
    std::string name;
    std::function<void()> fn;
};
std::vector<TestCase>& registry() {
    static std::vector<TestCase> r;
    return r;
}
struct Registrar {
    Registrar(const std::string& n, std::function<void()> f) { registry().push_back({n, std::move(f)}); }
};
#define TEST(name)                                     \
    static void name();                                \
    static Registrar reg_##name(#name, name);          \
    static void name()

const double deg = kPi / 180.0;

// --------------------------------------------------------------------------- math
TEST(quaternion_rotation_matches_known_cases) {
    const Quat qz = Quat::fromAxisAngle({0, 0, 1}, 90.0 * deg);
    const Vec3 v = qz.rotate({1, 0, 0});
    checkLess(norm(v - Vec3{0, 1, 0}), 1e-15, "90 deg about z maps x -> y");
    const Vec3 back = qz.rotateInverse(v);
    checkLess(norm(back - Vec3{1, 0, 0}), 1e-15, "rotateInverse undoes rotate");
    // composition: two 45 deg rotations == one 90 deg rotation
    const Quat q45 = Quat::fromAxisAngle({0, 0, 1}, 45.0 * deg);
    checkLess(angleBetween(q45 * q45, qz), 1e-14, "q45 * q45 == q90");
    checkLess(std::abs(angleBetween(Quat{}, qz) - 90.0 * deg), 1e-14, "angleBetween = 90 deg");
}

TEST(matrix_inverse_and_rigid_body_validation) {
    Mat3 A;
    A.m = {4, 1, 0.5, 1, 3, 0.2, 0.5, 0.2, 2};
    const Mat3 P = A * inverse(A);
    double err = 0;
    for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 3; ++j) err = std::max(err, std::abs(P(i, j) - (i == j ? 1.0 : 0.0)));
    checkLess(err, 1e-14, "A * inv(A) == I");

    bool threw = false;
    try { RigidBody::fromPrincipalMoments(1.0, -1.0, 1.0); } catch (const std::invalid_argument&) { threw = true; }
    check(threw, "negative principal moment is rejected");
    threw = false;
    Mat3 asym = Mat3::diag(1, 1, 1);
    asym(0, 1) = 0.3;
    try { RigidBody b(asym); } catch (const std::invalid_argument&) { threw = true; }
    check(threw, "non-symmetric inertia tensor is rejected");
}

// --------------------------------------------------------------------------- physics
TEST(axisymmetric_torque_free_matches_analytic_solution) {
    // I1 = I2 = It: omega_3 is constant and (omega_1, omega_2) rotate at Omega = (I3 - It)/It * omega_3.
    const double It = 0.10, I3 = 0.06;
    const RigidBody body = RigidBody::fromPrincipalMoments(It, It, I3);
    const Vec3 w0{0.10, -0.04, 0.50};
    const double Omega = (I3 - It) / It * w0.z;

    SimConfig cfg;
    cfg.dt = 0.01;
    cfg.t_end = 100.0;
    cfg.log_every = 100;
    const auto samples = simulate(body, NoTorque{}, State{Quat{}, w0}, cfg);

    double worst = 0;
    for (const auto& s : samples) {
        const double c = std::cos(Omega * s.t), sn = std::sin(Omega * s.t);
        const Vec3 exact{w0.x * c - w0.y * sn, w0.x * sn + w0.y * c, w0.z};
        worst = std::max(worst, norm(s.state.w - exact));
    }
    checkLess(worst, 1e-8, "max |omega - analytic| over 100 s [rad/s]");
}

TEST(torque_free_conserves_energy_and_angular_momentum) {
    const RigidBody body = RigidBody::fromPrincipalMoments(0.185, 0.144, 0.061);
    SimConfig cfg;
    cfg.dt = 0.01;
    cfg.t_end = 1000.0;
    cfg.log_every = 100;
    // spin about the intermediate axis: chaotic-looking tumbling, a demanding test
    const auto samples = simulate(body, NoTorque{}, State{Quat::fromAxisAngle({1, 2, 3}, 0.4), {0.005, 0.30, 0.005}}, cfg);
    const auto r = conservationDrift(samples);
    checkLess(r.max_rel_energy_drift, 1e-9, "relative kinetic-energy drift over 1000 s");
    checkLess(r.max_rel_momentum_drift, 1e-9, "relative angular-momentum drift over 1000 s");
}

TEST(rk4_converges_at_fourth_order) {
    const RigidBody body = RigidBody::fromPrincipalMoments(0.185, 0.144, 0.061);
    const State x0{Quat{}, {0.05, 0.30, 0.02}};
    auto finalState = [&](double dt) {
        SimConfig cfg;
        cfg.dt = dt;
        cfg.t_end = 20.0;
        cfg.log_every = 1u << 30;  // only first and last samples
        return simulate(body, NoTorque{}, x0, cfg).back().state;
    };
    const State ref = finalState(0.00625);
    const double e1 = norm(finalState(0.1).w - ref.w);
    const double e2 = norm(finalState(0.05).w - ref.w);
    const double ratio = e1 / e2;
    std::cout << "    error(dt=0.1) = " << e1 << ", error(dt=0.05) = " << e2 << ", ratio = " << ratio
              << " (4th order -> 16)\n";
    check(ratio > 12.0 && ratio < 20.0, "halving dt reduces error by ~16x");
}

TEST(rk4_is_far_more_accurate_than_euler) {
    const RigidBody body = RigidBody::fromPrincipalMoments(0.185, 0.144, 0.061);
    const State x0{Quat{}, {0.005, 0.30, 0.005}};
    auto drift = [&](Method m) {
        SimConfig cfg;
        cfg.dt = 0.01;
        cfg.t_end = 100.0;
        cfg.log_every = 100;
        cfg.method = m;
        return conservationDrift(simulate(body, NoTorque{}, x0, cfg)).max_rel_energy_drift;
    };
    const double e_rk4 = drift(Method::RK4), e_euler = drift(Method::Euler);
    std::cout << "    energy drift over 100 s: RK4 = " << e_rk4 << ", Euler = " << e_euler << "\n";
    check(e_rk4 * 1e4 < e_euler, "RK4 energy drift is at least 4 orders below explicit Euler");
}

TEST(constant_torque_spins_up_linearly) {
    // From rest about a principal axis: omega = tau / I * t exactly (gyroscopic term vanishes).
    const RigidBody body = RigidBody::fromPrincipalMoments(0.2, 0.3, 0.4);
    SimConfig cfg;
    cfg.dt = 0.01;
    cfg.t_end = 50.0;
    const auto last = simulate(body, ConstantTorque({0.0, 0.003, 0.0}), State{}, cfg).back();
    checkLess(std::abs(last.state.w.y - 0.003 / 0.3 * 50.0), 1e-12, "omega_y(50 s) error [rad/s]");
    checkLess(std::abs(last.state.w.x) + std::abs(last.state.w.z), 1e-14, "off-axis rates stay zero");
}

// --------------------------------------------------------------------------- models
TEST(gravity_gradient_torque_is_zero_when_aligned_and_scales_correctly) {
    const double mu = 3.986004418e14, r = 6903137.0;
    const RigidBody body = RigidBody::fromPrincipalMoments(0.185, 0.144, 0.061);
    const GravityGradient gg(mu, r);
    // At t=0 the radial direction is inertial x; with q = identity it is a principal axis -> no torque.
    checkLess(norm(gg.torque(0.0, State{}, body)), 1e-20, "torque = 0 with principal axis along radial");
    // 45 deg tilt about z between principal axes x and y: |tau| = 3 n^2 |I1 - I2| sin(45)cos(45)
    const State tilted{Quat::fromAxisAngle({0, 0, 1}, 45.0 * deg), {}};
    const double n2 = gg.meanMotion() * gg.meanMotion();
    const double expected = 3.0 * n2 * std::abs(0.185 - 0.144) * 0.5;
    checkLess(std::abs(norm(gg.torque(0.0, tilted, body)) - expected) / expected, 1e-12, "tilted torque magnitude (rel. error)");
}

TEST(pd_controller_slews_to_target_and_detumbles) {
    const RigidBody body = RigidBody::fromPrincipalMoments(0.185, 0.144, 0.061);
    const Quat target = Quat::fromAxisAngle({1, 1, 1}, 60.0 * deg);
    const PDController ctrl(target, 0.016, 0.06, 2e-3);
    SimConfig cfg;
    cfg.dt = 0.01;
    cfg.t_end = 300.0;
    cfg.log_every = 100;
    cfg.target = target;
    const auto last = simulate(body, ctrl, State{Quat{}, {0.08, -0.05, 0.06}}, cfg).back();
    checkLess(last.pointing_error_deg, 0.1, "final pointing error [deg]");
    checkLess(norm(last.state.w), 1e-4, "final |omega| [rad/s]");
}

TEST(controller_torque_respects_saturation_limit) {
    const RigidBody body = RigidBody::fromPrincipalMoments(0.185, 0.144, 0.061);
    const PDController ctrl(Quat::fromAxisAngle({0, 0, 1}, 170.0 * deg), 10.0, 10.0, 1e-3);
    const Vec3 tau = ctrl.torque(0.0, State{Quat{}, {1.0, -1.0, 1.0}}, body);
    check(std::abs(tau.x) <= 1e-3 && std::abs(tau.y) <= 1e-3 && std::abs(tau.z) <= 1e-3, "each axis within +/-1 mN m");
}

}  // namespace

int main() {
    for (const auto& t : registry()) {
        const int before = g_failures;
        std::cout << "[ RUN  ] " << t.name << "\n";
        try {
            t.fn();
        } catch (const std::exception& e) {
            ++g_failures;
            std::cerr << "    EXCEPTION: " << e.what() << "\n";
        }
        std::cout << (g_failures == before ? "[  OK  ] " : "[ FAIL ] ") << t.name << "\n";
    }
    std::cout << "\n" << registry().size() << " tests, " << g_checks << " checks, " << g_failures << " failures\n";
    return g_failures == 0 ? 0 : 1;
}
