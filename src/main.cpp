// attitude_sim: command-line front end with a few ready-made scenarios.

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <iostream>
#include <memory>
#include <string>

#include "attitude/simulator.hpp"

using namespace attitude;

namespace {

// 6U-class CubeSat: ~12 kg uniform box, 0.100 x 0.226 x 0.366 m (Ixx, Iyy, Izz below).
// Iyy is the INTERMEDIATE principal moment, which matters for the "tumble" scenario.
RigidBody cubesat6U() { return RigidBody::fromPrincipalMoments(0.185, 0.144, 0.061); }

struct Scenario {
    std::string description;
    RigidBody body;
    std::shared_ptr<const TorqueModel> torque;
    State initial;
    double t_end;
    double dt;
    double log_interval_s;
    std::optional<Quat> target;
    bool torque_free;
};

Scenario makeScenario(const std::string& name) {
    const double deg = kPi / 180.0;
    if (name == "tumble") {
        // Torque-free spin about the intermediate axis with a small perturbation: the
        // classic unstable rotation (Dzhanibekov / tennis-racket effect).
        return {"torque-free spin about the intermediate axis (unstable)",
                cubesat6U(),
                std::make_shared<NoTorque>(),
                {Quat{}, {0.005, 0.30, 0.005}},
                200.0, 0.01, 0.25, std::nullopt, true};
    }
    if (name == "slew") {
        // 60 deg slew about (1,1,1) from rest with a torque-limited PD controller.
        const Quat target = Quat::fromAxisAngle({1, 1, 1}, 60.0 * deg);
        return {"60 deg slew, PD controller, 2 mN m torque limit per axis",
                cubesat6U(),
                std::make_shared<PDController>(target, 0.016, 0.06, 2e-3),
                {Quat{}, {}},
                300.0, 0.01, 0.5, target, false};
    }
    if (name == "detumble") {
        // Initial tumble of ~6.4 deg/s, controller drives rates to zero and attitude to identity.
        const Quat target{};
        return {"detumble from ~6.4 deg/s and point to the identity attitude",
                cubesat6U(),
                std::make_shared<PDController>(target, 0.016, 0.06, 2e-3),
                {Quat::fromAxisAngle({0, 0, 1}, 30.0 * deg), {0.08, -0.05, 0.06}},
                400.0, 0.01, 0.5, target, false};
    }
    if (name == "gravity") {
        // Uncontrolled satellite in a 525 km circular orbit under the gravity-gradient torque.
        const double mu = 3.986004418e14, r = 6378137.0 + 525e3;
        return {"gravity-gradient torque over one 525 km orbit (uncontrolled)",
                cubesat6U(),
                std::make_shared<GravityGradient>(mu, r),
                {Quat::fromAxisAngle({1, 1, 0}, 10.0 * deg), {0.0, 0.0, 0.0}},
                5700.0, 0.05, 10.0, std::nullopt, false};
    }
    throw std::invalid_argument("unknown scenario '" + name + "'");
}

void usage() {
    std::cout << "usage: attitude_sim <scenario> [options]\n\n"
                 "scenarios:  tumble | slew | detumble | gravity\n"
                 "options:\n"
                 "  --integrator rk4|euler   (default rk4)\n"
                 "  --dt <seconds>           (scenario default)\n"
                 "  --t-end <seconds>        (scenario default)\n"
                 "  --out <file.csv>         (default data/<scenario>.csv)\n";
}

}  // namespace

int main(int argc, char** argv) {
    if (argc < 2 || std::string(argv[1]) == "-h" || std::string(argv[1]) == "--help") {
        usage();
        return argc < 2 ? 2 : 0;
    }
    try {
        const std::string name = argv[1];
        Scenario sc = makeScenario(name);
        Method method = Method::RK4;
        std::string out = "data/" + name + ".csv";
        for (int i = 2; i < argc; ++i) {
            const std::string a = argv[i];
            auto next = [&]() -> std::string {
                if (i + 1 >= argc) throw std::invalid_argument("missing value for " + a);
                return argv[++i];
            };
            if (a == "--integrator") method = parseMethod(next());
            else if (a == "--dt") sc.dt = std::stod(next());
            else if (a == "--t-end") sc.t_end = std::stod(next());
            else if (a == "--out") out = next();
            else throw std::invalid_argument("unknown option " + a);
        }

        SimConfig cfg;
        cfg.dt = sc.dt;
        cfg.t_end = sc.t_end;
        cfg.method = method;
        cfg.target = sc.target;
        cfg.log_every = static_cast<std::size_t>(std::max(1.0, std::round(sc.log_interval_s / sc.dt)));

        const auto t0 = std::chrono::steady_clock::now();
        const auto samples = simulate(sc.body, *sc.torque, sc.initial, cfg);
        const double wall = std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
        writeCsv(out, samples);

        std::cout << "scenario:   " << name << " - " << sc.description << "\n"
                  << "integrator: " << toString(method) << ", dt = " << cfg.dt << " s, t_end = " << cfg.t_end
                  << " s (" << static_cast<long long>(std::llround(cfg.t_end / cfg.dt)) << " steps, " << wall
                  << " s wall)\n"
                  << "wrote:      " << out << " (" << samples.size() << " rows)\n";
        if (sc.torque_free) {
            const auto r = conservationDrift(samples);
            std::cout << "max relative drift of |H| vector: " << r.max_rel_momentum_drift
                      << ",  of kinetic energy: " << r.max_rel_energy_drift << "\n";
        }
        if (sc.target) {
            const auto& last = samples.back();
            std::cout << "final pointing error: " << last.pointing_error_deg << " deg, |omega| = "
                      << norm(last.state.w) << " rad/s\n";
        }
        return 0;
    } catch (const std::exception& e) {
        std::cerr << "error: " << e.what() << "\n";
        return 1;
    }
}
