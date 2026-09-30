#include "attitude/simulator.hpp"

#include <algorithm>
#include <cmath>
#include <fstream>
#include <limits>
#include <stdexcept>

namespace attitude {

std::vector<Sample> simulate(const RigidBody& body, const TorqueModel& torque, const State& initial,
                             const SimConfig& cfg) {
    if (!(cfg.dt > 0.0)) throw std::invalid_argument("simulate: dt must be > 0");
    if (!(cfg.t_end >= 0.0)) throw std::invalid_argument("simulate: t_end must be >= 0");
    const std::size_t log_every = std::max<std::size_t>(cfg.log_every, 1);
    const auto n_steps = static_cast<std::size_t>(std::llround(cfg.t_end / cfg.dt));

    State s = initial;
    s.q = s.q.normalized();

    std::vector<Sample> out;
    out.reserve(n_steps / log_every + 2);

    auto record = [&](double t) {
        Sample smp;
        smp.t = t;
        smp.state = s;
        smp.torque = torque.torque(t, s, body);
        smp.kinetic_energy = body.kineticEnergy(s.w);
        smp.angular_momentum_inertial = s.q.rotate(body.angularMomentum(s.w));
        smp.pointing_error_deg = cfg.target ? angleBetween(s.q, *cfg.target) * 180.0 / kPi
                                            : std::numeric_limits<double>::quiet_NaN();
        out.push_back(smp);
    };

    record(0.0);
    for (std::size_t i = 1; i <= n_steps; ++i) {
        s = step(s, static_cast<double>(i - 1) * cfg.dt, cfg.dt, body, torque, cfg.method);
        if (i % log_every == 0 || i == n_steps) record(static_cast<double>(i) * cfg.dt);
    }
    return out;
}

ConservationReport conservationDrift(const std::vector<Sample>& samples) {
    ConservationReport r;
    if (samples.empty()) return r;
    const double e0 = samples.front().kinetic_energy;
    const Vec3 h0 = samples.front().angular_momentum_inertial;
    const double h0n = norm(h0);
    for (const auto& s : samples) {
        if (e0 > 0.0) r.max_rel_energy_drift = std::max(r.max_rel_energy_drift, std::abs(s.kinetic_energy - e0) / e0);
        if (h0n > 0.0)
            r.max_rel_momentum_drift = std::max(r.max_rel_momentum_drift, norm(s.angular_momentum_inertial - h0) / h0n);
    }
    return r;
}

void writeCsv(const std::string& path, const std::vector<Sample>& samples) {
    std::ofstream f(path);
    if (!f) throw std::runtime_error("cannot open '" + path + "' for writing");
    f.precision(15);
    f << std::scientific;
    f << "t,qw,qx,qy,qz,wx,wy,wz,tau_x,tau_y,tau_z,energy,hx,hy,hz,pointing_error_deg\n";
    for (const auto& s : samples) {
        f << s.t << ',' << s.state.q.w << ',' << s.state.q.x << ',' << s.state.q.y << ',' << s.state.q.z << ','
          << s.state.w.x << ',' << s.state.w.y << ',' << s.state.w.z << ',' << s.torque.x << ',' << s.torque.y << ','
          << s.torque.z << ',' << s.kinetic_energy << ',' << s.angular_momentum_inertial.x << ','
          << s.angular_momentum_inertial.y << ',' << s.angular_momentum_inertial.z << ',' << s.pointing_error_deg
          << '\n';
    }
}

}  // namespace attitude
