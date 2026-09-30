#pragma once

#include <optional>
#include <string>
#include <vector>

#include "attitude/integrator.hpp"

namespace attitude {

struct SimConfig {
    double dt{0.01};                 ///< fixed step [s]
    double t_end{100.0};             ///< end time [s]
    std::size_t log_every{1};        ///< record every N-th step (first and last always recorded)
    Method method{Method::RK4};
    std::optional<Quat> target{};    ///< if set, pointing error to this attitude is logged
};

/// One logged sample.
struct Sample {
    double t{};
    State state{};
    Vec3 torque{};                   ///< applied external torque, body frame [N m]
    double kinetic_energy{};         ///< [J]
    Vec3 angular_momentum_inertial{};///< H = R(q) I omega  [N m s]; constant if torque-free
    double pointing_error_deg{};     ///< NaN if no target
};

std::vector<Sample> simulate(const RigidBody& body, const TorqueModel& torque, const State& initial,
                             const SimConfig& cfg);

/// Drift of the torque-free invariants over a run (relative to their initial values).
struct ConservationReport {
    double max_rel_energy_drift{};
    double max_rel_momentum_drift{};  ///< |H - H0| / |H0| (vector, inertial frame)
};
ConservationReport conservationDrift(const std::vector<Sample>& samples);

/// Write samples as CSV (header + one row per sample).
void writeCsv(const std::string& path, const std::vector<Sample>& samples);

}  // namespace attitude
