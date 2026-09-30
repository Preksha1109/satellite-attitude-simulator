#pragma once

#include "attitude/quaternion.hpp"
#include "attitude/vec3.hpp"

namespace attitude {

/// Attitude state: orientation (body -> inertial) and body-frame angular velocity [rad/s].
struct State {
    Quat q{};
    Vec3 w{};
};

}  // namespace attitude
