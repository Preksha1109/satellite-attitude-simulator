#pragma once

#include <string>

#include "attitude/rigid_body.hpp"
#include "attitude/state.hpp"
#include "attitude/torque_model.hpp"

namespace attitude {

enum class Method { Euler, RK4 };

Method parseMethod(const std::string& name);  // "euler" | "rk4"; throws std::invalid_argument
const char* toString(Method m);

/// Time derivative of the state:
///   q_dot     = 1/2 q (x) (0, omega)
///   omega_dot = I^-1 ( tau - omega x (I omega) )        (Euler's equations)
struct Derivative {
    Quat dq;
    Vec3 dw;
};
Derivative derivative(double t, const State& s, const RigidBody& body, const TorqueModel& torque);

/// Advance the state by one step of size dt. The quaternion is re-normalised after the step.
State step(const State& s, double t, double dt, const RigidBody& body, const TorqueModel& torque, Method m);

}  // namespace attitude
