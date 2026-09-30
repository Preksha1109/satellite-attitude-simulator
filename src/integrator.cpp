#include "attitude/integrator.hpp"

#include <stdexcept>

namespace attitude {

Method parseMethod(const std::string& name) {
    if (name == "rk4") return Method::RK4;
    if (name == "euler") return Method::Euler;
    throw std::invalid_argument("unknown integrator '" + name + "' (expected rk4 or euler)");
}

const char* toString(Method m) { return m == Method::RK4 ? "rk4" : "euler"; }

Derivative derivative(double t, const State& s, const RigidBody& body, const TorqueModel& torque) {
    const Vec3 tau = torque.torque(t, s, body);
    const Vec3 Iw = body.inertia() * s.w;
    Derivative d;
    d.dq = 0.5 * (s.q * Quat(0.0, s.w.x, s.w.y, s.w.z));
    d.dw = body.inverseInertia() * (tau - cross(s.w, Iw));
    return d;
}

static State advance(const State& s, const Derivative& d, double h) {
    return {s.q + h * d.dq, s.w + h * d.dw};
}

State step(const State& s, double t, double dt, const RigidBody& body, const TorqueModel& torque, Method m) {
    State out;
    if (m == Method::Euler) {
        out = advance(s, derivative(t, s, body, torque), dt);
    } else {
        const Derivative k1 = derivative(t, s, body, torque);
        const Derivative k2 = derivative(t + 0.5 * dt, advance(s, k1, 0.5 * dt), body, torque);
        const Derivative k3 = derivative(t + 0.5 * dt, advance(s, k2, 0.5 * dt), body, torque);
        const Derivative k4 = derivative(t + dt, advance(s, k3, dt), body, torque);
        out.q = s.q + (dt / 6.0) * (k1.dq + 2.0 * k2.dq + 2.0 * k3.dq + k4.dq);
        out.w = s.w + (dt / 6.0) * (k1.dw + 2.0 * k2.dw + 2.0 * k3.dw + k4.dw);
    }
    out.q = out.q.normalized();
    return out;
}

}  // namespace attitude
