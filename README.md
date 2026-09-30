# Satellite Attitude Dynamics Simulator

![CI](https://github.com/Preksha1109/satellite-attitude-simulator/actions/workflows/ci.yml/badge.svg)

A modular C++17 simulator for the rotational dynamics of a rigid-body satellite. Attitude is propagated with quaternions and Euler's equations using a fourth-order Runge-Kutta integrator. Disturbance and control torques plug in through a single interface, so new models can be added without changing the simulation core. No external dependencies.

![Tumbling spin about the intermediate axis, and energy conservation of RK4 vs Euler](docs/tumble.png)

*Torque-free spin about the intermediate axis of a 6U-class CubeSat (left): the spin axis flips sign three times in 200 s. RK4 conserves kinetic energy to ~1e-14 through the flips; explicit Euler drifts by 1% (right).*

## Physics model

The satellite is a rigid body with body-frame inertia tensor **I**. State is the attitude quaternion **q** and body-frame angular velocity **ω**:

$$
\dot{\mathbf{q}} = \tfrac{1}{2}\,\mathbf{q}\otimes(0,\boldsymbol{\omega}),
\qquad
\mathbf{I}\,\dot{\boldsymbol{\omega}} = \boldsymbol{\tau} - \boldsymbol{\omega}\times(\mathbf{I}\boldsymbol{\omega})
$$

where **τ** is the total external torque. For torque-free motion two quantities are conserved and are used as correctness checks: the inertial-frame angular-momentum vector **H** = R(**q**)·**Iω**, and the rotational kinetic energy ½ **ω**ᵀ**Iω**.

**Conventions.** SI units. Quaternions are Hamilton, scalar-first (w, x, y, z), and `q` rotates vectors from the body frame to the inertial frame. **ω** and all torques are expressed in the body frame.

## Features

- Quaternion attitude propagation (re-normalised each step) and Euler's rigid-body equations
- Fixed-step **RK4** integrator, with explicit Euler available for comparison
- `TorqueModel` interface with ready-made models:
  - `NoTorque`, `ConstantTorque`
  - `GravityGradient`: circular-orbit gravity-gradient disturbance
  - `PDController`: quaternion-feedback PD attitude controller with per-axis torque saturation
  - `CompositeTorque`: sum of any number of the above
- Input validation (inertia tensor must be symmetric positive-definite)
- CSV output for every run; Python script to plot the results
- Unit tests that check the physics against analytic solutions and conservation laws

## Architecture

```
              +-----------------+
              |  simulate()     |   fixed-step loop, logging, diagnostics
              +--------+--------+
                       | step()  (Euler / RK4)
              +--------v--------+
              |  derivative()   |   q_dot, omega_dot from Euler's equations
              +--------+--------+
                       | torque(t, state, body)
        +--------------v---------------+
        |   TorqueModel  (interface)   |
        +--+-------+---------+-------+-+
           |       |         |       |
       NoTorque  Constant  GravityGradient  PDController  ... your own
                            (all combinable with CompositeTorque)
```

### Adding your own torque model

Derive from `TorqueModel` and implement one function. The full runnable version is in [`examples/custom_torque.cpp`](examples/custom_torque.cpp).

```cpp
class SinusoidalDisturbance final : public TorqueModel {
public:
    SinusoidalDisturbance(const Vec3& amplitude, double omega)
        : a_(amplitude), w_(omega) {}
    Vec3 torque(double t, const State&, const RigidBody&) const override {
        return std::sin(w_ * t) * a_;
    }
private:
    Vec3 a_;
    double w_;
};

CompositeTorque total;
total.add(std::make_shared<SinusoidalDisturbance>(Vec3{1e-4, 0, 2e-4}, 0.5));
total.add(std::make_shared<PDController>(target, 0.016, 0.06, 2e-3));
auto samples = simulate(body, total, initial_state, cfg);
```

## Build and run

Requires CMake (3.14+) and a C++17 compiler.

```bash
cmake -S . -B build
cmake --build build
ctest --test-dir build --output-on-failure      # run the tests
./build/attitude_sim tumble                      # writes data/tumble.csv
```

Scenarios: `tumble`, `slew`, `detumble`, `gravity`. Options: `--integrator rk4|euler`, `--dt <s>`, `--t-end <s>`, `--out <file.csv>`.

To regenerate the figures in this README:

```bash
./build/attitude_sim tumble --out data/tumble_rk4.csv
./build/attitude_sim tumble --integrator euler --out data/tumble_euler.csv
./build/attitude_sim slew && ./build/attitude_sim detumble && ./build/attitude_sim gravity
pip install numpy matplotlib
python scripts/plot_results.py
```

CSV columns: `t, qw..qz, wx..wz, tau_x..tau_z, energy, hx..hz, pointing_error_deg`.

## Validation

All of these are automated in [`tests/test_attitude.cpp`](tests/test_attitude.cpp) and run in CI. Values below are measured, not targets.

| Check | Result |
|---|---|
| Axisymmetric torque-free body vs. closed-form solution (100 s, dt = 0.01 s) | max error 2.9e-13 rad/s |
| Kinetic-energy drift, chaotic torque-free tumble (1000 s) | 6.4e-14 (relative) |
| Angular-momentum drift, same run | 7.2e-13 (relative) |
| Order of convergence (halving dt) | error ratio 16.03, i.e. 4th order |
| RK4 vs. explicit Euler energy drift (100 s) | 4.3e-14 vs. 5.7e-3 |
| Constant torque from rest vs. exact ω = τt/I | error 3.9e-14 rad/s |
| Gravity-gradient torque: zero when a principal axis is radial; magnitude at 45° tilt | matches analytic to 1.8e-16 (relative) |
| PD controller: 60° slew, 300 s | pointing error and rate converge to numerical zero |

## Scenarios

All scenarios use a 6U-class CubeSat: roughly 12 kg, a uniform 0.100 × 0.226 × 0.366 m box, principal moments (0.185, 0.144, 0.061) kg·m². The intermediate axis is y.

| Scenario | What it shows | Result (RK4, dt = 0.01 s) |
|---|---|---|
| `tumble` | Torque-free spin about the intermediate axis, ω = (0.005, 0.30, 0.005) rad/s | ω<sub>y</sub> flips sign 3 times in 200 s; drift of **H**: 6.1e-13, of energy: 5.6e-14. Explicit Euler: 5.0e-3 and 1.0e-2 |
| `slew` | 60° slew about (1,1,1) from rest, PD gains kp = 0.016, kd = 0.06, ±2 mN·m limit | within 1° at 27 s; torque limit active only during the first 3 s |
| `detumble` | Start at 6.4°/s and a 30° attitude offset, PD control to the identity attitude | \|ω\| < 1e-3 rad/s by 32.5 s; pointing error < 1° by 26 s |
| `gravity` | Uncontrolled, 525 km circular orbit (one period) | peak gravity-gradient torque 2.2e-7 N·m; peak rate 0.12°/s |

![60 degree slew with torque saturation](docs/slew.png)

![Detumble](docs/detumble.png)

![Gravity-gradient torque](docs/gravity.png)

## Project structure

```
include/attitude/   Vec3/Mat3, Quat, RigidBody, State, TorqueModel, integrator and simulator interfaces
src/                torque models, integrator, simulator, command-line front end (main.cpp)
tests/              analytic-solution, conservation and convergence tests
examples/           custom_torque.cpp: adding a new torque model
scripts/            plot_results.py
data/               sample CSV output from the scenarios above
docs/               figures used in this README
```

## Limitations

- Rigid body only: no flexible appendages, fuel slosh or wheel dynamics
- The reaction-wheel model is a per-axis torque clamp. It has no wheel momentum, saturation of stored momentum or bandwidth
- `GravityGradient` assumes a point-mass Earth and a fixed circular orbit in the inertial x-y plane; the orbit is not propagated from the attitude simulation
- Fixed-step integration only; the PD gains are shared across axes and hand-tuned for this inertia
- No sensor models (gyro noise, star tracker) yet

## Roadmap

- [ ] Sensor models (gyro bias and noise, star-tracker error) and a state estimator
- [ ] Reaction-wheel momentum storage and desaturation (magnetic torquers, B-dot detumbling)
- [ ] Adaptive-step integrator (RK45) with error control
- [ ] Scenario configuration from files instead of built-in scenarios
- [ ] Additional disturbances: aerodynamic drag, solar radiation pressure, residual magnetic dipole
