# Magnetorquer and Reaction Wheel Models

## Overview

This document describes the magnetorquer and reaction wheel models added to the satellite attitude simulator. These models enable realistic simulation of attitude control systems for spacecraft in low Earth orbit.

---

## 1. Magnetorquer Model

### Physics

A magnetorquer (magnetic torque rod) produces torque by interacting with Earth's magnetic field:

```
τ = m × B
```

Where:
- **τ** = torque vector (N·m)
- **m** = magnetic dipole moment (A·m²)
- **B** = magnetic field vector (Tesla)

### Key Features

**Direct Control Mode**
```
m = m_command
|m| ≤ m_max
```

**B-dot Detumbling (Passive)**
```
m = -k_bdot * dB/dt
```

The B-dot controller provides passive detumbling without requiring rate gyroscope measurements. The controller responds to the time derivative of the magnetic field, which correlates with angular velocity in orbit.

### Class: `Magnetorquer`

```cpp
class Magnetorquer {
    // Create with parameters
    Magnetorquer(double m_max = 0.5,        // max dipole (A·m²)
                 double k_bdot = 0.05,      // B-dot gain
                 bool use_bdot = true);     // enable auto B-dot
    
    // Update B-dot estimate (call once per time step)
    void updateBdot(const Vec3& B_field, double dt);
    
    // Compute torque
    Vec3 computeTorque(const Vec3& B_field) const;
    
    // Direct command mode
    void setMagneticMoment(const Vec3& m);
    
    // Enable/disable B-dot
    void enableBdot(bool enable);
};
```

### Typical Parameters

| Parameter | Value | Units |
|-----------|-------|-------|
| Max dipole moment | 0.1 - 1.0 | A·m² |
| B-dot gain | 0.01 - 0.1 | A·m²·s/T |
| Earth's field (LEO) | 24-30 | µT |
| Detumbling time | 500-2000 | s |

### Usage Example

```cpp
// Create magnetorquer with B-dot control
Magnetorquer mag(0.5, 0.05, true);

// In simulation loop:
Vec3 B_field = getOrbitalMagneticField(state.position, t);
mag.updateBdot(B_field, dt);
Vec3 tau_mag = mag.computeTorque(B_field);
```

### B-dot Controller Details

**How it works:**
1. Measure/estimate magnetic field **B(t)** from magnetometer or model
2. Compute numerical derivative: dB/dt = (B(t) - B(t-dt)) / dt
3. Command dipole moment: **m** = -k_bdot * **dB/dt**
4. Produce torque: **τ** = **m** × **B**

**Advantages:**
- ✓ Passive (no active power, only magnetic field interaction)
- ✓ No rate gyroscope needed
- ✓ Inherently stable
- ✓ Works for arbitrary initial tumble rates

**Disadvantages:**
- ✗ Slow (500-2000 seconds typical)
- ✗ Only works in orbits with significant B field
- ✗ Damping proportional to orbital velocity

---

## 2. Reaction Wheel Model

### Physics

A reaction wheel produces torque by changing its angular momentum:

```
τ = dh/dt
h = I_wheel * ω_wheel
```

Where:
- **τ** = output torque (N·m)
- **h** = angular momentum stored in wheel (N·m·s)
- **I_wheel** = wheel moment of inertia (kg·m²)
- **ω_wheel** = wheel spin rate (rad/s)

### Constraints

1. **Torque limit**: |τ| ≤ τ_max
2. **Momentum limit**: |h| ≤ h_max (saturation)
3. **Speed limit**: |ω| ≤ ω_max (derived from h_max)

### Class: `ReactionWheel`

```cpp
class ReactionWheel {
    // Create wheel on principal axis
    ReactionWheel(int axis = 2,             // 0=x, 1=y, 2=z
                  double I_wheel = 0.1,    // kg·m²
                  double tau_max = 0.01,   // N·m
                  double h_max = 0.2);     // N·m·s
    
    // Command torque
    void setTorqueCommand(double tau);
    
    // Get torque output (with integration)
    Vec3 getTorque(double dt);
    
    // State queries
    double getMomentum() const;
    double getSpinRate() const;
    bool isSaturated() const;
    
    // Desaturation (apply external torque)
    void applyExternalTorque(double tau_ext, double dt);
};
```

### Class: `ReactionWheelCluster`

For 3-axis attitude control, use a cluster of wheels:

```cpp
class ReactionWheelCluster {
    // Create 3 wheels (typically aligned Z, Y, X)
    ReactionWheelCluster(int num_wheels = 3);
    
    // Command 3-axis torque
    void setTorqueCommand(const Vec3& tau);
    void setTorqueCommand(double tau_x, double tau_y, double tau_z);
    
    // Get total torque and momentum
    Vec3 getTorque(double dt);
    Vec3 getTotalMomentum() const;
    
    // Momentum management
    bool isAnySaturated() const;
    void performDesaturation(const Vec3& tau_ext, double dt);
};
```

### Typical Parameters (0.1 kg·m² wheel)

| Parameter | Value | Units |
|-----------|-------|-------|
| Moment of inertia | 0.1 | kg·m² |
| Max torque | 0.005-0.02 | N·m |
| Max momentum | 0.1-0.5 | N·m·s |
| Max speed | 5000-10000 | RPM |
| Settling time (attitude) | 50-200 | s |

### Momentum Saturation

Reaction wheels accumulate angular momentum over time. When saturated:

```
|h| = h_max
```

At saturation:
- Cannot produce torque to increase momentum further
- Can still command torque toward zero (to desaturate)
- Wheel speed is at maximum: ω_max = h_max / I_wheel

**Desaturation methods:**
1. **Magnetorquer**: Apply external torque via magnetic field
2. **Thrusters**: Produce reaction torque
3. **Gravity gradient**: Use orbital dynamics

---

## 3. Hybrid Control Architecture

### Class: `HybridAttitudeControl`

Combines reaction wheels with magnetorquer for:
- Fine pointing (reaction wheels)
- Momentum desaturation (magnetorquer)
- Passive detumbling (B-dot)

```cpp
class HybridAttitudeControl : public TorqueModel {
    // Create hybrid controller
    HybridAttitudeControl(double tau_rw = 0.01,
                         double h_rw = 0.2,
                         double m_mag = 0.5,
                         double k_bdot = 0.05);
    
    // Set commands
    void setWheelTorque(const Vec3& tau);
    void setMagneticMoment(const Vec3& m);
    void setMagneticField(const Vec3& B);
    
    // Query state
    Vec3 getWheelMomentum() const;
    bool isWheelSaturated() const;
    bool isDesaturating() const;
    
    // Enable/disable subsystems
    void enableReactionWheels(bool enable);
    void enableMagnetorquer(bool enable);
    void enableBdot(bool enable);
};
```

### Control Logic Example

```cpp
HybridAttitudeControl ctrl;

if (t < 300) {
    // Phase 1: Detumbling (B-dot passive)
    ctrl.enableReactionWheels(false);
    ctrl.enableMagnetorquer(true);
    ctrl.enableBdot(true);
    
} else if (t < 600) {
    // Phase 2: Fine pointing (reaction wheels)
    ctrl.enableReactionWheels(true);
    ctrl.enableMagnetorquer(false);
    Vec3 tau_cmd(0.002, 0.003, 0.001);
    ctrl.setWheelTorque(tau_cmd);
    
} else {
    // Phase 3: Desaturation (wheels + magnetorquer)
    ctrl.enableReactionWheels(true);
    ctrl.enableMagnetorquer(true);
    ctrl.enableBdot(true);
    // Magnetorquer automatically desaturates wheels
}

// In simulation loop:
Vec3 B = getOrbitalMagField(state, t);
ctrl.setMagneticField(B);
Vec3 tau = ctrl.computeTorque(state, t);
```

---

## 4. Simulation Scenarios

### Scenario 1: B-dot Detumbling

**Initial conditions:**
- Spin rate: 30°/s (deployment tumble)
- No attitude constraints

**Expected results:**
- Exponential spin-down over ~500 seconds
- Final spin rate: <0.1°/s
- No momentum accumulation (passive)

**Output files:**
- Spin rate vs time (should show exponential decay)
- Magnetic torque magnitude
- Angular momentum conservation check

### Scenario 2: Fine Pointing with Reaction Wheels

**Initial conditions:**
- Starting from <1°/s spin rate
- 3-axis attitude control

**Control objectives:**
- Point sun panel to sun
- Maintain <0.1°/s error
- Minimize momentum buildup

**Expected results:**
- Settling time: 50-200 seconds
- Pointing accuracy: <0.5°
- Momentum grows linearly with control duration

### Scenario 3: Hybrid Mission Profile

**Three phases:**
1. **Detumbling (0-300s)**: B-dot passive control
2. **Fine pointing (300-600s)**: Reaction wheel control
3. **Desaturation (600-1200s)**: Wheels + magnetorquer

**Example output:**
```
t=0s    | Phase: DETUMBLING | ω = 30.0°/s | τ = 150 µN·m | h = 0 mN·m·s
t=100s  | Phase: DETUMBLING | ω = 15.2°/s | τ = 78 µN·m  | h = 0 mN·m·s
t=300s  | Phase: FINE_POINTING | ω = 0.5°/s | τ = 50 µN·m | h = 2.1 mN·m·s
t=600s  | Phase: DESATURATING | ω = 0.1°/s | τ = 30 µN·m | h = 1.8 mN·m·s
t=1200s | Phase: DESATURATING | ω = 0.05°/s | τ = 15 µN·m | h = 0.5 mN·m·s
```

---

## 5. Validation & Analysis

### Tests Provided

| Test | Validates |
|------|-----------|
| `test_magnetorquer_creation` | Parameter initialization |
| `test_magnetorquer_saturation` | Dipole moment limiting |
| `test_bdot_torque_computation` | B-dot controller torque |
| `test_reaction_wheel_basic` | Wheel initialization |
| `test_reaction_wheel_torque` | Torque-momentum relationship |
| `test_reaction_wheel_saturation` | Momentum limiting |
| `test_reaction_wheel_desaturation` | Momentum reduction |
| `test_reaction_wheel_cluster` | 3-axis operation |
| `test_bdot_detumble_scenario` | Realistic detumbling |
| `test_hybrid_control` | Hybrid architecture |

### Running Tests

```bash
cd satellite-attitude-simulator
mkdir build && cd build
cmake ..
make test_magnetorquer_rw
./test_magnetorquer_rw
```

Expected output:
```
=== Magnetorquer and Reaction Wheel Tests ===

Test 1: Magnetorquer creation and parameters
  ✓ Magnetorquer parameters correct
Test 2: Magnetorquer dipole moment saturation
  ✓ Dipole moment saturates to 0.5 A·m²
...
=== All tests passed! ===
```

### Example Scenarios

```bash
# Run hybrid control mission profile
./example_hybrid_control
```

Generates:
- `hybrid_control_state.csv` - Spin rate and attitude
- `hybrid_control_commands.csv` - Control commands and phase
- `hybrid_control_momentum.csv` - Reaction wheel momentum

---

## 6. Integration with Existing Models

### Compatible with:

- ✓ All existing torque models (no torque, constant torque, gravity gradient, PD controller)
- ✓ All integration methods (Euler, RK4)
- ✓ All dynamics models (can be used with any aircraft or satellite model)
- ✓ Existing test suite and CSV output

### Adding to Your Simulation

```cpp
#include "attitude/magnetorquer.hpp"
#include "attitude/reaction_wheel.hpp"
#include "attitude/torque_models_advanced.hpp"

// In your simulation:
HybridAttitudeControl control;

for (int i = 0; i < num_steps; i++) {
    // ... set commands, update magnetic field ...
    Vec3 tau = control.computeTorque(state, t);
    state = integrate(state, tau, dt);
}
```

---

## 7. Physical Constants & Typical Values

### Earth's Magnetic Field (Dipole Model)

| Location | Field Strength | Direction |
|----------|----------------|-----------|
| Equator | 30 µT | Horizontal |
| 45° latitude | 35 µT | 45° inclination |
| Pole | 60 µT | Vertical |
| LEO (500 km) | 24-30 µT | Varies |

### Reaction Wheel Characteristics

**Small CubeSat (0.1 kg·m²):**
- Max torque: 5-20 mN·m
- Max momentum: 100-500 mN·m·s
- Max speed: 5000-15000 RPM

**Large satellite (>100 kg·m²):**
- Max torque: 50-500 mN·m
- Max momentum: 1-10 N·m·s
- Max speed: 2000-6000 RPM

### Magnetorquer Specifications

**Passive rods (bar magnets):**
- Dipole moment: 0.01-0.1 A·m²
- Power: 0 (passive)
- Response: Slow (~minutes)

**Active electromagnets:**
- Dipole moment: 0.5-10 A·m²
- Power: 1-10 W
- Response: Fast (~seconds)

---

## 8. References

### Magnetorquer Control

- Hughes, P. C., *Spacecraft Attitude Dynamics*, Wiley, 1986
  - Chapter 9: Magnetic torque rods
- Wertz, J. R., *Spacecraft Attitude Determination and Control*, Kluwer, 1978
  - B-dot controller design

### Reaction Wheels

- Sidi, M. J., *Spacecraft Dynamics and Control*, AIAA, 1997
  - Chapter 6: Momentum exchange devices
- Wie, B., *Space Vehicle Dynamics and Control*, AIAA, 2008
  - Reaction wheel modeling and momentum management

### Hybrid Control

- Tekinalp, Ö. & Soken, H. E.
  - "Passive Magnetic Attitude Stabilization of Satellites"
  - *IEEE Trans. Aerospace Electronic Systems*, 2012
- Psiaki, M. L.
  - "Magnetic Torquers for Momentum Desaturation"
  - *Journal of Guidance, Control, and Dynamics*, 2004

---

## 9. Troubleshooting

### Magnetorquer

**Q: B-dot not working effectively?**
- Check: Is B field variation large enough?
- Check: Is k_bdot gain appropriate for your B field strength?
- Check: Are you updating B-dot estimate each time step?

**Q: Detumbling too slow?**
- Increase k_bdot (higher gain = faster response)
- Ensure magnetic field is varying (orbital dynamics)
- Check: Are you saturating the dipole moment?

### Reaction Wheels

**Q: Momentum growing too fast?**
- Reduce wheel torque commands
- Implement better control law to reduce accumulated momentum
- Activate desaturation sooner

**Q: Wheels saturating?**
- Enable magnetorquer desaturation
- Increase h_max (use larger wheels)
- Implement momentum bias (rotate wheels at bias rate)

### Hybrid System

**Q: Transition between phases unstable?**
- Reduce wheel torques before transition
- Ramp commands smoothly instead of step changes
- Ensure wheels have <50% momentum at transition

---

## 10. Future Enhancements

Possible extensions to these models:

- [ ] Magnetic field from geomagnetic model (IGRF)
- [ ] Wheel bearing friction and creep
- [ ] Magnetorquer power consumption modeling
- [ ] Gravity gradient desaturation
- [ ] Momentum bias wheel operation
- [ ] Control moment gyroscope (CMG) models
- [ ] Realistic orbital propagation
- [ ] Hardware-in-the-loop interface

---

**Last updated**: September 2026
