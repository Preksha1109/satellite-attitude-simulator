/**
 * Unit Tests: Magnetorquer and Reaction Wheel Models
 * 
 * Validates:
 *   - B-dot detumbling physics
 *   - Reaction wheel torque output
 *   - Momentum saturation
 *   - Desaturation control
 */

#include "../include/attitude/magnetorquer.hpp"
#include "../include/attitude/reaction_wheel.hpp"
#include "../include/attitude/torque_models_advanced.hpp"
#include <cmath>
#include <cassert>
#include <iostream>

void test_magnetorquer_creation() {
    std::cout << "Test 1: Magnetorquer creation and parameters\n";
    
    Magnetorquer mag(0.5, 0.05, true);
    
    assert(mag.getMaxDipoleMoment() == 0.5);
    assert(mag.getBdotGain() == 0.05);
    assert(mag.isBdotEnabled() == true);
    
    std::cout << "  ✓ Magnetorquer parameters correct\n";
}

void test_magnetorquer_saturation() {
    std::cout << "Test 2: Magnetorquer dipole moment saturation\n";
    
    Magnetorquer mag(0.5, 0.05, false);  // Direct control, no B-dot
    
    // Command dipole moment > max
    Vec3 m_too_large(1.0, 1.0, 1.0);
    mag.setMagneticMoment(m_too_large);
    
    Vec3 m = mag.getMagneticMoment(Vec3(0, 0, 0));
    double mag_norm = m.norm();
    
    // Should saturate to m_max
    assert(std::abs(mag_norm - 0.5) < 1e-6);
    
    std::cout << "  ✓ Dipole moment saturates to " << mag_norm << " A·m²\n";
}

void test_bdot_torque_computation() {
    std::cout << "Test 3: B-dot torque from magnetic field change\n";
    
    Magnetorquer mag(0.5, 0.05, true);  // B-dot enabled
    
    // Simulate changing magnetic field (e.g., from orbital dynamics)
    Vec3 B1(0, 0, 25e-6);  // 25 µT pointing down
    Vec3 B2(0, 0, 24e-6);  // Slightly weaker
    
    mag.updateBdot(B1, 0.01);  // First update
    mag.updateBdot(B2, 0.01);  // Second update (now dB/dt is computed)
    
    Vec3 tau = mag.computeTorque(B2);
    
    // Torque should be non-zero because dB/dt is non-zero
    double tau_mag = tau.norm();
    assert(tau_mag > 1e-10);
    
    std::cout << "  ✓ B-dot torque magnitude: " << tau_mag * 1e6 << " µN·m\n";
}

void test_reaction_wheel_basic() {
    std::cout << "Test 4: Reaction wheel basic operation\n";
    
    ReactionWheel wheel(2, 0.1, 0.01, 0.2);  // Z-axis wheel
    
    assert(wheel.getAxis() == 2);
    assert(wheel.getMaxTorque() == 0.01);
    assert(wheel.getMaxMomentum() == 0.2);
    assert(wheel.getMomentum() == 0);
    assert(wheel.isSaturated() == false);
    
    std::cout << "  ✓ Reaction wheel initialized correctly\n";
}

void test_reaction_wheel_torque() {
    std::cout << "Test 5: Reaction wheel torque and momentum integration\n";
    
    ReactionWheel wheel(2, 0.1, 0.01, 0.2);  // Z-axis
    double dt = 0.01;  // 100 Hz
    
    wheel.setTorqueCommand(0.005);  // Command 5 mN·m
    
    Vec3 tau = wheel.getTorque(dt);
    assert(tau[2] == 0.005);  // Torque along z-axis
    
    // Momentum should increase: h = τ * dt
    double h_expected = 0.005 * dt;
    double h_actual = wheel.getMomentum();
    
    assert(std::abs(h_actual - h_expected) < 1e-10);
    
    std::cout << "  ✓ Momentum increased: " << h_actual * 1e6 << " µN·m·s\n";
}

void test_reaction_wheel_saturation() {
    std::cout << "Test 6: Reaction wheel momentum saturation\n";
    
    ReactionWheel wheel(2, 0.1, 0.01, 0.2);  // Max momentum = 0.2 N·m·s
    double dt = 0.01;
    
    // Spin up to near saturation
    for (int i = 0; i < 19; i++) {
        wheel.setTorqueCommand(0.01);
        wheel.getTorque(dt);
    }
    
    // Should be near saturation
    assert(wheel.isSaturated() == false);
    double h_before = wheel.getMomentum();
    assert(h_before < 0.2);
    
    // Try to increase beyond limit
    wheel.setTorqueCommand(0.01);
    wheel.getTorque(dt);
    
    // Momentum should be clamped at h_max
    double h_after = wheel.getMomentum();
    assert(std::abs(h_after - 0.2) < 1e-10);
    assert(wheel.isSaturated() == true);
    
    std::cout << "  ✓ Momentum saturated at " << h_after << " N·m·s\n";
}

void test_reaction_wheel_desaturation() {
    std::cout << "Test 7: Reaction wheel desaturation\n";
    
    ReactionWheel wheel(2, 0.1, 0.01, 0.2);
    double dt = 0.01;
    
    // Spin up to saturation
    wheel.setTorqueCommand(0.01);
    for (int i = 0; i < 20; i++) {
        wheel.getTorque(dt);
    }
    
    double h_saturated = wheel.getMomentum();
    assert(wheel.isSaturated() == true);
    
    // Apply external desaturation torque (negative)
    wheel.applyExternalTorque(-0.005, dt);
    
    double h_after = wheel.getMomentum();
    assert(h_after < h_saturated);
    
    std::cout << "  ✓ Momentum reduced from " << h_saturated 
              << " to " << h_after << " N·m·s\n";
}

void test_reaction_wheel_cluster() {
    std::cout << "Test 8: 3-axis reaction wheel cluster\n";
    
    ReactionWheelCluster cluster(3);
    
    assert(cluster.getNumWheels() == 3);
    
    // Command torque on each axis
    cluster.setTorqueCommand(0.005, 0.007, 0.003);
    
    Vec3 tau = cluster.getTorque(0.01);
    assert(std::abs(tau[0] - 0.005) < 1e-10);
    assert(std::abs(tau[1] - 0.007) < 1e-10);
    assert(std::abs(tau[2] - 0.003) < 1e-10);
    
    // Check momentum
    Vec3 h = cluster.getTotalMomentum();
    double h_mag = h.norm();
    
    std::cout << "  ✓ Cluster torque: [" << tau[0] << ", " << tau[1] 
              << ", " << tau[2] << "] N·m\n";
    std::cout << "  ✓ Total momentum: " << h_mag * 1e3 << " mN·m·s\n";
}

void test_bdot_detumble_scenario() {
    std::cout << "Test 9: B-dot detumbling scenario\n";
    
    // Initial spin rate: 30 deg/s
    double omega_initial = 30 * M_PI / 180.0;  // rad/s
    
    // Simulate detumbling over 100 seconds
    BdotDetumbleTorque detumbler(0.5, 0.1);
    State state;
    state.q = Quat();  // Identity quaternion
    state.w = Vec3(0, omega_initial, 0);  // Spin about Y
    
    // Magnetic field varying with rotation
    Vec3 B_equatorial(0, 0, 25e-6);  // Equatorial: field mostly vertical
    
    double t_sim = 0;
    double dt = 0.01;
    int steps = 10000;  // 100 seconds
    
    double tau_mean = 0;
    for (int i = 0; i < steps; i++) {
        // Update B-dot with changing field (simplified model)
        // In reality, B-dot varies with orbital position
        Vec3 B_sim = B_equatorial * (1.0 + 0.1 * std::sin(2 * M_PI * i / 100.0));
        detumbler.setMagneticField(B_sim);
        
        Vec3 tau = detumbler.computeTorque(state, t_sim);
        tau_mean += tau.norm() / steps;
        t_sim += dt;
    }
    
    std::cout << "  ✓ Mean detumbling torque: " << tau_mean * 1e6 << " µN·m\n";
    std::cout << "  ✓ Over 100 seconds of simulation\n";
}

void test_hybrid_control() {
    std::cout << "Test 10: Hybrid reaction wheel + magnetorquer control\n";
    
    HybridAttitudeControl hybrid(0.01, 0.2, 0.5, 0.05);
    State state;
    
    // Command fine pointing with reaction wheels
    hybrid.setWheelTorque(Vec3(0.003, 0.005, 0.002));
    
    // Set initial wheel momentum
    Vec3 tau_rw = hybrid.computeTorque(state, 0.0);
    
    // Set magnetic field
    hybrid.setMagneticField(Vec3(0, 0, 25e-6));
    
    // Enable B-dot for desaturation
    hybrid.enableBdot(true);
    
    // Simulate for several seconds
    double t = 0;
    double dt = 0.01;
    for (int i = 0; i < 100; i++) {
        Vec3 tau = hybrid.computeTorque(state, t);
        
        if (i % 20 == 0) {
            Vec3 h_wheels = hybrid.getWheelMomentum();
            std::cout << "  t=" << t << "s, wheel momentum: " << h_wheels.norm() * 1e3 
                      << " mN·m·s\n";
        }
        t += dt;
    }
    
    std::cout << "  ✓ Hybrid control simulation complete\n";
}

int main() {
    std::cout << "\n=== Magnetorquer and Reaction Wheel Tests ===\n\n";
    
    test_magnetorquer_creation();
    test_magnetorquer_saturation();
    test_bdot_torque_computation();
    test_reaction_wheel_basic();
    test_reaction_wheel_torque();
    test_reaction_wheel_saturation();
    test_reaction_wheel_desaturation();
    test_reaction_wheel_cluster();
    test_bdot_detumble_scenario();
    test_hybrid_control();
    
    std::cout << "\n=== All tests passed! ===\n\n";
    return 0;
}
