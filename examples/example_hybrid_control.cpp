/**
 * Example: Hybrid Attitude Control Scenario
 * 
 * Demonstrates:
 *   1. Initial B-dot detumbling from deployment spin
 *   2. Transition to reaction wheel control for fine pointing
 *   3. Momentum management and desaturation
 *   
 * Spacecraft: 6U CubeSat with 3-axis reaction wheels + magnetorquer
 * Mission profile:
 *   - t=0-300s: Detumbling phase (B-dot passive control)
 *   - t=300-600s: Fine pointing (reaction wheels)
 *   - t=600+: Momentum desaturation (magnetorquer)
 */

#include "../include/attitude/vec3.hpp"
#include "../include/attitude/quaternion.hpp"
#include "../include/attitude/rigid_body.hpp"
#include "../include/attitude/state.hpp"
#include "../include/attitude/integrator.hpp"
#include "../include/attitude/torque_models_advanced.hpp"
#include <iostream>
#include <fstream>
#include <cmath>
#include <iomanip>

const double DEG2RAD = M_PI / 180.0;
const double RAD2DEG = 180.0 / M_PI;

int main() {
    std::cout << "\n=== Hybrid Attitude Control Scenario ===\n\n";
    
    // ============================================================================
    // SPACECRAFT MODEL (6U CubeSat)
    // ============================================================================
    std::cout << "Initializing spacecraft...\n";
    
    // Principal moments of inertia (6U CubeSat)
    Vec3 principal_moments(0.185, 0.144, 0.061);  // kg·m²
    RigidBody satellite = RigidBody::fromPrincipalMoments(principal_moments);
    
    std::cout << "  Inertia tensor:\n";
    const Mat3& I = satellite.getInertia();
    std::cout << "    [" << I(0,0) << ", " << I(0,1) << ", " << I(0,2) << "]\n";
    std::cout << "    [" << I(1,0) << ", " << I(1,1) << ", " << I(1,2) << "]\n";
    std::cout << "    [" << I(2,0) << ", " << I(2,1) << ", " << I(2,2) << "]\n";
    
    // ============================================================================
    // ATTITUDE CONTROL SYSTEM
    // ============================================================================
    std::cout << "\nInitializing attitude control system...\n";
    
    // Create hybrid controller
    // Reaction wheels: 0.01 N·m max, 0.2 N·m·s max momentum each
    // Magnetorquer: 0.5 A·m² max dipole moment, 0.05 gain for B-dot
    HybridAttitudeControl controller(0.01, 0.2, 0.5, 0.05);
    
    std::cout << "  Reaction wheels: 3-axis cluster\n";
    std::cout << "    Max torque: 0.01 N·m per axis\n";
    std::cout << "    Max momentum: 0.2 N·m·s per wheel\n";
    std::cout << "  Magnetorquer:\n";
    std::cout << "    Max dipole moment: 0.5 A·m²\n";
    std::cout << "    B-dot gain: 0.05 A·m²·s/T\n";
    
    // ============================================================================
    // INITIAL CONDITIONS
    // ============================================================================
    std::cout << "\nSetting initial conditions...\n";
    
    State state;
    state.q = Quat();  // Identity quaternion (nadir-pointing)
    
    // Initial spin: 30°/s about each axis (realistic deployment tumble)
    state.w = Vec3(30, 25, 20) * DEG2RAD;
    
    std::cout << "  Initial attitude: Nadir-pointing\n";
    std::cout << "  Initial spin rate: [" 
              << 30 << ", " << 25 << ", " << 20 << "] °/s\n";
    
    double omega_mag = state.w.norm() * RAD2DEG;
    std::cout << "  Total spin rate magnitude: " << omega_mag << " °/s\n";
    
    // Magnetic field (LEO circular orbit, 500 km altitude)
    // Approximate dipole field strength: 25-30 µT depending on location
    Vec3 B_orbit(5e-6, 10e-6, 24e-6);  // µT in body frame, tilted
    
    // ============================================================================
    // SIMULATION PARAMETERS
    // ============================================================================
    double dt = 0.01;  // 100 Hz control rate
    double t_total = 1200.0;  // 20 minutes
    int num_steps = (int)(t_total / dt);
    
    // Phase durations
    double t_detumble_end = 300.0;      // Detumbling: 0-5 min
    double t_pointing_end = 600.0;      // Fine pointing: 5-10 min
    // Desaturation: 10-20 min
    
    // ============================================================================
    // OUTPUT FILES
    // ============================================================================
    std::ofstream file_state("hybrid_control_state.csv");
    std::ofstream file_control("hybrid_control_commands.csv");
    std::ofstream file_momentum("hybrid_control_momentum.csv");
    
    // Write headers
    file_state << "time,omega_x,omega_y,omega_z,omega_mag,"
               << "q_x,q_y,q_z,q_w,theta_error\n";
    file_control << "time,tau_x,tau_y,tau_z,phase\n";
    file_momentum << "time,h_x,h_y,h_z,h_total,saturated\n";
    
    // ============================================================================
    // SIMULATION LOOP
    // ============================================================================
    std::cout << "\nStarting simulation...\n";
    
    Method integration_method = Method::RK4;
    
    for (int step = 0; step < num_steps; step++) {
        double t = step * dt;
        
        // ====================================================================
        // CONTROL LOGIC (3 phases)
        // ====================================================================
        std::string phase_name;
        
        if (t < t_detumble_end) {
            // PHASE 1: Detumbling via B-dot (passive, no gyroscope needed)
            phase_name = "DETUMBLING";
            controller.enableReactionWheels(false);
            controller.enableMagnetorquer(true);
            controller.enableBdot(true);
            
        } else if (t < t_pointing_end) {
            // PHASE 2: Fine pointing via reaction wheels
            phase_name = "FINE_POINTING";
            controller.enableReactionWheels(true);
            controller.enableMagnetorquer(false);
            
            // Command small torques for pointing (e.g., to Sun-pointing)
            Vec3 tau_cmd(0.002, 0.003, 0.001);  // mN·m (small stabilizing torques)
            controller.setWheelTorque(tau_cmd);
            
        } else {
            // PHASE 3: Desaturation via magnetorquer while wheels de-spin
            phase_name = "DESATURATING";
            controller.enableReactionWheels(true);
            controller.enableMagnetorquer(true);
            controller.enableBdot(true);
            
            // Reduce wheel torque to decrease momentum
            Vec3 tau_cmd(0.0005, 0.0008, 0.0003);
            controller.setWheelTorque(tau_cmd);
        }
        
        // Update magnetic field (simplified: vary with orbital position)
        double field_variation = 1.0 + 0.1 * std::sin(2 * M_PI * t / 600.0);
        Vec3 B_current = B_orbit * field_variation;
        controller.setMagneticField(B_current);
        
        // ====================================================================
        // COMPUTE TORQUES
        // ====================================================================
        Vec3 tau = controller.computeTorque(state, t);
        
        // ====================================================================
        // PROPAGATE STATE
        // ====================================================================
        Derivative deriv = attitude::derivative(state, tau, satellite);
        state = attitude::step(state, deriv, dt, integration_method, satellite);
        
        // ====================================================================
        // LOGGING (every 10 Hz, write every 100 ms)
        // ====================================================================
        if (step % 10 == 0) {
            // State
            double omega_mag = state.w.norm() * RAD2DEG;
            double theta_error = 2.0 * std::acos(std::abs(state.q.s)) * RAD2DEG;
            
            file_state << std::fixed << std::setprecision(6) << t << ","
                       << state.w[0]*RAD2DEG << "," 
                       << state.w[1]*RAD2DEG << ","
                       << state.w[2]*RAD2DEG << ","
                       << omega_mag << ","
                       << state.q.x << "," << state.q.y << "," 
                       << state.q.z << "," << state.q.s << ","
                       << theta_error << "\n";
            
            // Control commands
            int phase_id = (t < t_detumble_end) ? 1 : (t < t_pointing_end) ? 2 : 3;
            file_control << std::fixed << std::setprecision(6) << t << ","
                         << tau[0] << "," << tau[1] << "," << tau[2] << ","
                         << phase_id << "\n";
            
            // Momentum
            Vec3 h = controller.getWheelMomentum();
            double h_mag = h.norm();
            int saturated = controller.isWheelSaturated() ? 1 : 0;
            file_momentum << std::fixed << std::setprecision(6) << t << ","
                          << h[0] << "," << h[1] << "," << h[2] << ","
                          << h_mag << "," << saturated << "\n";
            
            // Console output (every 10 seconds)
            if (step % 1000 == 0) {
                std::cout << "t=" << std::setw(6) << std::fixed << std::setprecision(1) << t 
                          << "s | Phase: " << phase_name
                          << " | ω = " << std::setw(6) << std::setprecision(2) << omega_mag 
                          << "°/s | τ = " << std::setw(7) << std::setprecision(2) 
                          << tau.norm()*1e6 << " µN·m"
                          << " | h = " << std::setw(7) << std::setprecision(3) 
                          << h_mag*1e3 << " mN·m·s\n";
            }
        }
    }
    
    file_state.close();
    file_control.close();
    file_momentum.close();
    
    // ============================================================================
    // RESULTS SUMMARY
    // ============================================================================
    std::cout << "\n=== Simulation Complete ===\n\n";
    std::cout << "Final state:\n";
    std::cout << "  Spin rate: [" 
              << state.w[0]*RAD2DEG << ", " 
              << state.w[1]*RAD2DEG << ", " 
              << state.w[2]*RAD2DEG << "] °/s\n";
    
    double omega_final = state.w.norm() * RAD2DEG;
    std::cout << "  |ω| = " << omega_final << " °/s ";
    if (omega_final < 1.0) {
        std::cout << "(DETUMBLED ✓)\n";
    } else {
        std::cout << "(Still tumbling)\n";
    }
    
    Vec3 h_final = controller.getWheelMomentum();
    std::cout << "  Stored momentum: [" 
              << h_final[0]*1e3 << ", " 
              << h_final[1]*1e3 << ", " 
              << h_final[2]*1e3 << "] mN·m·s\n";
    
    double h_mag_final = h_final.norm();
    std::cout << "  |h| = " << h_mag_final*1e3 << " mN·m·s\n";
    
    std::cout << "\nOutput files:\n";
    std::cout << "  - hybrid_control_state.csv (spin rate, attitude)\n";
    std::cout << "  - hybrid_control_commands.csv (torque commands, phase)\n";
    std::cout << "  - hybrid_control_momentum.csv (reaction wheel momentum)\n";
    
    std::cout << "\n";
    return 0;
}
