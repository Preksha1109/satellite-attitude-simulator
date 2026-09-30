#ifndef MAGNETORQUER_HPP
#define MAGNETORQUER_HPP

#include "vec3.hpp"
#include <cmath>

/**
 * Magnetorquer Model
 * 
 * A magnetic torque rod that creates torque by interacting with Earth's magnetic field.
 * 
 * Physics:
 *   τ = m × B
 *   where m is magnetic moment (A·m²), B is magnetic field (Tesla)
 *   
 * B-dot Detumbling Controller:
 *   Creates magnetic moment proportional to rate of change of B field
 *   m = -k_bdot * dB/dt
 *   This naturally damps angular velocity without needing gyro measurements
 *   
 * Typical parameters:
 *   - Maximum dipole moment: 0.1 to 1.0 A·m²
 *   - Detumbling gain: 0.01 to 0.1 (A·m²·s/T)
 *   - Settling time: 500-2000 seconds to low spin rates
 */

class Magnetorquer {
public:
    /**
     * Create a magnetorquer with specified parameters
     * 
     * @param m_max       Maximum magnetic dipole moment (A·m²)
     * @param k_bdot      B-dot detumbling controller gain (A·m²·s/T)
     * @param use_bdot    Enable B-dot controller (passive detumbling)
     * @param m_command   Initial magnetic moment command (if not using B-dot)
     */
    Magnetorquer(double m_max = 0.5, double k_bdot = 0.05, 
                 bool use_bdot = true, const Vec3& m_command = Vec3(0, 0, 0))
        : m_max_(m_max), k_bdot_(k_bdot), use_bdot_(use_bdot),
          m_command_(m_command), B_prev_(0, 0, 0), B_dot_est_(0, 0, 0) {}

    /**
     * Set magnetic moment command (direct control, no B-dot)
     * Automatically saturates to m_max
     */
    void setMagneticMoment(const Vec3& m) {
        double norm_m = m.norm();
        if (norm_m > m_max_) {
            m_command_ = m * (m_max_ / norm_m);  // saturate
        } else {
            m_command_ = m;
        }
    }

    /**
     * Update B-dot estimate (call once per time step with current B field)
     * This estimates dB/dt numerically
     * 
     * @param B_field     Current magnetic field (Tesla), body frame
     * @param dt          Time step (seconds)
     */
    void updateBdot(const Vec3& B_field, double dt) {
        if (dt > 1e-6) {
            B_dot_est_ = (B_field - B_prev_) / dt;
        }
        B_prev_ = B_field;
    }

    /**
     * Compute magnetic torque
     * 
     * @param B_field     Current magnetic field (Tesla), body frame
     * @return            Torque (N·m) in body frame
     */
    Vec3 computeTorque(const Vec3& B_field) const {
        // Get current magnetic moment
        Vec3 m;
        if (use_bdot_) {
            // B-dot law: m = -k_bdot * dB/dt
            m = B_dot_est_ * (-k_bdot_);
            
            // Saturate to maximum dipole moment
            double norm_m = m.norm();
            if (norm_m > m_max_) {
                m = m * (m_max_ / norm_m);
            }
        } else {
            m = m_command_;
        }

        // Torque from cross product: τ = m × B
        return m.cross(B_field);
    }

    /**
     * Get current magnetic moment (for diagnostics/telemetry)
     * This is what's actually being commanded after saturation
     */
    Vec3 getMagneticMoment(const Vec3& B_field) const {
        if (use_bdot_) {
            Vec3 m = B_dot_est_ * (-k_bdot_);
            double norm_m = m.norm();
            if (norm_m > m_max_) {
                m = m * (m_max_ / norm_m);
            }
            return m;
        } else {
            return m_command_;
        }
    }

    /**
     * Get B-dot estimate (rate of change of magnetic field)
     */
    Vec3 getBdotEstimate() const {
        return B_dot_est_;
    }

    /**
     * Enable/disable B-dot controller
     */
    void enableBdot(bool enable) {
        use_bdot_ = enable;
    }

    bool isBdotEnabled() const {
        return use_bdot_;
    }

    /**
     * Get parameters for logging/analysis
     */
    double getMaxDipoleMoment() const { return m_max_; }
    double getBdotGain() const { return k_bdot_; }
    void setBdotGain(double k) { k_bdot_ = k; }

private:
    double m_max_;              // Maximum magnetic dipole moment (A·m²)
    double k_bdot_;             // B-dot controller gain
    bool use_bdot_;             // Whether to use B-dot controller
    Vec3 m_command_;            // Commanded magnetic moment (direct control)
    Vec3 B_prev_;               // Previous magnetic field (for dB/dt estimation)
    Vec3 B_dot_est_;            // Estimated dB/dt
};

#endif // MAGNETORQUER_HPP
