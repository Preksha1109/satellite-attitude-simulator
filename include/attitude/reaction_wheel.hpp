#ifndef REACTION_WHEEL_HPP
#define REACTION_WHEEL_HPP

#include "vec3.hpp"
#include <algorithm>
#include <cmath>

/**
 * Reaction Wheel Model
 * 
 * A momentum storage device that produces torque by changing its angular momentum.
 * 
 * Physics:
 *   τ = dh/dt
 *   where h is angular momentum (kg·m²/s)
 *   h = I_wheel * ω_wheel
 *   
 * Constraints:
 *   - Maximum torque output: τ_max
 *   - Maximum angular momentum: h_max (momentum saturation)
 *   - Maximum spin rate: ω_max
 *   
 * Typical parameters (0.1 kg·m² reaction wheel):
 *   - Moment of inertia: 0.1 kg·m²
 *   - Max torque: 0.01 N·m
 *   - Max momentum: 0.2 N·m·s
 *   - Max speed: 2000 RPM
 */

class ReactionWheel {
public:
    /**
     * Create a reaction wheel aligned along a principal axis
     * 
     * @param axis         Alignment axis (0=x, 1=y, 2=z)
     * @param I_wheel      Moment of inertia of wheel (kg·m²)
     * @param tau_max      Maximum torque output (N·m)
     * @param h_max        Maximum angular momentum (N·m·s)
     */
    ReactionWheel(int axis = 2, double I_wheel = 0.1, 
                  double tau_max = 0.01, double h_max = 0.2)
        : axis_(axis), I_wheel_(I_wheel), tau_max_(tau_max), h_max_(h_max),
          h_current_(0), tau_command_(0), saturated_(false) {
        if (axis < 0 || axis > 2) {
            throw std::runtime_error("ReactionWheel: invalid axis (must be 0, 1, or 2)");
        }
        if (I_wheel <= 0 || tau_max <= 0 || h_max <= 0) {
            throw std::runtime_error("ReactionWheel: parameters must be positive");
        }
    }

    /**
     * Set desired torque command
     * Torque is along the wheel's principal axis
     * Automatically saturated to tau_max
     * 
     * @param tau          Desired torque magnitude (N·m)
     */
    void setTorqueCommand(double tau) {
        tau_command_ = std::clamp(tau, -tau_max_, tau_max_);
    }

    /**
     * Get the torque output vector in body frame
     * 
     * @param dt           Time step for integration (seconds)
     * @return             Torque vector (N·m) in body frame
     */
    Vec3 getTorque(double dt) {
        // Integrate momentum: h(t+dt) = h(t) + τ_cmd * dt
        // But check if we would exceed momentum limit
        double h_next = h_current_ + tau_command_ * dt;
        
        saturated_ = false;
        if (std::abs(h_next) > h_max_) {
            // Saturated: momentum can't increase further
            saturated_ = true;
            
            // Can only command torque toward zero (desaturation)
            if ((h_current_ > 0 && tau_command_ > 0) ||
                (h_current_ < 0 && tau_command_ < 0)) {
                // Trying to increase momentum beyond limit
                tau_command_ = 0;
            }
            // If h_current_ > h_max, can still command negative torque to reduce
            // If h_current_ < -h_max, can still command positive torque to reduce
        }
        
        // Update actual momentum
        h_current_ = std::clamp(h_next, -h_max_, h_max_);
        
        // Convert torque to body frame vector (along principal axis)
        Vec3 tau_out(0, 0, 0);
        if (axis_ == 0) tau_out[0] = tau_command_;
        else if (axis_ == 1) tau_out[1] = tau_command_;
        else if (axis_ == 2) tau_out[2] = tau_command_;
        
        return tau_out;
    }

    /**
     * Get current angular momentum stored in wheel
     */
    double getMomentum() const {
        return h_current_;
    }

    /**
     * Get current spin rate of wheel
     */
    double getSpinRate() const {
        return h_current_ / I_wheel_;  // ω = h / I
    }

    /**
     * Get spin rate in RPM (for diagnostics)
     */
    double getSpinRateRPM() const {
        return getSpinRate() * 60.0 / (2.0 * M_PI);
    }

    /**
     * Check if wheel is momentum saturated
     */
    bool isSaturated() const {
        return saturated_;
    }

    /**
     * Get available momentum capacity (how much more can be stored)
     */
    double getAvailableCapacity() const {
        if (h_current_ >= 0) {
            return h_max_ - h_current_;
        } else {
            return h_max_ + h_current_;  // h_current_ is negative
        }
    }

    /**
     * Perform momentum desaturation via magnetorquer interaction
     * (external torque that reduces momentum without ground contact)
     * 
     * @param external_torque  Torque from magnetorquer or other source (N·m)
     * @param dt               Time step (seconds)
     */
    void applyExternalTorque(double external_torque, double dt) {
        double dh = external_torque * dt;
        h_current_ = std::clamp(h_current_ + dh, -h_max_, h_max_);
    }

    /**
     * Reset wheel (emergency stop)
     */
    void reset() {
        h_current_ = 0;
        tau_command_ = 0;
        saturated_ = false;
    }

    /**
     * Get parameters for diagnostics
     */
    int getAxis() const { return axis_; }
    double getMomentOfInertia() const { return I_wheel_; }
    double getMaxTorque() const { return tau_max_; }
    double getMaxMomentum() const { return h_max_; }
    double getCurrentTorqueCommand() const { return tau_command_; }

private:
    int axis_;                  // Wheel alignment axis (0=x, 1=y, 2=z)
    double I_wheel_;            // Moment of inertia (kg·m²)
    double tau_max_;            // Maximum torque (N·m)
    double h_max_;              // Maximum momentum (N·m·s)
    double h_current_;          // Current angular momentum (N·m·s)
    double tau_command_;        // Current torque command (N·m)
    bool saturated_;            // Whether momentum is saturated
};

/**
 * Reaction Wheel Cluster
 * 
 * Multiple reaction wheels (typically 3-4) for 3-axis attitude control
 * with momentum management
 */
class ReactionWheelCluster {
public:
    /**
     * Create a 3-axis reaction wheel cluster
     * 
     * @param wheel_params  [I, tau_max, h_max] for each wheel
     * @param num_wheels    Number of wheels (1-4 typical)
     */
    ReactionWheelCluster(int num_wheels = 3) : wheels_(num_wheels) {
        if (num_wheels < 1 || num_wheels > 4) {
            throw std::runtime_error("ReactionWheelCluster: 1-4 wheels supported");
        }
        
        // Create wheels aligned along principal axes (Z-then-Y-then-X, typical)
        for (int i = 0; i < num_wheels; i++) {
            wheels_[i] = ReactionWheel((2 - i) % 3);  // z, y, x, repeat
        }
    }

    /**
     * Get individual wheel
     */
    ReactionWheel& getWheel(int i) {
        return wheels_[i];
    }

    const ReactionWheel& getWheel(int i) const {
        return wheels_[i];
    }

    /**
     * Set torque commands for all wheels
     * 
     * @param tau_x, tau_y, tau_z   Desired torques along each axis
     */
    void setTorqueCommand(double tau_x, double tau_y, double tau_z) {
        for (auto& wheel : wheels_) {
            if (wheel.getAxis() == 0) wheel.setTorqueCommand(tau_x);
            else if (wheel.getAxis() == 1) wheel.setTorqueCommand(tau_y);
            else if (wheel.getAxis() == 2) wheel.setTorqueCommand(tau_z);
        }
    }

    /**
     * Set torque commands from vector
     */
    void setTorqueCommand(const Vec3& tau) {
        setTorqueCommand(tau[0], tau[1], tau[2]);
    }

    /**
     * Get total torque output from all wheels
     */
    Vec3 getTorque(double dt) {
        Vec3 tau_total(0, 0, 0);
        for (auto& wheel : wheels_) {
            tau_total = tau_total + wheel.getTorque(dt);
        }
        return tau_total;
    }

    /**
     * Get total stored momentum
     */
    Vec3 getTotalMomentum() const {
        Vec3 h_total(0, 0, 0);
        for (const auto& wheel : wheels_) {
            if (wheel.getAxis() == 0) h_total[0] = wheel.getMomentum();
            else if (wheel.getAxis() == 1) h_total[1] = wheel.getMomentum();
            else if (wheel.getAxis() == 2) h_total[2] = wheel.getMomentum();
        }
        return h_total;
    }

    /**
     * Check if any wheel is saturated
     */
    bool isAnySaturated() const {
        for (const auto& wheel : wheels_) {
            if (wheel.isSaturated()) return true;
        }
        return false;
    }

    /**
     * Momentum desaturation via external torques (e.g., magnetorquer)
     * 
     * @param external_torques   External torque vector (from magnetorquer, etc.)
     * @param dt                 Time step
     */
    void performDesaturation(const Vec3& external_torques, double dt) {
        for (auto& wheel : wheels_) {
            if (wheel.getAxis() == 0) wheel.applyExternalTorque(external_torques[0], dt);
            else if (wheel.getAxis() == 1) wheel.applyExternalTorque(external_torques[1], dt);
            else if (wheel.getAxis() == 2) wheel.applyExternalTorque(external_torques[2], dt);
        }
    }

    /**
     * Reset all wheels
     */
    void reset() {
        for (auto& wheel : wheels_) {
            wheel.reset();
        }
    }

    int getNumWheels() const {
        return wheels_.size();
    }

private:
    std::vector<ReactionWheel> wheels_;
};

#endif // REACTION_WHEEL_HPP
