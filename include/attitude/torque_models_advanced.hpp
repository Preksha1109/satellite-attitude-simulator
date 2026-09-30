#ifndef TORQUE_MODELS_ADVANCED_HPP
#define TORQUE_MODELS_ADVANCED_HPP

#include "torque_model.hpp"
#include "magnetorquer.hpp"
#include "reaction_wheel.hpp"
#include "state.hpp"

/**
 * B-dot Detumbling Torque Model
 * 
 * Passive magnetic detumbling using B-dot controller
 * Requires orbital magnetic field model (e.g., dipole field)
 * Does NOT require gyroscope measurements
 * 
 * Typical use:
 *   - Post-deployment detumbling
 *   - Passive initial acquisition
 *   - Momentum desaturation
 */
class BdotDetumbleTorque : public TorqueModel {
public:
    /**
     * Create B-dot detumbling model
     * 
     * @param m_max       Maximum dipole moment (A·m²)
     * @param k_bdot      Controller gain (A·m²·s/T)
     * @param B_field_fn  Function to compute magnetic field B(t, r) in body frame
     */
    BdotDetumbleTorque(double m_max = 0.5, double k_bdot = 0.05)
        : magnetorquer_(m_max, k_bdot, true), 
          B_field_(0, 0, 5e-5)  // Earth's field ~50 µT, pointing down */
    {}

    Vec3 computeTorque(const State& state, double t) override {
        // In a real scenario, B_field would vary with orbital position
        // For now, use constant dipole field approximation
        // In orbit at 500 km: B ≈ 24-30 µT
        
        // Update B-dot estimate (needs current magnetic field)
        // In a full simulation, this would come from magnetometer
        magnetorquer_.updateBdot(B_field_, 0.01);  // assume 100 Hz
        
        // Compute torque from B-dot law
        return magnetorquer_.computeTorque(B_field_);
    }

    /**
     * Set magnetic field (e.g., from orbit propagation or measurement)
     */
    void setMagneticField(const Vec3& B) {
        B_field_ = B;
    }

    Magnetorquer& getMagnetorquer() {
        return magnetorquer_;
    }

private:
    Magnetorquer magnetorquer_;
    Vec3 B_field_;  // Earth's magnetic field in body frame
};

/**
 * Reaction Wheel Control Torque Model
 * 
 * Produces torque using 3-axis reaction wheel cluster
 * Enables 3-axis attitude control
 * 
 * Typical use:
 *   - Fine pointing control
 *   - Attitude stabilization
 *   - 3-axis control after detumbling
 */
class ReactionWheelTorque : public TorqueModel {
public:
    /**
     * Create reaction wheel torque model with 3-axis cluster
     * 
     * @param tau_x, tau_y, tau_z   Maximum torques (N·m)
     * @param h_max_x, h_max_y, h_max_z   Maximum moments (N·m·s)
     */
    ReactionWheelTorque(double tau_max = 0.01, double h_max = 0.2)
        : wheels_(3) {
        // Configure three reaction wheels (z, y, x axes)
        for (int i = 0; i < 3; i++) {
            wheels_.getWheel(i) = ReactionWheel(i, 0.1, tau_max, h_max);
        }
    }

    Vec3 computeTorque(const State& state, double t) override {
        return wheels_.getTorque(0.01);  // assume 100 Hz
    }

    /**
     * Set desired torque command
     */
    void setTorqueCommand(const Vec3& tau) {
        wheels_.setTorqueCommand(tau);
    }

    /**
     * Set individual axis torque
     */
    void setTorqueCommand(double tau_x, double tau_y, double tau_z) {
        wheels_.setTorqueCommand(tau_x, tau_y, tau_z);
    }

    /**
     * Get stored momentum
     */
    Vec3 getStoredMomentum() const {
        return wheels_.getTotalMomentum();
    }

    /**
     * Check if any wheel is saturated
     */
    bool isAnySaturated() const {
        return wheels_.isAnySaturated();
    }

    /**
     * Apply external desaturation torques (from magnetorquer)
     */
    void applyDesaturationTorque(const Vec3& tau_desaturate, double dt) {
        wheels_.performDesaturation(tau_desaturate, dt);
    }

    ReactionWheelCluster& getWheelCluster() {
        return wheels_;
    }

private:
    ReactionWheelCluster wheels_;
};

/**
 * Hybrid Attitude Control
 * 
 * Combines reaction wheels for fine control with magnetorquer for:
 *   - Momentum desaturation
 *   - Unloading reaction wheel momentum to Earth's field
 *   - Initial detumbling before RW activation
 */
class HybridAttitudeControl : public TorqueModel {
public:
    HybridAttitudeControl(double tau_rw = 0.01, double h_rw = 0.2,
                         double m_mag = 0.5, double k_bdot = 0.05)
        : rw_model_(tau_rw, h_rw), 
          mag_model_(m_mag, k_bdot),
          use_rw_(true), use_mag_(true),
          desaturation_active_(false) {}

    Vec3 computeTorque(const State& state, double t) override {
        Vec3 tau_total(0, 0, 0);

        // Reaction wheel torque (fine control)
        if (use_rw_) {
            tau_total = tau_total + rw_model_.computeTorque(state, t);
        }

        // Check for momentum saturation and activate desaturation
        if (use_mag_ && rw_model_.isAnySaturated()) {
            desaturation_active_ = true;
            
            // Use magnetorquer to create desaturation torque
            // This should be designed to create opposite torque to RW momentum
            Vec3 mag_torque = mag_model_.computeTorque(state, t);
            tau_total = tau_total + mag_torque;
            
            // Apply magnetic torque to desaturate wheels
            rw_model_.applyDesaturationTorque(mag_torque, 0.01);
        } else {
            desaturation_active_ = false;
        }

        return tau_total;
    }

    /**
     * Set reaction wheel torque command
     */
    void setWheelTorque(const Vec3& tau) {
        rw_model_.setTorqueCommand(tau);
    }

    /**
     * Set magnetic torque command (for direct control, not B-dot)
     */
    void setMagneticMoment(const Vec3& m) {
        mag_model_.getMagnetorquer().setMagneticMoment(m);
        mag_model_.getMagnetorquer().enableBdot(false);  // disable auto B-dot
    }

    /**
     * Enable/disable B-dot desaturation controller
     */
    void enableBdot(bool enable) {
        mag_model_.getMagnetorquer().enableBdot(enable);
    }

    /**
     * Set magnetic field for B-dot controller
     */
    void setMagneticField(const Vec3& B) {
        mag_model_.setMagneticField(B);
    }

    /**
     * Get current state
     */
    Vec3 getWheelMomentum() const { return rw_model_.getStoredMomentum(); }
    bool isWheelSaturated() const { return rw_model_.isAnySaturated(); }
    bool isDesaturating() const { return desaturation_active_; }

    void enableReactionWheels(bool enable) { use_rw_ = enable; }
    void enableMagnetorquer(bool enable) { use_mag_ = enable; }

private:
    ReactionWheelTorque rw_model_;
    BdotDetumbleTorque mag_model_;
    bool use_rw_;
    bool use_mag_;
    bool desaturation_active_;
};

#endif // TORQUE_MODELS_ADVANCED_HPP
