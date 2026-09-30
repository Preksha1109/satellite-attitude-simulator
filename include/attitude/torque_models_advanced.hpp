#ifndef TORQUE_MODELS_ADVANCED_HPP
#define TORQUE_MODELS_ADVANCED_HPP

#include "torque_model.hpp"
#include "magnetorquer.hpp"
#include "reaction_wheel.hpp"
#include "state.hpp"

namespace attitude {

/**
 * B-dot Detumbling Torque Model
 */
class BdotDetumbleTorque : public TorqueModel {
public:
    BdotDetumbleTorque(double m_max = 0.5, double k_bdot = 0.05)
        : magnetorquer_(m_max, k_bdot, true), 
          B_field_(0, 0, 5e-5) {}

    Vec3 computeTorque(const State& state, double t) const override {
        magnetorquer_.updateBdot(B_field_, 0.01);
        return magnetorquer_.computeTorque(B_field_);
    }

    void setMagneticField(const Vec3& B) {
        B_field_ = B;
    }

    Magnetorquer& getMagnetorquer() {
        return magnetorquer_;
    }

private:
    mutable Magnetorquer magnetorquer_;
    mutable Vec3 B_field_;
};

/**
 * Reaction Wheel Control Torque Model
 */
class ReactionWheelTorque : public TorqueModel {
public:
    ReactionWheelTorque(double tau_max = 0.01, double h_max = 0.2)
        : wheels_(3) {
        for (int i = 0; i < 3; i++) {
            wheels_.getWheel(i).setTorqueCommand(0);
        }
    }

    Vec3 computeTorque(const State& state, double t) const override {
        return wheels_.getTorque(0.01);
    }

    void setTorqueCommand(const Vec3& tau) {
        wheels_.setTorqueCommand(tau);
    }

    void setTorqueCommand(double tau_x, double tau_y, double tau_z) {
        wheels_.setTorqueCommand(tau_x, tau_y, tau_z);
    }

    Vec3 getStoredMomentum() const {
        return wheels_.getTotalMomentum();
    }

    bool isAnySaturated() const {
        return wheels_.isAnySaturated();
    }

    void applyDesaturationTorque(const Vec3& tau_desaturate, double dt) {
        wheels_.performDesaturation(tau_desaturate, dt);
    }

    ReactionWheelCluster& getWheelCluster() {
        return wheels_;
    }

private:
    mutable ReactionWheelCluster wheels_;
};

/**
 * Hybrid Attitude Control - Wheels + Magnetorquer
 */
class HybridAttitudeControl : public TorqueModel {
public:
    HybridAttitudeControl(double tau_rw = 0.01, double h_rw = 0.2,
                         double m_mag = 0.5, double k_bdot = 0.05)
        : rw_model_(tau_rw, h_rw), 
          mag_model_(m_mag, k_bdot),
          use_rw_(true), use_mag_(true),
          desaturation_active_(false) {}

    Vec3 computeTorque(const State& state, double t) const override {
        Vec3 tau_total(0, 0, 0);

        if (use_rw_) {
            tau_total = tau_total + rw_model_.computeTorque(state, t);
        }

        if (use_mag_ && rw_model_.isAnySaturated()) {
            desaturation_active_ = true;
            Vec3 mag_torque = mag_model_.computeTorque(state, t);
            tau_total = tau_total + mag_torque;
            rw_model_.applyDesaturationTorque(mag_torque, 0.01);
        } else {
            desaturation_active_ = false;
        }

        return tau_total;
    }

    void setWheelTorque(const Vec3& tau) {
        rw_model_.setTorqueCommand(tau);
    }

    void setMagneticMoment(const Vec3& m) {
        mag_model_.getMagnetorquer().setMagneticMoment(m);
        mag_model_.getMagnetorquer().enableBdot(false);
    }

    void enableBdot(bool enable) {
        mag_model_.getMagnetorquer().enableBdot(enable);
    }

    void setMagneticField(const Vec3& B) {
        mag_model_.setMagneticField(B);
    }

    Vec3 getWheelMomentum() const { 
        return rw_model_.getStoredMomentum(); 
    }
    
    bool isWheelSaturated() const { 
        return rw_model_.isAnySaturated(); 
    }
    
    bool isDesaturating() const { 
        return desaturation_active_; 
    }

    void enableReactionWheels(bool enable) { 
        use_rw_ = enable; 
    }
    
    void enableMagnetorquer(bool enable) { 
        use_mag_ = enable; 
    }

private:
    mutable ReactionWheelTorque rw_model_;
    mutable BdotDetumbleTorque mag_model_;
    bool use_rw_;
    bool use_mag_;
    mutable bool desaturation_active_;
};

}  // namespace attitude

#endif // TORQUE_MODELS_ADVANCED_HPP
