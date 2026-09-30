#ifndef REACTION_WHEEL_HPP
#define REACTION_WHEEL_HPP

#include "vec3.hpp"
#include <algorithm>
#include <cmath>
#include <vector>

namespace attitude {

/**
 * Reaction Wheel Model - Single axis momentum storage
 */
class ReactionWheel {
public:
    ReactionWheel(int axis = 2, double I_wheel = 0.1, 
                  double tau_max = 0.01, double h_max = 0.2)
        : axis_(axis), I_wheel_(I_wheel), tau_max_(tau_max), h_max_(h_max),
          h_current_(0), tau_command_(0), saturated_(false) {
        if (axis < 0 || axis > 2) {
            throw std::runtime_error("ReactionWheel: invalid axis");
        }
    }

    void setTorqueCommand(double tau) {
        tau_command_ = std::clamp(tau, -tau_max_, tau_max_);
    }

    Vec3 getTorque(double dt) {
        double h_next = h_current_ + tau_command_ * dt;
        
        saturated_ = false;
        if (std::abs(h_next) > h_max_) {
            saturated_ = true;
            if ((h_current_ > 0 && tau_command_ > 0) ||
                (h_current_ < 0 && tau_command_ < 0)) {
                tau_command_ = 0;
            }
        }
        
        h_current_ = std::clamp(h_next, -h_max_, h_max_);
        
        Vec3 tau_out(0, 0, 0);
        if (axis_ == 0) tau_out.x = tau_command_;
        else if (axis_ == 1) tau_out.y = tau_command_;
        else if (axis_ == 2) tau_out.z = tau_command_;
        
        return tau_out;
    }

    double getMomentum() const { return h_current_; }
    double getSpinRate() const { return h_current_ / I_wheel_; }
    double getSpinRateRPM() const { return getSpinRate() * 60.0 / (2.0 * kPi); }
    bool isSaturated() const { return saturated_; }
    double getAvailableCapacity() const {
        return (h_current_ >= 0) ? (h_max_ - h_current_) : (h_max_ + h_current_);
    }

    void applyExternalTorque(double external_torque, double dt) {
        double dh = external_torque * dt;
        h_current_ = std::clamp(h_current_ + dh, -h_max_, h_max_);
    }

    void reset() {
        h_current_ = 0;
        tau_command_ = 0;
        saturated_ = false;
    }

    int getAxis() const { return axis_; }
    double getMomentOfInertia() const { return I_wheel_; }
    double getMaxTorque() const { return tau_max_; }
    double getMaxMomentum() const { return h_max_; }
    double getCurrentTorqueCommand() const { return tau_command_; }

private:
    int axis_;
    double I_wheel_;
    double tau_max_;
    double h_max_;
    double h_current_;
    double tau_command_;
    bool saturated_;
};

/**
 * Reaction Wheel Cluster - 3-axis control
 */
class ReactionWheelCluster {
public:
    ReactionWheelCluster(int num_wheels = 3) {
        if (num_wheels < 1 || num_wheels > 4) {
            throw std::runtime_error("ReactionWheelCluster: 1-4 wheels supported");
        }
        for (int i = 0; i < num_wheels; i++) {
            wheels_.emplace_back((2 - i) % 3);
        }
    }

    ReactionWheel& getWheel(int i) { return wheels_[i]; }
    const ReactionWheel& getWheel(int i) const { return wheels_[i]; }

    void setTorqueCommand(double tau_x, double tau_y, double tau_z) {
        for (auto& wheel : wheels_) {
            if (wheel.getAxis() == 0) wheel.setTorqueCommand(tau_x);
            else if (wheel.getAxis() == 1) wheel.setTorqueCommand(tau_y);
            else if (wheel.getAxis() == 2) wheel.setTorqueCommand(tau_z);
        }
    }

    void setTorqueCommand(const Vec3& tau) {
        setTorqueCommand(tau.x, tau.y, tau.z);
    }

    Vec3 getTorque(double dt) {
        Vec3 tau_total(0, 0, 0);
        for (auto& wheel : wheels_) {
            tau_total = tau_total + wheel.getTorque(dt);
        }
        return tau_total;
    }

    Vec3 getTotalMomentum() const {
        Vec3 h_total(0, 0, 0);
        for (const auto& wheel : wheels_) {
            if (wheel.getAxis() == 0) h_total.x = wheel.getMomentum();
            else if (wheel.getAxis() == 1) h_total.y = wheel.getMomentum();
            else if (wheel.getAxis() == 2) h_total.z = wheel.getMomentum();
        }
        return h_total;
    }

    bool isAnySaturated() const {
        for (const auto& wheel : wheels_) {
            if (wheel.isSaturated()) return true;
        }
        return false;
    }

    void performDesaturation(const Vec3& external_torques, double dt) {
        for (auto& wheel : wheels_) {
            if (wheel.getAxis() == 0) wheel.applyExternalTorque(external_torques.x, dt);
            else if (wheel.getAxis() == 1) wheel.applyExternalTorque(external_torques.y, dt);
            else if (wheel.getAxis() == 2) wheel.applyExternalTorque(external_torques.z, dt);
        }
    }

    void reset() {
        for (auto& wheel : wheels_) {
            wheel.reset();
        }
    }

    int getNumWheels() const { return wheels_.size(); }

private:
    std::vector<ReactionWheel> wheels_;
};

}  // namespace attitude

#endif // REACTION_WHEEL_HPP
