#ifndef MAGNETORQUER_HPP
#define MAGNETORQUER_HPP

#include "vec3.hpp"
#include <cmath>

namespace attitude {

/**
 * Magnetorquer Model - Magnetic torque rod with B-dot detumbling
 */
class Magnetorquer {
public:
    Magnetorquer(double m_max = 0.5, double k_bdot = 0.05, 
                 bool use_bdot = true, const Vec3& m_command = Vec3(0, 0, 0))
        : m_max_(m_max), k_bdot_(k_bdot), use_bdot_(use_bdot),
          m_command_(m_command), B_prev_(0, 0, 0), B_dot_est_(0, 0, 0) {}

    void setMagneticMoment(const Vec3& m) {
        double norm_m = norm(m);
        if (norm_m > m_max_) {
            m_command_ = m * (m_max_ / norm_m);
        } else {
            m_command_ = m;
        }
    }

    void updateBdot(const Vec3& B_field, double dt) {
        if (dt > 1e-6) {
            B_dot_est_ = (B_field - B_prev_) / dt;
        }
        B_prev_ = B_field;
    }

    Vec3 computeTorque(const Vec3& B_field) const {
        Vec3 m;
        if (use_bdot_) {
            m = B_dot_est_ * (-k_bdot_);
            double norm_m = norm(m);
            if (norm_m > m_max_) {
                m = m * (m_max_ / norm_m);
            }
        } else {
            m = m_command_;
        }
        return cross(m, B_field);
    }

    Vec3 getMagneticMoment(const Vec3& B_field) const {
        if (use_bdot_) {
            Vec3 m = B_dot_est_ * (-k_bdot_);
            double norm_m = norm(m);
            if (norm_m > m_max_) {
                m = m * (m_max_ / norm_m);
            }
            return m;
        } else {
            return m_command_;
        }
    }

    Vec3 getBdotEstimate() const { return B_dot_est_; }
    void enableBdot(bool enable) { use_bdot_ = enable; }
    bool isBdotEnabled() const { return use_bdot_; }
    double getMaxDipoleMoment() const { return m_max_; }
    double getBdotGain() const { return k_bdot_; }
    void setBdotGain(double k) { k_bdot_ = k; }

private:
    double m_max_;
    double k_bdot_;
    bool use_bdot_;
    Vec3 m_command_;
    Vec3 B_prev_;
    Vec3 B_dot_est_;
};

}  // namespace attitude

#endif // MAGNETORQUER_HPP
