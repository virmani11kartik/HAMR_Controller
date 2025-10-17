#ifndef EKF_LOCALIZATION_H
#define EKF_LOCALIZATION_H

#include <cmath>
#include <algorithm>
#include "odometry.h"

inline float wrapToPi(float a){
    while (a >  M_PI) a -= 2.0f * M_PI;
    while (a < -M_PI) a += 2.0f * M_PI;
    return a;
}

struct EkfFlowConfig {};

struct EkfYawConfig {
    float R_yaw_rad2 = (10.0f * M_PI/180.0f) * (10.0f * M_PI/180.0f);
    float gate_sigma = 3.0f;
    float alignment_timeout_ms = 5000.0f;
    float min_calibration_level = 2.0f;   // (unused here)
    bool  enable_periodic_realignment = true;
    float realignment_threshold = 30.0f * M_PI/180.0f;
    int   realignment_count_threshold = 10;
};

static bool           g_aligned                = false;
static float          g_yaw_offset             = 0.0f;
static unsigned long  g_last_realignment_ms    = 0;
static int            g_large_innovation_count = 0;
static float          theta_prev_wrapped       = 0.0f;
static float          theta_unwrapped          = 0.0f;

inline void ekfYawResetAlignment() {
    g_aligned = false;
    g_yaw_offset = 0.0f;
    g_last_realignment_ms = 0;
    g_large_innovation_count = 0;
    Serial.println("EKF: Yaw alignment reset");
}

inline bool shouldRealign(float innovation, const EkfYawConfig& cfg) {
    if (!cfg.enable_periodic_realignment) return false;
    const unsigned long now = millis();

    if (std::fabs(innovation) > cfg.realignment_threshold) {
        ++g_large_innovation_count;
        if (g_large_innovation_count >= cfg.realignment_count_threshold &&
            (now - g_last_realignment_ms) > cfg.alignment_timeout_ms) {
            return true;
        }
    } else {
        g_large_innovation_count = std::max(0, g_large_innovation_count - 1);
    }
    return false;
}

inline bool ekfYawUpdate(float imu_yaw_rad, const EkfYawConfig& cfg = EkfYawConfig()){
    if (!std::isfinite(imu_yaw_rad)) {
        Serial.println("EKF: Invalid IMU yaw measurement");
        return false;
    }

    // Initial alignment ONLY (no shouldRealign peeking here)
    if (!g_aligned) {
        const unsigned long prev_last = g_last_realignment_ms; // 0 if never aligned
        g_yaw_offset = wrapToPi(robot_theta - imu_yaw_rad);
        g_aligned = true;
        g_last_realignment_ms = millis();
        g_large_innovation_count = 0;
        Serial.printf("EKF: Yaw %saligned, offset=%.2f deg\n",
                      prev_last == 0 ? "" : "re-",
                      g_yaw_offset * 180.0f / M_PI);
    }

    // Predicted measurement after applying current offset
    float z = wrapToPi(imu_yaw_rad + g_yaw_offset);

    // Pull current covariance
    float P[9];
    covarianceToMatrix(covariance, P);

    // Innovation (measured - predicted heading)
    float y = wrapToPi(z - robot_theta);

    // Re-align only based on real innovation y
    if (shouldRealign(y, cfg)) {
        g_yaw_offset = wrapToPi(robot_theta - imu_yaw_rad);
        g_last_realignment_ms = millis();
        g_large_innovation_count = 0;
        z = wrapToPi(imu_yaw_rad + g_yaw_offset);
        y = wrapToPi(z - robot_theta);
    }

    // S = P(θ,θ) + R
    float S = P[8] + cfg.R_yaw_rad2;
    if (S <= 1e-12f) return false;

    // Gate by Mahalanobis distance
    if (cfg.gate_sigma > 0.0f) {
        const float maha2 = (y * y) / S;
        if (maha2 > cfg.gate_sigma * cfg.gate_sigma) return false;
    }

    // K for H = [0 0 1]
    const float Kx = P[2] / S;
    const float Ky = P[5] / S;
    const float Kt = P[8] / S;

    // State update
    robot_x     += Kx * y;
    robot_y     += Ky * y;
    robot_theta  = wrapToPi(robot_theta + Kt * y);

    // Covariance update
    const float HP[3] = { P[6], P[7], P[8] };
    P[0] -= Kx * HP[0];  P[1] -= Kx * HP[1];  P[2] -= Kx * HP[2];
    P[3] -= Ky * HP[0];  P[4] -= Ky * HP[1];  P[5] -= Ky * HP[2];
    P[6] -= Kt * HP[0];  P[7] -= Kt * HP[1];  P[8] -= Kt * HP[2];

    const float R = cfg.R_yaw_rad2; // Joseph stabilization
    P[0] += Kx * Kx * R;  P[1] += Kx * Ky * R;  P[2] += Kx * Kt * R;
    P[3] += Ky * Kx * R;  P[4] += Ky * Ky * R;  P[5] += Ky * Kt * R;
    P[6] += Kt * Kx * R;  P[7] += Kt * Ky * R;  P[8] += Kt * Kt * R;

    // Enforce symmetry
    P[1] = P[3]; P[2] = P[6]; P[5] = P[7];

    matrixToCovariance(P, covariance);
    covariance[0] = std::fmax(covariance[0], 1e-9f);
    covariance[3] = std::fmax(covariance[3], 1e-9f);
    covariance[5] = std::fmax(covariance[5], 1e-9f);

    return true;
}

inline void getEkfAlignmentInfo(bool& is_aligned, float& offset_deg, unsigned long& last_alignment_ms) {
    is_aligned        = g_aligned;
    offset_deg        = g_yaw_offset * 180.0f / M_PI;
    last_alignment_ms = g_last_realignment_ms;
}

inline void updateThetaUnwrapped(float theta_wrapped_now){
    const float d = wrapToPi(theta_wrapped_now - theta_prev_wrapped);
    theta_unwrapped += d;
    theta_prev_wrapped = theta_wrapped_now;
}

inline float getRobotThetaUnwrapped(){ return theta_unwrapped; }

#endif // EKF_LOCALIZATION_H
