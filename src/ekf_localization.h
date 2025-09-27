#ifndef EKF_LOCALIZATION_H
#define EKF_LOCALIZATION_H

#include <math.h>
#include "odometry.h"

inline float wrapToPi(float a){
    while(a>M_PI) a -=2.0*M_PI;
    while(a<-M_PI) a +=2.0*M_PI;
    return a;
}

struct EkfYawConfig {
    float R_yaw_rad2 = (10.0f * M_PI/180.0f) * (10.0f * M_PI/180.0f); // IMU yaw measurement noise (rad^2)
    float gate_sigma = 3.0f;
    float alignment_timeout_ms = 5000.0f;
    float min_calibration_level = 2;
    bool enable_periodic_realignment = true;
    float realignment_threshold = 30.0f * M_PI/180.0f; 
    int realignment_count_threshold = 10;
};

static bool g_aligned = false;
static float g_yaw_offset = 0.0f;
static unsigned long g_last_realignment_ms = 0;
static int g_large_innovation_count = 0;

inline void ekfYawResetAlignment() {
    g_aligned = false;
    g_yaw_offset = 0.0f;
    g_last_realignment_ms = 0;
    g_large_innovation_count = 0;
    Serial.println("EKF: Yaw alignment reset");
}

// check if aligned, if not align once
inline bool shouldRealign(float innovation, const EkfYawConfig& cfg) {
    if (!cfg.enable_periodic_realignment) return false;
    
    unsigned long now = millis();
    if (fabs(innovation) > cfg.realignment_threshold) {
        g_large_innovation_count++;
        if (g_large_innovation_count >= cfg.realignment_count_threshold &&
            (now - g_last_realignment_ms) > cfg.alignment_timeout_ms) {
            return true;
        }
    } else {
        g_large_innovation_count = max(0, g_large_innovation_count - 1); // Decay counter
    }
    return false;
}


inline bool ekfYawUpdate(float imu_yaw_rad, const EkfYawConfig& cfg = EkfYawConfig()){
    if (!isfinite(imu_yaw_rad)) {
        Serial.println("EKF: Invalid IMU yaw measurement");
        return false;
    }

    // Initial alignment or re-alignment
    if (!g_aligned || shouldRealign(0.0f, cfg)) { // check innovation after first calculation
        g_yaw_offset = wrapToPi(robot_theta - imu_yaw_rad);
        g_aligned = true;
        g_last_realignment_ms = millis();
        g_large_innovation_count = 0;
        Serial.printf("EKF: Yaw %saligned, offset=%.2f deg\n", 
                     g_last_realignment_ms == 0 ? "" : "re-", 
                     g_yaw_offset * 180.0f / M_PI);
    }

    float z = wrapToPi(imu_yaw_rad + g_yaw_offset);

    float P[9];
    covarianceToMatrix(covariance, P);

    // Innovation
    float y = wrapToPi(z - robot_theta);

    if (shouldRealign(y, cfg)) {
        // Serial.printf("EKF: Re-aligning due to large innovation: %.2f deg\n", y * 180.0f / M_PI);
        g_yaw_offset = wrapToPi(robot_theta - imu_yaw_rad);
        g_last_realignment_ms = millis();
        g_large_innovation_count = 0;
        z = wrapToPi(imu_yaw_rad + g_yaw_offset);
        y = wrapToPi(z - robot_theta);
    }

    // Innovation covariance
    float S = P[8] + cfg.R_yaw_rad2;
    if(S<=1e-12f){
        // Serial.println("EKF: Innovation covariance too small");
        return false;
    } 
    
    // Mahalanobis distance for gating
    if(cfg.gate_sigma > 0.0f){
        float maha2 = (y * y) / S;
        if (maha2 > cfg.gate_sigma * cfg.gate_sigma) {
            // Serial.printf("EKF: Measurement gated, Maha=%.2f > %.2f\n", 
            //              sqrt(maha2), cfg.gate_sigma);
            return false;
        }
    }

    // Kalman gain K = P H^T / S = [P(x,θ), P(y,θ), P(θ,θ)]^T / S
    float Kx = P[2] / S;  // P(x,θ)
    float Ky = P[5] / S;  // P(y,θ)
    float Kt = P[8] / S;  // P(θ,θ)

    // State update
    robot_x    += Kx * y;
    robot_y    += Ky * y;
    robot_theta= wrapToPi(robot_theta + Kt * y);

    // P <- (I-KH)P(I-KH)^T + K R K^T ; with H=[0 0 1]
    float HP[3] = {P[2], P[5], P[8]};

    // Update covariance: P = P - K*(H*P)
    // Row 0: P(x,·)
    P[0] -= Kx * HP[0]; // P(x,x)
    P[1] -= Kx * HP[1]; // P(x,y)
    P[2] -= Kx * HP[2]; // P(x,θ)
    
    // Row 1: P(y,·) - FIXED INDEXING
    P[3] -= Ky * HP[1]; // P(y,y) - was incorrectly HP[0]
    P[4] -= Ky * HP[2]; // P(y,θ) - was incorrectly HP[1]
    
    // Row 2: P(θ,·)
    P[6] -= Kt * HP[0]; // P(θ,x)
    P[7] -= Kt * HP[1]; // P(θ,y)
    P[8] -= Kt * HP[2]; // P(θ,θ)

    // Add K*R*K^T term for numerical stability
    float R = cfg.R_yaw_rad2;
    P[0] += Kx * Kx * R; // P(x,x)
    P[1] += Kx * Ky * R; // P(x,y)
    P[2] += Kx * Kt * R; // P(x,θ)
    P[3] += Ky * Ky * R; // P(y,y)
    P[4] += Ky * Kt * R; // P(y,θ)
    P[6] += Kt * Kx * R; // P(θ,x)  
    P[7] += Kt * Ky * R; // P(θ,y)
    P[8] += Kt * Kt * R; // P(θ,θ)

    // Ensure symmetry (numerical precision)
    P[3] = P[1]; // P(y,x) = P(x,y)
    P[6] = P[2]; // P(θ,x) = P(x,θ)
    P[7] = P[4]; // P(θ,y) = P(y,θ)

    matrixToCovariance(P, covariance);
    covariance[0] = fmaxf(covariance[0], 1e-9f);
    covariance[3] = fmaxf(covariance[3], 1e-9f);
    covariance[5] = fmaxf(covariance[5], 1e-9f);


    return true; // Measurement accepted and state updated
}

inline void getEkfAlignmentInfo(bool& is_aligned, float& offset_deg, unsigned long& last_alignment_ms) {
    is_aligned = g_aligned;
    offset_deg = g_yaw_offset * 180.0f / M_PI;
    last_alignment_ms = g_last_realignment_ms;
}

#endif // EKF_LOCALIZATION_H