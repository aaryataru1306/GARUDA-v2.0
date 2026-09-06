#ifndef EKF_IMU_H
#define EKF_IMU_H

#include <math.h>

class EKF_IMU {
public:
    // State: Quaternion [q0, q1, q2, q3]
    float q[4];

    // Estimated Euler Angles (in degrees)
    float roll;
    float pitch;
    float yaw;

    // 4x4 State Covariance Matrix
    float P[4][4];

    // Process Noise Variance (Gyroscope)
    float Q_gyro;

    // Measurement Noise Variance (Accelerometer)
    float R_accel;

    EKF_IMU(float gyro_noise = 0.005f, float accel_noise = 0.5f) {
        Q_gyro = gyro_noise;
        R_accel = accel_noise;
        reset();
    }

    void reset() {
        // Initial quaternion (Identity / Level)
        q[0] = 1.0f;
        q[1] = 0.0f;
        q[2] = 0.0f;
        q[3] = 0.0f;

        // Initialize Covariance Matrix to small identity
        for (int i = 0; i < 4; i++) {
            for (int j = 0; j < 4; j++) {
                P[i][j] = (i == j) ? 0.01f : 0.0f;
            }
        }

        roll = pitch = yaw = 0.0f;
    }

    // --- 1. PREDICT STEP ---
    // Inputs: gx, gy, gz in radians/sec, dt in seconds
    void predict(float gx, float gy, float gz, float dt) {
        float q0 = q[0], q1 = q[1], q2 = q[2], q3 = q[3];

        // State Transition Jacobian (F = I + 0.5 * dt * Omega)
        float F[4][4] = {
            { 1.0f,          -0.5f*gx*dt, -0.5f*gy*dt, -0.5f*gz*dt },
            { 0.5f*gx*dt,     1.0f,        0.5f*gz*dt, -0.5f*gy*dt },
            { 0.5f*gy*dt,    -0.5f*gz*dt,  1.0f,        0.5f*gx*dt },
            { 0.5f*gz*dt,     0.5f*gy*dt, -0.5f*gx*dt,  1.0f       }
        };

        // Propagate State: q_k = F * q_(k-1)
        float q_new[4];
        for (int i = 0; i < 4; i++) {
            q_new[i] = 0.0f;
            for (int j = 0; j < 4; j++) {
                q_new[i] += F[i][j] * q[j];
            }
        }
        for (int i = 0; i < 4; i++) q[i] = q_new[i];

        // Process Noise Matrix Q = (0.5 * dt)^2 * Q_gyro * I_4
        float q_factor = 0.25f * dt * dt * Q_gyro;

        // Propagate Covariance: P = F * P * F^T + Q
        float FP[4][4] = {0};
        for (int i = 0; i < 4; i++) {
            for (int j = 0; j < 4; j++) {
                for (int k = 0; k < 4; k++) {
                    FP[i][j] += F[i][k] * P[k][j];
                }
            }
        }

        for (int i = 0; i < 4; i++) {
            for (int j = 0; j < 4; j++) {
                float sum = 0.0f;
                for (int k = 0; k < 4; k++) {
                    sum += FP[i][k] * F[j][k]; // F[j][k] is F^T[k][j]
                }
                P[i][j] = sum + (i == j ? q_factor : 0.0f);
            }
        }

        normalizeQuaternion();
    }

    // --- 2. UPDATE STEP ---
    // Inputs: ax, ay, az (normalized or raw accelerometer readings)
    void update(float ax, float ay, float az) {
        // Normalize accelerometer readings
        float norm = sqrtf(ax * ax + ay * ay + az * az);
        if (norm < 0.0001f) return; // Discard invalid reading
        ax /= norm;
        ay /= norm;
        az /= norm;

        float q0 = q[0], q1 = q[1], q2 = q[2], q3 = q[3];

        // Measurement Model h(q): Estimated gravity vector in body frame
        float h[3] = {
            2.0f * (q1 * q3 - q0 * q2),
            2.0f * (q0 * q1 + q2 * q3),
            q0 * q0 - q1 * q1 - q2 * q2 + q3 * q3
        };

        // Innovation / Residual: y = z - h(q)
        float y[3] = { ax - h[0], ay - h[1], az - h[2] };

        // Measurement Jacobian H (3x4)
        float H[3][4] = {
            { -2.0f * q2,   2.0f * q3,  -2.0f * q0,   2.0f * q1 },
            {  2.0f * q1,   2.0f * q0,   2.0f * q3,   2.0f * q2 },
            {  2.0f * q0,  -2.0f * q1,  -2.0f * q2,   2.0f * q3 }
        };

        // P * H^T (4x3)
        float PHT[4][3] = {0};
        for (int i = 0; i < 4; i++) {
            for (int j = 0; j < 3; j++) {
                for (int k = 0; k < 4; k++) {
                    PHT[i][j] += P[i][k] * H[j][k];
                }
            }
        }

        // S = H * (P * H^T) + R (3x3)
        float S[3][3] = {0};
        for (int i = 0; i < 3; i++) {
            for (int j = 0; j < 3; j++) {
                float sum = 0.0f;
                for (int k = 0; k < 4; k++) {
                    sum += H[i][k] * PHT[k][j];
                }
                S[i][j] = sum + (i == j ? R_accel : 0.0f);
            }
        }

        // Invert 3x3 Matrix S using analytic matrix inversion
        float S_inv[3][3];
        if (!invert3x3(S, S_inv)) return;

        // Kalman Gain: K = (P * H^T) * S_inv (4x3)
        float K[4][3] = {0};
        for (int i = 0; i < 4; i++) {
            for (int j = 0; j < 3; j++) {
                for (int k = 0; k < 3; k++) {
                    K[i][j] += PHT[i][k] * S_inv[k][j];
                }
            }
        }

        // Update State: q = q + K * y
        for (int i = 0; i < 4; i++) {
            for (int j = 0; j < 3; j++) {
                q[i] += K[i][j] * y[j];
            }
        }

        // Update Covariance: P = (I - K * H) * P
        float I_KH[4][4];
        for (int i = 0; i < 4; i++) {
            for (int j = 0; j < 4; j++) {
                float sum = (i == j) ? 1.0f : 0.0f;
                for (int k = 0; k < 3; k++) {
                    sum -= K[i][k] * H[k][j];
                }
                I_KH[i][j] = sum;
            }
        }

        float P_new[4][4] = {0};
        for (int i = 0; i < 4; i++) {
            for (int j = 0; j < 4; j++) {
                for (int k = 0; k < 4; k++) {
                    P_new[i][j] += I_KH[i][k] * P[k][j];
                }
            }
        }

        for (int i = 0; i < 4; i++) {
            for (int j = 0; j < 4; j++) {
                P[i][j] = P_new[i][j];
            }
        }

        normalizeQuaternion();
        computeEulerAngles();
    }

private:
    void normalizeQuaternion() {
        float norm = sqrtf(q[0]*q[0] + q[1]*q[1] + q[2]*q[2] + q[3]*q[3]);
        if (norm > 0.0001f) {
            q[0] /= norm;
            q[1] /= norm;
            q[2] /= norm;
            q[3] /= norm;
        }
    }

    void computeEulerAngles() {
        // Roll (X-axis rotation)
        float sinr_cosp = 2.0f * (q[0] * q[1] + q[2] * q[3]);
        float cosr_cosp = 1.0f - 2.0f * (q[1] * q[1] + q[2] * q[2]);
        roll = atan2f(sinr_cosp, cosr_cosp) * (180.0f / M_PI);

        // Pitch (Y-axis rotation)
        float sinp = 2.0f * (q[0] * q[2] - q[3] * q[1]);
        if (fabsf(sinp) >= 1.0f) {
            pitch = copysignf(90.0f, sinp); // 90 degrees if out of range
        } else {
            pitch = asinf(sinp) * (180.0f / M_PI);
        }

        // Yaw (Z-axis rotation)
        float siny_cosp = 2.0f * (q[0] * q[3] + q[1] * q[2]);
        float cosy_cosp = 1.0f - 2.0f * (q[2] * q[2] + q[3] * q[3]);
        yaw = atan2f(siny_cosp, cosy_cosp) * (180.0f / M_PI);
    }

    // Analytic inversion for 3x3 matrix
    bool invert3x3(const float A[3][3], float inv[3][3]) {
        float det = A[0][0] * (A[1][1] * A[2][2] - A[1][2] * A[2][1]) -
                    A[0][1] * (A[1][0] * A[2][2] - A[1][2] * A[2][0]) +
                    A[0][2] * (A[1][0] * A[2][1] - A[1][1] * A[2][0]);

        if (fabsf(det) < 1e-6f) return false;

        float invDet = 1.0f / det;

        inv[0][0] = (A[1][1] * A[2][2] - A[1][2] * A[2][1]) * invDet;
        inv[0][1] = (A[0][2] * A[2][1] - A[0][0] * A[2][2]) * invDet; // cofactor transposed
        inv[0][2] = (A[0][1] * A[1][2] - A[0][2] * A[1][1]) * invDet;

        inv[1][0] = (A[1][2] * A[2][0] - A[1][0] * A[2][2]) * invDet;
        inv[1][1] = (A[0][0] * A[2][2] - A[0][2] * A[2][0]) * invDet;
        inv[1][2] = (A[0][2] * A[1][0] - A[0][0] * A[1][2]) * invDet;

        inv[2][0] = (A[1][0] * A[2][1] - A[1][1] * A[2][0]) * invDet;
        inv[2][1] = (A[0][1] * A[2][0] - A[0][0] * A[2][1]) * invDet;
        inv[2][2] = (A[0][0] * A[1][1] - A[0][1] * A[1][0]) * invDet;

        return true;
    }
};

#endif