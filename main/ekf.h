#ifndef EKF_IMU_H
#define EKF_IMU_H

#include <math.h>

// ============================================================
// EKF_IMU  -  Quaternion-measurement Extended Kalman Filter
//
// New architecture (matches the "EKF - actual mathematics for
// quaternion attitude fusion" reference):
//
//   MPU raw data  -> Madgwick -> q_MPU  \
//                                         >--> EKF (this class) -> q_FINAL -> RPY
//   ICM raw data  -> Madgwick -> q_ICM  /
//
// predict()  : propagates the state quaternion using gyro data,
//              exactly like a normal quaternion-kinematics EKF.
// update()   : takes ONE already-computed quaternion (either
//              q_MPU or q_ICM) as a direct measurement, i.e.
//              z = q, H = Identity(4x4).
//
// Because the measurement noise is uncorrelated between the two
// sensors, calling update() twice per loop (once with q_MPU, once
// with q_ICM) is mathematically identical to stacking them into a
// single 8-length measurement vector and doing one 8x8 solve - but
// it only ever needs a 4x4 inversion, which is far cheaper on a
// Teensy. This is the "sequential measurement update" trick.
// ============================================================
class EKF_IMU {
public:
    // State: Quaternion [q0, q1, q2, q3]
    float q[4];

    // Estimated Euler Angles (in degrees), updated after every update()
    float roll;
    float pitch;
    float yaw;

    // 4x4 State Covariance Matrix
    float P[4][4];

    // Process Noise Variance (Gyroscope)
    float Q_gyro;

    // Measurement Noise Variance for each source quaternion.
    // Smaller = "I trust this sensor's Madgwick output more".
    float R_mpu;
    float R_icm;

    EKF_IMU(float gyro_noise = 0.005f, float meas_noise_mpu = 0.05f, float meas_noise_icm = 0.05f) {
        Q_gyro = gyro_noise;
        R_mpu  = meas_noise_mpu;
        R_icm  = meas_noise_icm;
        reset();
    }

    void reset() {
        q[0] = 1.0f;
        q[1] = 0.0f;
        q[2] = 0.0f;
        q[3] = 0.0f;

        for (int i = 0; i < 4; i++) {
            for (int j = 0; j < 4; j++) {
                P[i][j] = (i == j) ? 0.01f : 0.0f;
            }
        }

        roll = pitch = yaw = 0.0f;
    }

    // --- 1. PREDICT STEP ---
    // Inputs: gx, gy, gz in radians/sec, dt in seconds
    // (Same quaternion-kinematics propagation as the original filter.)
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

    // --- Innovation-based sensor selection helper ---
    // Returns a "distance" between the EKF's current predicted quaternion
    // and a candidate measurement quaternion: 0 = identical orientation,
    // up to 1 = maximally different. Handles the q / -q sign ambiguity
    // (same rotation) by taking the absolute value of the dot product.
    // Call this on q_MPU and q_ICM right after predict(); whichever comes
    // back smaller is the one to feed into update() this cycle.
    float residualMagnitude(const float qMeas[4]) const {
        float dot = q[0]*qMeas[0] + q[1]*qMeas[1] + q[2]*qMeas[2] + q[3]*qMeas[3];
        return 1.0f - fabsf(dot);
    }

    // --- 2. UPDATE STEP (direct quaternion measurement, H = I) ---
    // qMeas : a unit quaternion coming out of one of the Madgwick
    //         filters (q_MPU or q_ICM).
    // R     : measurement noise variance to use for this source
    //         (pass R_mpu or R_icm from the call site).
    void update(const float qMeas[4], float R) {

        // Quaternion sign ambiguity: q and -q represent the same
        // rotation. If the measurement is pointing the "opposite way"
        // in quaternion space relative to our current estimate, flip
        // its sign before using it, or the innovation will blow up.
        float dot = q[0]*qMeas[0] + q[1]*qMeas[1] + q[2]*qMeas[2] + q[3]*qMeas[3];
        float sign = (dot < 0.0f) ? -1.0f : 1.0f;
        float z[4] = { sign*qMeas[0], sign*qMeas[1], sign*qMeas[2], sign*qMeas[3] };

        // Innovation: y = z - h(q), h(q) = q  (H = Identity)
        float y[4];
        for (int i = 0; i < 4; i++) y[i] = z[i] - q[i];

        // Innovation covariance: S = H*P*H^T + R = P + R*I
        float S[4][4];
        for (int i = 0; i < 4; i++) {
            for (int j = 0; j < 4; j++) {
                S[i][j] = P[i][j] + ((i == j) ? R : 0.0f);
            }
        }

        float S_inv[4][4];
        if (!invert4x4(S, S_inv)) return; // singular - skip this update

        // Kalman Gain: K = P * H^T * S_inv = P * S_inv  (since H = I)
        float K[4][4] = {0};
        for (int i = 0; i < 4; i++) {
            for (int j = 0; j < 4; j++) {
                for (int k = 0; k < 4; k++) {
                    K[i][j] += P[i][k] * S_inv[k][j];
                }
            }
        }

        // State update: q = q + K*y
        for (int i = 0; i < 4; i++) {
            for (int j = 0; j < 4; j++) {
                q[i] += K[i][j] * y[j];
            }
        }

        // Covariance update: P = (I - K) * P   (since H = I)
        float IK[4][4];
        for (int i = 0; i < 4; i++) {
            for (int j = 0; j < 4; j++) {
                IK[i][j] = ((i == j) ? 1.0f : 0.0f) - K[i][j];
            }
        }

        float P_new[4][4] = {0};
        for (int i = 0; i < 4; i++) {
            for (int j = 0; j < 4; j++) {
                for (int k = 0; k < 4; k++) {
                    P_new[i][j] += IK[i][k] * P[k][j];
                }
            }
        }
        for (int i = 0; i < 4; i++)
            for (int j = 0; j < 4; j++)
                P[i][j] = P_new[i][j];

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
        float sinr_cosp = 2.0f * (q[0] * q[1] + q[2] * q[3]);
        float cosr_cosp = 1.0f - 2.0f * (q[1] * q[1] + q[2] * q[2]);
        roll = atan2f(sinr_cosp, cosr_cosp) * (180.0f / M_PI);

        float sinp = 2.0f * (q[0] * q[2] - q[3] * q[1]);
        if (fabsf(sinp) >= 1.0f) {
            pitch = copysignf(90.0f, sinp);
        } else {
            pitch = asinf(sinp) * (180.0f / M_PI);
        }

        float siny_cosp = 2.0f * (q[0] * q[3] + q[1] * q[2]);
        float cosy_cosp = 1.0f - 2.0f * (q[2] * q[2] + q[3] * q[3]);
        yaw = atan2f(siny_cosp, cosy_cosp) * (180.0f / M_PI);
    }

    // Generic 4x4 matrix inversion via Gauss-Jordan elimination with
    // partial pivoting. Needed because S = P + R*I is a general
    // symmetric 4x4 matrix (not the 3x3 accel-only case the old
    // filter had an analytic formula for).
    bool invert4x4(const float Ain[4][4], float inv[4][4]) {
        float A[4][8];
        for (int i = 0; i < 4; i++) {
            for (int j = 0; j < 4; j++) A[i][j] = Ain[i][j];
            for (int j = 0; j < 4; j++) A[i][4 + j] = (i == j) ? 1.0f : 0.0f;
        }

        for (int col = 0; col < 4; col++) {
            // Partial pivot
            int pivotRow = col;
            float maxVal = fabsf(A[col][col]);
            for (int r = col + 1; r < 4; r++) {
                if (fabsf(A[r][col]) > maxVal) {
                    maxVal = fabsf(A[r][col]);
                    pivotRow = r;
                }
            }
            if (maxVal < 1e-9f) return false; // singular

            if (pivotRow != col) {
                for (int k = 0; k < 8; k++) {
                    float tmp = A[col][k];
                    A[col][k] = A[pivotRow][k];
                    A[pivotRow][k] = tmp;
                }
            }

            float pivot = A[col][col];
            for (int k = 0; k < 8; k++) A[col][k] /= pivot;

            for (int r = 0; r < 4; r++) {
                if (r == col) continue;
                float factor = A[r][col];
                if (factor == 0.0f) continue;
                for (int k = 0; k < 8; k++) {
                    A[r][k] -= factor * A[col][k];
                }
            }
        }

        for (int i = 0; i < 4; i++)
            for (int j = 0; j < 4; j++)
                inv[i][j] = A[i][4 + j];

        return true;
    }
};

#endif