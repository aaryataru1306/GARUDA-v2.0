#ifndef EKF_IMU_H
#define EKF_IMU_H

#include <math.h>

// ============================================================
// EKF_IMU  -  Quaternion-state Extended Kalman Filter
//
// Current architecture (Madgwick removed — raw accel feeds the EKF directly):
//
//   MPU gyro  --+                                  MPU accel  --+
//               +--> gyro fusion/selection -> predict()          +--> updateAccel() -> RPY
//   ICM gyro  --+                                  ICM accel  --+
//
// predict()    : propagates the state quaternion using gyro data
//                (quaternion-kinematics EKF prediction step, unchanged).
// updateAccel(): NEW. Takes a calibrated raw accelerometer reading
//                (ax, ay, az straight from mpu.getMotion6()/icm.getMotion6(),
//                no Madgwick step) and corrects roll/pitch using gravity as
//                the reference. h(q) = predicted gravity direction in the
//                body frame; H = dh/dq is the same 3x4 Jacobian used by
//                standard gradient-descent AHRS filters (Madgwick), but the
//                correction size here comes from the actual Kalman gain
//                (P, R), not a fixed step size.
// update()     : the original direct-quaternion measurement update
//                (z = q, H = Identity(4x4)) — kept for compatibility/testing,
//                not used by the current raw-accel loop.
//
// Calling updateAccel() twice per loop (once with MPU accel, once with ICM
// accel) is the same "sequential measurement update" trick as before: it's
// equivalent to stacking both sources into one 6-length measurement vector,
// but only ever needs 3x3 inversions instead of a 6x6 one.
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

    float residualMagnitude(const float qMeas[4]) const {
        float dot = q[0]*qMeas[0] + q[1]*qMeas[1] + q[2]*qMeas[2] + q[3]*qMeas[3];
        return 1.0f - fabsf(dot);
    }

    void update(const float qMeas[4], float R) {

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

    void updateAccel(float ax, float ay, float az, float R) {
        float normAcc = sqrtf(ax * ax + ay * ay + az * az);
        if (normAcc < 1e-6f) return;
        float invNorm = 1.0f / normAcc;
        float axN = ax * invNorm, ayN = ay * invNorm, azN = az * invNorm;

        float q0 = q[0], q1 = q[1], q2 = q[2], q3 = q[3];

        float hx = 2.0f * (q1 * q3 - q0 * q2);
        float hy = 2.0f * (q0 * q1 + q2 * q3);
        float hz = 1.0f - 2.0f * (q1 * q1 + q2 * q2); // = q0^2-q1^2-q2^2+q3^2 for a unit quaternion

        // Innovation
        float y[3] = { axN - hx, ayN - hy, azN - hz };

        // Jacobian H = dh/dq (3x4) — same structure as the Madgwick gradient matrix
        float H[3][4] = {
            { -2.0f * q2,  2.0f * q3, -2.0f * q0,  2.0f * q1 },
            {  2.0f * q1,  2.0f * q0,  2.0f * q3,  2.0f * q2 },
            {  0.0f,      -4.0f * q1, -4.0f * q2,  0.0f      }
        };

        // Innovation covariance: S = H*P*H^T + R*I (3x3)
        float HP[3][4] = {0};
        for (int i = 0; i < 3; i++)
            for (int j = 0; j < 4; j++)
                for (int k = 0; k < 4; k++)
                    HP[i][j] += H[i][k] * P[k][j];

        float S[3][3];
        for (int i = 0; i < 3; i++) {
            for (int j = 0; j < 3; j++) {
                float sum = 0.0f;
                for (int k = 0; k < 4; k++) sum += HP[i][k] * H[j][k]; // H[j][k] == H^T[k][j]
                S[i][j] = sum + ((i == j) ? R : 0.0f);
            }
        }

        float S_inv[3][3];
        if (!invert3x3(S, S_inv)) return; // singular - skip this update

        // Kalman gain: K = P*H^T*S_inv (4x3)
        float PHt[4][3] = {0};
        for (int i = 0; i < 4; i++)
            for (int j = 0; j < 3; j++)
                for (int k = 0; k < 4; k++)
                    PHt[i][j] += P[i][k] * H[j][k];

        float K[4][3] = {0};
        for (int i = 0; i < 4; i++)
            for (int j = 0; j < 3; j++)
                for (int k = 0; k < 3; k++)
                    K[i][j] += PHt[i][k] * S_inv[k][j];

        // State update: q = q + K*y
        for (int i = 0; i < 4; i++)
            for (int j = 0; j < 3; j++)
                q[i] += K[i][j] * y[j];

        // Covariance update: P = (I - K*H) * P
        float KH[4][4] = {0};
        for (int i = 0; i < 4; i++)
            for (int j = 0; j < 4; j++)
                for (int k = 0; k < 3; k++)
                    KH[i][j] += K[i][k] * H[k][j];

        // Covariance update: P_new = (I - K*H) * P
        float P_new[4][4] = {0};
        for (int i = 0; i < 4; i++) {
            for (int j = 0; j < 4; j++) {
                float sum = 0.0f;
                for (int k = 0; k < 4; k++) {
                    float IK = ((i == k) ? 1.0f : 0.0f) - KH[i][k];
                    sum += IK * P[k][j];
                }
                P_new[i][j] = sum;
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
        pitch = atan2f(sinp, cosr_cosp) * (180.0f / M_PI);

        float siny_cosp = 2.0f * (q[0] * q[3] + q[1] * q[2]);
        float cosy_cosp = 1.0f - 2.0f * (q[2] * q[2] + q[3] * q[3]);
        yaw = atan2f(siny_cosp, cosy_cosp) * (180.0f / M_PI);
    }

    bool invert3x3(const float A[3][3], float inv[3][3]) {
        float det = A[0][0] * (A[1][1] * A[2][2] - A[1][2] * A[2][1])
                  - A[0][1] * (A[1][0] * A[2][2] - A[1][2] * A[2][0])
                  + A[0][2] * (A[1][0] * A[2][1] - A[1][1] * A[2][0]);
        if (fabsf(det) < 1e-9f) return false; // singular
        float invDet = 1.0f / det;

        inv[0][0] =  (A[1][1] * A[2][2] - A[1][2] * A[2][1]) * invDet;
        inv[0][1] = -(A[0][1] * A[2][2] - A[0][2] * A[2][1]) * invDet;
        inv[0][2] =  (A[0][1] * A[1][2] - A[0][2] * A[1][1]) * invDet;
        inv[1][0] = -(A[1][0] * A[2][2] - A[1][2] * A[2][0]) * invDet;
        inv[1][1] =  (A[0][0] * A[2][2] - A[0][2] * A[2][0]) * invDet;
        inv[1][2] = -(A[0][0] * A[1][2] - A[0][2] * A[1][0]) * invDet;
        inv[2][0] =  (A[1][0] * A[2][1] - A[1][1] * A[2][0]) * invDet;
        inv[2][1] = -(A[0][0] * A[2][1] - A[0][1] * A[2][0]) * invDet;
        inv[2][2] =  (A[0][0] * A[1][1] - A[0][1] * A[1][0]) * invDet;

        return true;
    }

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