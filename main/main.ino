#include 
#include 
#include "mpu.h"
#include "icm.h"
#include "ekf.h"
#include "pid.h"
#include "pm02d.h"
#include 

// ==================================================
// TEENSY 4.1 PIN CONFIGURATION
// ==================================================
constexpr int MPU_CS_PIN = 10;
constexpr int ICM_CS_PIN = 9;
constexpr float THROTTLE_ALPHA = 0.15f;
constexpr int rx_pin = 28;
constexpr uint8_t PPM_MAX_CHANNELS = 10;
volatile uint32_t ppm_last_edge = 0;
volatile uint8_t ppm_channel = 0;
volatile uint16_t ppm_channels[PPM_MAX_CHANNELS];
volatile uint32_t ppm_last_signal = 0;
#define SD_CS BUILTIN_SDCARD

constexpr int MOTOR1_PIN = 22;
constexpr int MOTOR2_PIN = 2;
constexpr int MOTOR3_PIN = 3;
constexpr int MOTOR4_PIN = 4;

constexpr int PWM_FREQUENCY  = 50;    // 50 Hz standard ESC update rate
constexpr int PWM_RESOLUTION = 16;    // 16-bit PWM resolution (0 - 65535)
constexpr int PWM_MAX_DUTY   = 65535;

constexpr float ESC_MIN_US = 1000.0f; // 0% Throttle (Arm / Disarm Pulse)
constexpr float ESC_MAX_US = 2000.0f; // 100% Full Throttle

float baseThrottlePercent = 0.0f;          // Starts safely at 0% after arming
constexpr float MAX_BASE_THROTTLE = 100.0f; // Maximum base throttle allowed via serial
constexpr float MAX_MOTOR_LIMIT   = 100.0f; // Absolute output limit per motor

float m1_throttle = 0.0f;
float m2_throttle = 0.0f;
float m3_throttle = 0.0f;
float m4_throttle = 0.0f;
float filtered_m1 = 0.0f;
float filtered_m2 = 0.0f;
float filtered_m3 = 0.0f;
float filtered_m4 = 0.0f;
File logFile;

uint32_t logTimer = 0;
uint32_t logStartTime = 0;

// ==================================================
// LOW-PASS FILTER STRUCTURE
// ==================================================
struct LowPassFilter {
    float cutoffHz;
    float state;
    bool initialized;

    LowPassFilter(float cutoff = 30.0f) : cutoffHz(cutoff), state(0.0f), initialized(false) {}

    float update(float input, float dt) {
        if (!initialized) {
            state = input;
            initialized = true;
            return state;
        }
        float alpha = (2.0f * M_PI * cutoffHz * dt) / (1.0f + 2.0f * M_PI * cutoffHz * dt);
        alpha = constrain(alpha, 0.0f, 1.0f);
        state += alpha * (input - state);
        return state;
    }

    void reset() {
        initialized = false;
        state = 0.0f;
    }
};

// LPF instances for sensor signals (30 Hz Cutoff for Gyro, 10 Hz Cutoff for Accel)
LowPassFilter lpf_mpu_ax(10.0f), lpf_mpu_ay(10.0f), lpf_mpu_az(10.0f);
LowPassFilter lpf_mpu_gx(30.0f), lpf_mpu_gy(30.0f), lpf_mpu_gz(30.0f);

LowPassFilter lpf_icm_ax(10.0f), lpf_icm_ay(10.0f), lpf_icm_az(10.0f);
LowPassFilter lpf_icm_gx(30.0f), lpf_icm_gy(30.0f), lpf_icm_gz(30.0f);

// ==================================================
// GYRO VARIANCE TRACKING
// ==================================================
constexpr uint8_t VAR_WINDOW_SIZE = 10;

struct GyroVarianceTracker {
  float samplesX[VAR_WINDOW_SIZE] = {0};
  float samplesY[VAR_WINDOW_SIZE] = {0};
  float samplesZ[VAR_WINDOW_SIZE] = {0};
  uint8_t index = 0;
  bool filled = false;

  void addSample(float gx, float gy, float gz) {
    samplesX[index] = gx;
    samplesY[index] = gy;
    samplesZ[index] = gz;
    index = (index + 1) % VAR_WINDOW_SIZE;
    if (index == 0) filled = true;
  }

  float getVarianceSum() const {
    uint8_t count = filled ? VAR_WINDOW_SIZE : index;
    if (count < 2) return 0.0f;

    float meanX = 0, meanY = 0, meanZ = 0;
    for (uint8_t i = 0; i < count; i++) {
      meanX += samplesX[i];
      meanY += samplesY[i];
      meanZ += samplesZ[i];
    }
    meanX /= count; meanY /= count; meanZ /= count;

    float varX = 0, varY = 0, varZ = 0;
    for (uint8_t i = 0; i < count; i++) {
      varX += (samplesX[i] - meanX) * (samplesX[i] - meanX);
      varY += (samplesY[i] - meanY) * (samplesY[i] - meanY);
      varZ += (samplesZ[i] - meanZ) * (samplesZ[i] - meanZ);
    }
    return (varX + varY + varZ) / count;
  }
};

GyroVarianceTracker mpuVarTracker;
GyroVarianceTracker icmVarTracker;

// ==================================================
// ATTITUDE / RATE CONTROL ARCHITECTURE
// ==================================================
constexpr float ANGLE_ROLL_KP   = 4.0f;   // deg error -> deg/s
constexpr float ANGLE_PITCH_KP  = 4.0f;   // deg error -> deg/s

constexpr float MAX_ROLL_RATE   = 250.0f; // deg/s
constexpr float MAX_PITCH_RATE  = 250.0f; // deg/s
constexpr float MAX_YAW_RATE    = 250.0f;

PID_struct rate_roll_pid;
PID_struct rate_pitch_pid;
PID_struct rate_yaw_pid;

SPISettings spiSettings(1000000, MSBFIRST, SPI_MODE0);
MPU9250 mpu(MPU_CS_PIN, spiSettings);
ICM20948 icm(ICM_CS_PIN, spiSettings);
PM02D powerMonitor(Wire, PM02D_I2C_ADDR);
PM02D_Data powerData = {
    0.0f,
    0.0f,
    0.0f,
    0.0f,
    0.0f,
    false
};

EKF_IMU fusedEKF(0.005f, 0.6f);

unsigned long previousTime = 0;

uint32_t microsecondsToDuty(float pulseUs) {
  float periodUs = 1000000.0f / (float)PWM_FREQUENCY;
  pulseUs = constrain(pulseUs, ESC_MIN_US, ESC_MAX_US);
  return (uint32_t)((pulseUs / periodUs) * (float)PWM_MAX_DUTY);
}

void writeRawPulseUs(int pin, float pulseUs) {
  uint32_t duty = microsecondsToDuty(pulseUs);
  analogWrite(pin, duty);
}

void writeMotorThrottle(int pin, float percent) {
  percent = constrain(percent, 0.0f, MAX_MOTOR_LIMIT);
  float pulseUs = ESC_MIN_US + (percent / 100.0f) * (ESC_MAX_US - ESC_MIN_US);
  writeRawPulseUs(pin, pulseUs);
}

void writeAllMotorsRaw(float pulseUs) {
  writeRawPulseUs(MOTOR1_PIN, pulseUs);
  writeRawPulseUs(MOTOR2_PIN, pulseUs);
  writeRawPulseUs(MOTOR3_PIN, pulseUs);
  writeRawPulseUs(MOTOR4_PIN, pulseUs);
}

void ppm_isr()
{
    uint32_t now = micros();
    uint32_t dt = now - ppm_last_edge;

    ppm_last_edge = now;

    // Long gap = start of a new PPM frame
    if (dt > 3000)
    {
        ppm_channel = 0;
    }
    else if (dt >= 900 && dt <= 2200)
    {
        if (ppm_channel < PPM_MAX_CHANNELS)
        {
            ppm_channels[ppm_channel] = (uint16_t)dt;
            ppm_channel++;
        }

        ppm_last_signal = now;
    }
}

float read_throttle()
{
    uint16_t throttlePulse;
    uint32_t lastSignal;

    noInterrupts();

    throttlePulse = ppm_channels[2];   // CH3 = throttle
    lastSignal = ppm_last_signal;

    interrupts();

    // Receiver signal lost
    if ((micros() - lastSignal) > 100000)
    {
        return 0.0f;
    }

    // Invalid throttle
    if (throttlePulse < 900 || throttlePulse > 2100)
    {
        return 0.0f;
    }

    float throttle =
        (throttlePulse - 1000.0f) / 10.0f;

    return constrain(throttle, 0.0f, 100.0f);
}

// ==================================================
// SETUP
// ==================================================
void setup() {
  Wire.begin();
  Wire.setClock(400000);
  Serial.begin(115200);

  // ==================================================
  // SD CARD INITIALIZATION
  // ==================================================
  if (!SD.begin(BUILTIN_SDCARD)) {
      Serial.println("SD CARD INIT FAILED!");
  }
  else {
      Serial.println("SD CARD INITIALIZED.");

      logFile = SD.open("flight.csv", FILE_WRITE);

      if (logFile) {
          // Write CSV header only if file is new/empty
          if (logFile.size() == 0) {
              logFile.println(
                  "Time_ms,Throttle,Roll,Pitch,"
                  "GyroRollRate,GyroPitchRate,GyroYawRate,"
                  "DesRollRate,DesPitchRate,DesYawRate,"
                  "RollRateErr,PitchRateErr,YawRateErr,"
                  "RollPID,PitchPID,YawPID,"
                  "M1,M2,M3,M4,"
                  "Voltage,Current,Power"
              );
              logFile.flush();
          }

          logStartTime = millis();

          Serial.println("LOGGING STARTED.");
      }
      else {
          Serial.println("FAILED TO OPEN flight.csv");
      }
  }

  pinMode(rx_pin, INPUT);
  attachInterrupt(digitalPinToInterrupt(rx_pin), ppm_isr, RISING);

  // Configure Hardware PWM
  analogWriteResolution(PWM_RESOLUTION);
  analogWriteFrequency(MOTOR1_PIN, PWM_FREQUENCY);
  analogWriteFrequency(MOTOR2_PIN, PWM_FREQUENCY);
  analogWriteFrequency(MOTOR3_PIN, PWM_FREQUENCY);
  analogWriteFrequency(MOTOR4_PIN, PWM_FREQUENCY);

  // Send minimum PWM pulse immediately on boot
  writeAllMotorsRaw(ESC_MIN_US);

  SPI.begin();

  if (!mpu.begin()) {
    while (1); // Halt on sensor failure
  }

  uint8_t icmWho = 0;
  if (!icm.begin(icmWho)) {
    while (1); // Halt on sensor failure
  }

  if (!powerMonitor.begin()) {
      Serial.println("PM02D / INA228 NOT FOUND!");
  } else {
      Serial.println("PM02D / INA228 detected.");
  }

  // Calibrate IMUs on boot up
  mpu.calibrate();
  icm.calibrate();

  // Initialize the inner rate-loop PID controllers
  PID_init(&rate_roll_pid,  0.15f, 0.001f, 0.002f, -15.0f, 15.0f);
  PID_init(&rate_pitch_pid, 0.15f, 0.001f, 0.002f, -15.0f, 15.0f);
  PID_init(&rate_yaw_pid,   3.2f, 0.03f, 0.1f, -10.0f, 10.0f);

  // Holding disarm pulse (1000us) for 3s to allow standard ESC initialization chimes
  uint32_t armStart = millis();
  while (millis() - armStart < 3000) {
    writeAllMotorsRaw(ESC_MIN_US);
    delay(20);
  }

  previousTime = micros();
}

float smoothThrottle(float target, float current, float alpha)
{
    return current + alpha * (target - current);
}

void loop() {
  // --- Receiver / Serial Input ---
  float read_PWM = read_throttle();
  baseThrottlePercent = read_PWM;
  
  if (Serial.available() > 0) {
    String input = Serial.readStringUntil('\n');
    input.trim();

    if (input.equalsIgnoreCase("STOP") || input == "0" || input == " ") {
      baseThrottlePercent = 0.0f;
    } else if (input.length() > 0) {
      float val = input.toFloat();
      if (val >= 0.0f && val <= MAX_BASE_THROTTLE) {
        baseThrottlePercent = val;
      }
    }
  }

  // --- Timing & Loop Calculation ---
  unsigned long currentTime = micros();
  float dt = (currentTime - previousTime) / 1000000.0f;
  previousTime = currentTime;

  if (dt <= 0.0f || dt > 0.2f) dt = 0.01f;

  // 1. Read raw motion vectors from both sensors
  float raw_mpu_ax, raw_mpu_ay, raw_mpu_az, raw_mpu_gx, raw_mpu_gy, raw_mpu_gz;
  float raw_icm_ax, raw_icm_ay, raw_icm_az, raw_icm_gx, raw_icm_gy, raw_icm_gz;

  mpu.getMotion6(raw_mpu_ax, raw_mpu_ay, raw_mpu_az, raw_mpu_gx, raw_mpu_gy, raw_mpu_gz);
  icm.getMotion6(raw_icm_ax, raw_icm_ay, raw_icm_az, raw_icm_gx, raw_icm_gy, raw_icm_gz);

  // 2. Apply Low-Pass Filtering
  float mpu_ax = lpf_mpu_ax.update(raw_mpu_ax, dt);
  float mpu_ay = lpf_mpu_ay.update(raw_mpu_ay, dt);
  float mpu_az = lpf_mpu_az.update(raw_mpu_az, dt);
  float mpu_gx = lpf_mpu_gx.update(raw_mpu_gx, dt);
  float mpu_gy = lpf_mpu_gy.update(raw_mpu_gy, dt);
  float mpu_gz = lpf_mpu_gz.update(raw_mpu_gz, dt);

  float icm_ax = lpf_icm_ax.update(raw_icm_ax, dt);
  float icm_ay = lpf_icm_ay.update(raw_icm_ay, dt);
  float icm_az = lpf_icm_az.update(raw_icm_az, dt);
  float icm_gx = lpf_icm_gx.update(raw_icm_gx, dt);
  float icm_gy = lpf_icm_gy.update(raw_icm_gy, dt);
  float icm_gz = lpf_icm_gz.update(raw_icm_gz, dt);

  // 3. Compute variance and select gyro with lower variance
  mpuVarTracker.addSample(mpu_gx, mpu_gy, mpu_gz);
  icmVarTracker.addSample(icm_gx, icm_gy, icm_gz);

  float mpu_var = mpuVarTracker.getVarianceSum();
  float icm_var = icmVarTracker.getVarianceSum();

  float fused_gx_rad, fused_gy_rad, fused_gz_rad;

  if (mpu_var <= icm_var) {
    fused_gx_rad = mpu_gx;
    fused_gy_rad = mpu_gy;
    fused_gz_rad = mpu_gz;
  } else {
    fused_gx_rad = icm_gx;
    fused_gy_rad = icm_gy;
    fused_gz_rad = icm_gz;
  }

  // Convert angular rates to deg/s
  float fused_gx_deg = fused_gx_rad * (180.0f / M_PI);
  float fused_gy_deg = fused_gy_rad * (180.0f / M_PI);
  float fused_gz_deg = - fused_gz_rad * (180.0f / M_PI);

  // 4. EKF Filter Update
  fusedEKF.predict(fused_gx_rad, fused_gy_rad, fused_gz_rad, dt);
  fusedEKF.update(mpu_ax, mpu_ay, mpu_az);
  fusedEKF.update(icm_ax, icm_ay, icm_az);

  static uint32_t powerTimer = 0;

  if (millis() - powerTimer >= 100){
    powerTimer = millis();
    powerMonitor.read(powerData);
  }

  // 5. Closed-loop control: cascaded outer attitude (P) + inner rate (PID)
  float desired_roll_rate  = 0.0f;
  float desired_pitch_rate = 0.0f;
  float desired_yaw_rate   = 0.0f;

  float roll_rate_error  = 0.0f;
  float pitch_rate_error = 0.0f;
  float yaw_rate_error   = 0.0f;

  float roll_output  = 0.0f;
  float pitch_output = 0.0f;
  float yaw_output   = 0.0f;

  if (baseThrottlePercent > 0.0f) {
    desired_roll_rate  = constrain(ANGLE_ROLL_KP   * (0.0f - fusedEKF.roll),  -MAX_ROLL_RATE,  MAX_ROLL_RATE);
    desired_pitch_rate = constrain(ANGLE_PITCH_KP * (0.0f - fusedEKF.pitch), -MAX_PITCH_RATE, MAX_PITCH_RATE);
    desired_yaw_rate   = constrain(4.0f * (0.0f - fusedEKF.yaw), -MAX_YAW_RATE, MAX_YAW_RATE);

    roll_rate_error  = desired_roll_rate  - fused_gx_deg;
    pitch_rate_error = desired_pitch_rate - fused_gy_deg;
    yaw_rate_error   = desired_yaw_rate   - fused_gz_deg;

    roll_output  = PID_update(&rate_roll_pid,  desired_roll_rate,  fused_gx_deg, dt);
    pitch_output = PID_update(&rate_pitch_pid, desired_pitch_rate, fused_gy_deg, dt);
    yaw_output   = PID_update(&rate_yaw_pid,   desired_yaw_rate,   fused_gz_deg, dt);

    // Quad X Motor Mix
    m1_throttle = baseThrottlePercent + pitch_output - roll_output - yaw_output; // front right CW
    m2_throttle = baseThrottlePercent - pitch_output - roll_output + yaw_output; // front left CCW
    m3_throttle = baseThrottlePercent - pitch_output + roll_output - yaw_output; // rear left CW
    m4_throttle = baseThrottlePercent + pitch_output + roll_output + yaw_output; // rear right CCW

  } else {
    PID_reset(&rate_roll_pid);
    PID_reset(&rate_pitch_pid);
    PID_reset(&rate_yaw_pid);

    m1_throttle = m2_throttle = m3_throttle = m4_throttle = 0.0f;
  }

  if (baseThrottlePercent <= 0.0f)
  {
      filtered_m1 = 0.0f;
      filtered_m2 = 0.0f;
      filtered_m3 = 0.0f;
      filtered_m4 = 0.0f;
  }
  else
  {
      filtered_m1 = smoothThrottle(m1_throttle, filtered_m1, THROTTLE_ALPHA);
      filtered_m2 = smoothThrottle(m2_throttle, filtered_m2, THROTTLE_ALPHA);
      filtered_m3 = smoothThrottle(m3_throttle, filtered_m3, THROTTLE_ALPHA);
      filtered_m4 = smoothThrottle(m4_throttle, filtered_m4, THROTTLE_ALPHA);
  }

  // 6. Output to ESCs
  writeMotorThrottle(MOTOR1_PIN, filtered_m1);
  writeMotorThrottle(MOTOR2_PIN, filtered_m2);
  writeMotorThrottle(MOTOR3_PIN, filtered_m3);
  writeMotorThrottle(MOTOR4_PIN, filtered_m4);

  // --- Serial Plotter & Logging Output ---
  static uint32_t debugTimer = 0;

  if (millis() - debugTimer >= 20) // 50 Hz refresh rate
  {
      debugTimer = millis();

      char plotBuffer[400];
      snprintf(plotBuffer, sizeof(plotBuffer),
              "R:%.2f P:%.2f Y:%.2f ",
              fusedEKF.roll,
              fusedEKF.pitch,
              fusedEKF.yaw);

      Serial.println(plotBuffer);

    if (logFile){
        logFile.print(millis() - logStartTime);
        logFile.print(",");

        logFile.print(baseThrottlePercent, 2);
        logFile.print(",");

        logFile.print(fusedEKF.roll, 2);
        logFile.print(",");

        logFile.print(fusedEKF.pitch, 2);
        logFile.print(",");

        logFile.print(fused_gx_deg, 2);
        logFile.print(",");
        logFile.print(fused_gy_deg, 2);
        logFile.print(",");
        logFile.print(fused_gz_deg, 2);
        logFile.print(",");

        logFile.print(desired_roll_rate, 2);
        logFile.print(",");
        logFile.print(desired_pitch_rate, 2);
        logFile.print(",");
        logFile.print(desired_yaw_rate, 2);
        logFile.print(",");

        logFile.print(roll_rate_error, 2);
        logFile.print(",");
        logFile.print(pitch_rate_error, 2);
        logFile.print(",");
        logFile.print(yaw_rate_error, 2);
        logFile.print(",");

        logFile.print(roll_output, 2);
        logFile.print(",");
        logFile.print(pitch_output, 2);
        logFile.print(",");
        logFile.print(yaw_output, 2);
        logFile.print(",");

        logFile.print(filtered_m1, 2);
        logFile.print(",");

        logFile.print(filtered_m2, 2);
        logFile.print(",");

        logFile.print(filtered_m3, 2);
        logFile.print(",");

        logFile.print(filtered_m4, 2);
        logFile.print(",");

        logFile.print(powerData.voltageV, 2);
        logFile.print(",");

        logFile.print(powerData.currentA, 2);
        logFile.print(",");

        logFile.println(powerData.powerW, 2);
    }

    static uint32_t flushTimer = 0;

    if (millis() - flushTimer >= 1000)
    {
        flushTimer = millis();

        if (logFile)
            logFile.flush();
    }
  }
  delay(2);
}
