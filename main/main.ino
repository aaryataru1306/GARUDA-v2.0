#include <Arduino.h>
#include <SPI.h>
#include "mpu.h"
#include "icm.h"
#include "ekf.h"
#include "pid.h"
#include "bmp581.h"
#include "pm02d.h"
#include "lora_radio.h"

// ==================================================
// PIN CONFIGURATION
// ==================================================
constexpr int MPU_CS_PIN = 10;
constexpr int ICM_CS_PIN = 9;

SPISettings spiSettings(1000000, MSBFIRST, SPI_MODE0);

MPU9250  mpu(MPU_CS_PIN, spiSettings);
ICM20948 icm(ICM_CS_PIN, spiSettings);

// EKF_IMU(gyro_noise, meas_noise_mpu, meas_noise_icm)
// R_mpu/R_icm are now the accelerometer measurement-noise variances
// (used by updateAccel()), not quaternion-measurement noise.
EKF_IMU fusedEKF(0.005f, 0.02f, 0.015f);

// ---- Power monitor (PM02D / INA228) ----
PM02D powerMonitor(Wire, PM02D_I2C_ADDR);
PM02D_Data powerData = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, false};

// ---- Barometer (BMP581) ----
constexpr float SEA_LEVEL_PRESSURE_PA = 101325.0f; // adjust to local QNH for accurate altitude
float baroTemperatureC = NAN;
float baroPressurePa   = NAN;
float baroAltitudeM    = NAN;

// ---- LoRa (SX1262, bit-bang SPI HAL) ----
BitBangHal customHal;
SX1262 radio = new Module(&customHal, PIN_CS, PIN_DIO1, PIN_RST, PIN_BUSY);
volatile bool dio1_triggered = false;
uint32_t lastTxTime = 0;

void dio1_isr() {
  dio1_triggered = true;
}

// atan2f() (used for both roll and pitch in ekf.h) already returns a signed
// value in (-180, 180], so tilting forward gives + degrees and tilting
// backward gives - degrees, with no wrap needed. The old version added 360
// to any negative angle, which silently converted every backward tilt into
// a large positive number instead of a negative one — that was the bug.
// This now only zeroes out tiny noise right around 0 degrees.
float applyDeadband(float val) {
  if (val > -0.8f && val < 0.8f) val = 0.0f;
  return val;
}

// ==================================================
// Axis-wise gyro selection.
// Instead of averaging MPU/ICM gyro (which lets a noisy/glitching sensor
// drag the fused rate off), compare each sensor's reading on this axis to
// a running "expected" reference (the rate we selected last cycle) and
// keep whichever sensor is closer. No arbitrary threshold: it's a pure
// nearest-to-reference comparison, done independently per axis, so a
// transient spike on one sensor's Z axis (say) doesn't affect X/Y.
// ==================================================
float selectGyroAxis(float mpuVal, float icmVal, float &reference) {
  float dMpu = fabsf(mpuVal - reference);
  float dIcm = fabsf(icmVal - reference);
  float selected = (dMpu <= dIcm) ? mpuVal : icmVal;
  reference = selected; // this becomes next cycle's "expected" rate
  return selected;
}

// Running per-axis gyro references used by selectGyroAxis()
static float refGX = 0.0f, refGY = 0.0f, refGZ = 0.0f;

unsigned long previousTime = 0;

// ==================================================
// PWM / ESC CONSTANTS
// ==================================================
constexpr int PWM_FREQUENCY   = 50;
constexpr int PWM_RESOLUTION  = 16;
constexpr int PWM_MAX_DUTY    = 65535;

constexpr float ESC_MIN_US    = 1000.0f;
constexpr float ESC_MAX_US    = 2000.0f;

constexpr float THROTTLE_ALPHA    = 0.15f;
constexpr float MAX_BASE_THROTTLE = 100.0f;
constexpr float MAX_MOTOR_LIMIT   = 100.0f;

// ==================================================
// MOTOR PINS
// ==================================================
constexpr int MOTOR1_PIN = 22;
constexpr int MOTOR2_PIN = 2;
constexpr int MOTOR3_PIN = 3;
constexpr int MOTOR4_PIN = 4;

// ==================================================
// PPM RECEIVER
// ==================================================
constexpr int RX_PIN = 28;

constexpr uint8_t PPM_MAX_CHANNELS = 10;
volatile uint32_t ppm_last_edge   = 0;
volatile uint8_t  ppm_channel     = 0;
volatile uint16_t ppm_channels[PPM_MAX_CHANNELS];
volatile uint32_t ppm_last_signal = 0;

// ==================================================
// MOTOR / THROTTLE VARIABLES
// ==================================================
float baseThrottlePercent = 0.0f;

float m1_throttle = 0.0f;
float m2_throttle = 0.0f;
float m3_throttle = 0.0f;
float m4_throttle = 0.0f;

float filtered_m1 = 0.0f;
float filtered_m2 = 0.0f;
float filtered_m3 = 0.0f;
float filtered_m4 = 0.0f;

// ==================================================
// PID CONTROLLERS (pid.h/pid.cpp — driver, unchanged)
// ==================================================
PID_struct roll_pid;
PID_struct pitch_pid;
PID_struct yaw_pid;

// ==================================================
// PPM INTERRUPT
// ==================================================
void ppm_isr() {
  uint32_t now = micros();
  uint32_t dt  = now - ppm_last_edge;
  ppm_last_edge = now;

  if (dt > 3000) {
    ppm_channel = 0;
  } else if (dt >= 900 && dt <= 2200) {
    if (ppm_channel < PPM_MAX_CHANNELS) {
      ppm_channels[ppm_channel] = (uint16_t)dt;
      ppm_channel++;
    }
    ppm_last_signal = now;
  }
}

// ==================================================
// READ THROTTLE FROM PPM CH3
// ==================================================
float read_throttle() {
  uint16_t throttlePulse;
  uint32_t lastSignal;

  noInterrupts();
  throttlePulse = ppm_channels[2];
  lastSignal    = ppm_last_signal;
  interrupts();

  if ((micros() - lastSignal) > 100000 ||
      throttlePulse < 900 ||
      throttlePulse > 2100) {
    return 0.0f;
  }

  float throttle = (throttlePulse - 1000.0f) / 10.0f;
  return constrain(throttle, 0.0f, 100.0f);
}

// ==================================================
// CONVERT MICROSECONDS TO PWM DUTY
// ==================================================
uint32_t microsecondsToDuty(float pulseUs) {
  float periodUs = 1000000.0f / (float)PWM_FREQUENCY;
  pulseUs = constrain(pulseUs, ESC_MIN_US, ESC_MAX_US);
  return (uint32_t)((pulseUs / periodUs) * (float)PWM_MAX_DUTY);
}

// ==================================================
// WRITE RAW ESC PULSE
// ==================================================
void writeRawPulseUs(int pin, float pulseUs) {
  analogWrite(pin, microsecondsToDuty(pulseUs));
}

// ==================================================
// CONVERT MOTOR THROTTLE % TO ESC PWM
// ==================================================
void writeMotorThrottle(int pin, float percent) {
  percent = constrain(percent, 0.0f, MAX_MOTOR_LIMIT);
  float pulseUs = ESC_MIN_US + (percent / 100.0f) * (ESC_MAX_US - ESC_MIN_US);
  writeRawPulseUs(pin, pulseUs);
}

// ==================================================
// WRITE SAME PWM TO ALL MOTORS
// ==================================================
void writeAllMotorsRaw(float pulseUs) {
  writeRawPulseUs(MOTOR1_PIN, pulseUs);
  writeRawPulseUs(MOTOR2_PIN, pulseUs);
  writeRawPulseUs(MOTOR3_PIN, pulseUs);
  writeRawPulseUs(MOTOR4_PIN, pulseUs);
}

// ==================================================
// THROTTLE SMOOTHING
// ==================================================
float smoothThrottle(float target, float current, float alpha) {
  return current + alpha * (target - current);
}

void setup() {
  Serial.begin(115200);
  while (!Serial && millis() < 3000);

  SPI.begin();

  if (!mpu.begin()) {
    Serial.println("MPU9250 init FAILED");
    while (1);
  }

  uint8_t icmWho = 0;
  if (!icm.begin(icmWho)) {
    Serial.print("ICM20948 init FAILED (WHO_AM_I=0x");
    Serial.print(icmWho, HEX);
    Serial.println(")");
    while (1);
  }

  Serial.println("Calibrating MPU + ICM...");
  mpu.calibrate();
  icm.calibrate();
  Serial.println("Calibration done.");

  // ---- I2C bus + BMP581 + PM02D/INA228 ----
  Wire.begin();
  Wire.setClock(400000);

  if (!BMP581_Begin()) {
    Serial.println("BMP581 init FAILED");
  } else {
    Serial.println("BMP581 init OK");
  }

  if (!powerMonitor.begin()) {
    Serial.println("PM02D / INA228 NOT FOUND");
  } else {
    Serial.println("PM02D / INA228 detected.");
  }

  // ---- SX1262 LoRa (bit-bang SPI HAL) ----
  Serial.println("Initializing SX1262 with Custom Soft-SPI HAL (Teensy)...");
  int loraState = radio.begin(867.5, 125.0, 7, 5, 0x12, 22, 8, 3.3);
  if (loraState != RADIOLIB_ERR_NONE) {
    Serial.print("SX1262 init FAILED! Error code: ");
    Serial.println(loraState);
    while (1);
  }
  radio.setDio2AsRfSwitch(true);
  radio.setPacketSentAction(dio1_isr);
  Serial.println("SX1262 Initialization Complete.");
  lastTxTime = millis();

  // ---- PWM / ESC setup (not in the pasted snippet, but required for
  // analogWrite() to actually behave as a 50Hz/16-bit ESC signal) ----
  analogWriteResolution(PWM_RESOLUTION);
  analogWriteFrequency(MOTOR1_PIN, PWM_FREQUENCY);
  analogWriteFrequency(MOTOR2_PIN, PWM_FREQUENCY);
  analogWriteFrequency(MOTOR3_PIN, PWM_FREQUENCY);
  analogWriteFrequency(MOTOR4_PIN, PWM_FREQUENCY);
  writeAllMotorsRaw(ESC_MIN_US); // safe disarmed pulse immediately on boot

  // ---- PPM receiver ----
  pinMode(RX_PIN, INPUT);
  attachInterrupt(digitalPinToInterrupt(RX_PIN), ppm_isr, RISING);

  // ---- PID init ----
  PID_init(&roll_pid,  0.2f, 0.002f, 0.06f, -15.0f, 15.0f);
  PID_init(&pitch_pid, 0.2f, 0.001f, 0.03f, -15.0f, 15.0f);
  PID_init(&yaw_pid,   0.3f, 0.001f, 0.06f, -10.0f, 10.0f);

  // ---- ESC arming hold: not in the pasted snippet, but required so ESCs
  // see a steady disarm pulse (their init chime) before flight code runs ----
  uint32_t armStart = millis();
  while (millis() - armStart < 3000) {
    writeAllMotorsRaw(ESC_MIN_US);
    delay(20);
  }

  Serial.println("Fused_Roll,Fused_Pitch,Fused_Yaw,Temp_C,Pressure_Pa,Altitude_m,Voltage_V,Current_A,Power_W");
  previousTime = micros();
}

void loop() {
  unsigned long currentTime = micros();
  float dt = (currentTime - previousTime) / 1000000.0f;
  previousTime = currentTime;
  if (dt <= 0.0f || dt > 0.2f) dt = 0.005f; // guard against startup/overflow glitches

  // ---- Calibrated raw sensor reads (driver-handled calibration/scaling; unchanged) ----
  float mpu_ax, mpu_ay, mpu_az, mpu_gx, mpu_gy, mpu_gz;
  mpu.getMotion6(mpu_ax, mpu_ay, mpu_az, mpu_gx, mpu_gy, mpu_gz);

  float icm_ax, icm_ay, icm_az, icm_gx, icm_gy, icm_gz;
  icm.getMotion6(icm_ax, icm_ay, icm_az, icm_gx, icm_gy, icm_gz);

  // ---- Gyro fusion: per-axis "closer to expected" selection instead of averaging ----
  float gx = selectGyroAxis(mpu_gx, icm_gx, refGX);
  float gy = selectGyroAxis(mpu_gy, icm_gy, refGY);
  float gz = selectGyroAxis(mpu_gz, icm_gz, refGZ);

  // ---- EKF PREDICT: propagate attitude using the selected gyro rate ----
  fusedEKF.predict(gx, gy, gz, dt);

  // ---- EKF UPDATE: correct using raw calibrated accel directly (no Madgwick) ----
  fusedEKF.updateAccel(mpu_ax, mpu_ay, mpu_az, fusedEKF.R_mpu);
  fusedEKF.updateAccel(icm_ax, icm_ay, icm_az, fusedEKF.R_icm);

  float fused_roll  = applyDeadband(fusedEKF.roll);
  float fused_pitch = applyDeadband(fusedEKF.pitch);
  float fused_yaw   = applyDeadband(fusedEKF.yaw);

  // ==================================================
  // BMP581 + PM02D — read at 10Hz, not every 2ms loop tick
  // (BMP581_ReadRaw() is called once and shared between temp/pressure/
  // altitude instead of calling BMP581_ReadTemperature()/ReadPressure()
  // separately, which would each trigger their own I2C transaction)
  // ==================================================
  static uint32_t envTimer = 0;
  if (millis() - envTimer >= 100) {
    envTimer = millis();

    if (BMP581_ReadRaw()) {
      baroTemperatureC = BMP581_CompensateTemperature(bmp581_raw_temperature);
      baroPressurePa   = BMP581_CompensatePressure(bmp581_raw_pressure);
      baroAltitudeM     = 44330.0f * (1.0f - powf(baroPressurePa / SEA_LEVEL_PRESSURE_PA, 0.1903f));
    }

    powerMonitor.read(powerData);
  }

  // ==================================================
  // PPM -> PID -> MOTOR MIXING
  // ==================================================
  baseThrottlePercent = read_throttle();

  if (baseThrottlePercent > 0.0f) {
    float roll_output  = PID_update(&roll_pid,  0.0f, fused_roll,  dt);
    float pitch_output = PID_update(&pitch_pid, 0.0f, fused_pitch, dt);
    float yaw_output   = PID_update(&yaw_pid,   0.0f, fused_yaw,   dt);

    // Quad-X Motor Mixing Matrix
    m1_throttle = baseThrottlePercent + pitch_output - roll_output - yaw_output;
    m2_throttle = baseThrottlePercent - pitch_output - roll_output + yaw_output;
    m3_throttle = baseThrottlePercent - pitch_output + roll_output - yaw_output;
    m4_throttle = baseThrottlePercent + pitch_output + roll_output + yaw_output;
  } else {
    roll_pid.integral  = 0.0f;
    pitch_pid.integral = 0.0f;
    yaw_pid.integral   = 0.0f;

    m1_throttle = m2_throttle = m3_throttle = m4_throttle = 0.0f;
  }

  // ==================================================
  // MOTOR OUTPUT SMOOTHING
  // ==================================================
  if (baseThrottlePercent <= 0.0f) {
    filtered_m1 = filtered_m2 = filtered_m3 = filtered_m4 = 0.0f;
  } else {
    filtered_m1 = smoothThrottle(m1_throttle, filtered_m1, THROTTLE_ALPHA);
    filtered_m2 = smoothThrottle(m2_throttle, filtered_m2, THROTTLE_ALPHA);
    filtered_m3 = smoothThrottle(m3_throttle, filtered_m3, THROTTLE_ALPHA);
    filtered_m4 = smoothThrottle(m4_throttle, filtered_m4, THROTTLE_ALPHA);
  }

  // ==================================================
  // FINAL PWM OUTPUT
  // ==================================================
  writeMotorThrottle(MOTOR1_PIN, filtered_m1);
  writeMotorThrottle(MOTOR2_PIN, filtered_m2);
  writeMotorThrottle(MOTOR3_PIN, filtered_m3);
  writeMotorThrottle(MOTOR4_PIN, filtered_m4);

  Serial.print(fused_roll, 2);
  Serial.print(",");
  Serial.print(fused_pitch, 2);
  Serial.print(",");
  Serial.print(fused_yaw, 2);
  Serial.print(",");
  Serial.print(baroTemperatureC, 2);
  Serial.print(",");
  Serial.print(baroPressurePa, 1);
  Serial.print(",");
  Serial.print(baroAltitudeM, 2);
  Serial.print(",");
  Serial.print(powerData.voltageV, 2);
  Serial.print(",");
  Serial.print(powerData.currentA, 2);
  Serial.print(",");
  Serial.println(powerData.powerW, 2);

  // ==================================================
  // SX1262 LoRa — non-blocking TX-complete check + periodic telemetry send
  // Payload is the same fields as the Serial CSV line above:
  // roll,pitch,yaw,temp_C,pressure_Pa,altitude_m,voltage_V,current_A,power_W
  // ==================================================
  if (dio1_triggered) {
    dio1_triggered = false;
    radio.finishTransmit();
    Serial.println("--> TX Complete! (DIO1 Interrupt Cleared)");
  }

  if (millis() - lastTxTime > 200) {
    char telemetryBuf[96];
    snprintf(telemetryBuf, sizeof(telemetryBuf),
             "%.2f,%.2f,%.2f,%.2f,%.1f,%.2f,%.2f,%.2f,%.2f",
             fused_roll, fused_pitch, fused_yaw,
             baroTemperatureC, baroPressurePa, baroAltitudeM,
             powerData.voltageV, powerData.currentA, powerData.powerW);

    String msg = telemetryBuf;
    Serial.println("Transmitting: " + msg);
    radio.startTransmit(msg);
    lastTxTime = millis();
  }

  delay(2);
}