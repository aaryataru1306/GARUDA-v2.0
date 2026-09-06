#include <Arduino.h>
#include <SPI.h>
#include "mpu.h"
#include "icm.h"
#include "ekf.h"
#include "pid.h"

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

constexpr int MOTOR1_PIN = 22; // Front Right (CCW)
constexpr int MOTOR2_PIN = 4;  // Rear Right  (CW)
constexpr int MOTOR3_PIN = 3;  // Rear Left   (CCW)
constexpr int MOTOR4_PIN = 2;  // Front Left  (CW)

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

// ==================================================
// OBJECTS & GLOBALS
// ==================================================
PID_struct roll_pid;
PID_struct pitch_pid;
PID_struct yaw_pid;

SPISettings spiSettings(1000000, MSBFIRST, SPI_MODE0);
MPU9250 mpu(MPU_CS_PIN, spiSettings);
ICM20948 icm(ICM_CS_PIN, spiSettings);

EKF_IMU fusedEKF(0.005f, 0.6f);

unsigned long previousTime = 0;

// ==================================================
// HELPER FUNCTIONS
// ==================================================

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
  Serial.begin(115200);
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

  // Calibrate IMUs on boot up
  mpu.calibrate();
  icm.calibrate();

  // Initialize PID rate controllers with safe bench values (Ki = 0.0 to prevent windup)
  PID_init(&roll_pid, 0.2f, 0.002f, 0.005f, -15.0f, 15.0f);
  PID_init(&pitch_pid, 0.2f, 0.001f, 0.003f, -15.0f, 15.0f);
  PID_init(&yaw_pid, 0.0f, 0.0f, 0.00f, -10.0f, 10.0f);

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

// ==================================================
// LOOP
// ==================================================
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

  // 1. Read motion vectors from both sensors
  float mpu_ax, mpu_ay, mpu_az, mpu_gx, mpu_gy, mpu_gz;
  float icm_ax, icm_ay, icm_az, icm_gx, icm_gy, icm_gz;

  mpu.getMotion6(mpu_ax, mpu_ay, mpu_az, mpu_gx, mpu_gy, mpu_gz);
  icm.getMotion6(icm_ax, icm_ay, icm_az, icm_gx, icm_gy, icm_gz);

  // 2. Average rates (rad/s)
  float fused_gx_rad = 0.5f * (mpu_gx + icm_gx);
  float fused_gy_rad = 0.5f * (mpu_gy + icm_gy);
  float fused_gz_rad = 0.5f * (mpu_gz + icm_gz);

  // Convert angular rates to deg/s
  float fused_gx_deg = fused_gx_rad * (180.0f / M_PI);
  float fused_gy_deg = fused_gy_rad * (180.0f / M_PI);
  float fused_gz_deg = fused_gz_rad * (180.0f / M_PI);

  // 3. EKF Filter Update
  fusedEKF.predict(fused_gx_rad, fused_gy_rad, fused_gz_rad, dt);
  fusedEKF.update(mpu_ax, mpu_ay, mpu_az);
  fusedEKF.update(icm_ax, icm_ay, icm_az);

  // 4. Closed-loop control
  if (baseThrottlePercent > 0.0f) {
    float angle_kp = 4.0f;
    float desired_roll_rate  = angle_kp * (0.0f - fusedEKF.roll);
    float desired_pitch_rate  = angle_kp * (0.0f - fusedEKF.pitch);
    //float desired_yaw_rate  = angle_kp * (0.0f - fusedEKF.yaw);

    float roll_output  = PID_update(&roll_pid,  desired_roll_rate,  fused_gx_deg, dt);
    float pitch_output = PID_update(&pitch_pid, desired_pitch_rate, fused_gy_deg, dt);
    float yaw_output   = PID_update(&yaw_pid,   0.0f,   fused_gz_deg, dt);

    // Quad X Motor Mix
    m1_throttle = baseThrottlePercent + pitch_output - roll_output - yaw_output;
    m2_throttle = baseThrottlePercent - pitch_output - roll_output + yaw_output;
    m3_throttle = baseThrottlePercent - pitch_output + roll_output - yaw_output;
    m4_throttle = baseThrottlePercent + pitch_output + roll_output + yaw_output;
  
  } else {
    // Reset PID integral memory when idle
    roll_pid.integral = 0.0f;
    pitch_pid.integral = 0.0f;
    yaw_pid.integral = 0.0f;
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

  // 5. Output to ESCs
  writeMotorThrottle(MOTOR1_PIN, filtered_m1);
  writeMotorThrottle(MOTOR2_PIN, filtered_m2);
  writeMotorThrottle(MOTOR3_PIN, filtered_m3);
  writeMotorThrottle(MOTOR4_PIN, filtered_m4);

  // --- Arduino Serial Plotter Standard Output ---
  static uint32_t debugTimer = 0;

  if (millis() - debugTimer >= 20) // 50 Hz refresh rate for smooth plotting
  {
      debugTimer = millis();

      char plotBuffer[128];
      snprintf(plotBuffer, sizeof(plotBuffer),
               "Throttle:%.2f Roll:%.2f Pitch:%.2f M1:%.2f M2:%.2f M3:%.2f M4:%.2f",
               baseThrottlePercent,
               fusedEKF.roll,
               fusedEKF.pitch,
               filtered_m1,
               filtered_m2,
               filtered_m3,
               filtered_m4);

      Serial.println(plotBuffer);
  }
  delay(2);
}