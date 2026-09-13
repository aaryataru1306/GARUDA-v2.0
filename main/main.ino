#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>
#include <SD.h>
#include <RadioLib.h> // No SPI.h needed for the LoRa link - it uses bit-banged soft SPI below

#include "mpu.h"
#include "icm.h"
#include "ekf.h"
#include "pid.h"
#include "pm02d.h"
#include "bmp581.h"
#include "l89h_gps.h"

// ==================================================
// LoRa (SX1262) bit-bang SPI pin assignment
// Verified against motor pins (2,3,4,22), IMU CS pins (9,10), and the
// PPM RX pin (28) - no conflicts.
// ==================================================

// ==================================================
// TEENSY 4.1 PIN CONFIGURATION & CONSTANTS
// ==================================================
constexpr int MPU_CS_PIN = 10;
constexpr int ICM_CS_PIN = 9;
constexpr int RX_PIN     = 28;
#define SD_CS BUILTIN_SDCARD

constexpr int MOTOR1_PIN = 22; // Front Right (CCW)
constexpr int MOTOR2_PIN = 4;  // Rear Right  (CW)
constexpr int MOTOR3_PIN = 3;  // Rear Left   (CCW)
constexpr int MOTOR4_PIN = 2;  // Front Left  (CW)

// --- LoRa (SX1262) bit-bang SPI pins ---
const int LORA_PIN_MOSI = 26;
const int LORA_PIN_MISO = 39;
const int LORA_PIN_SCK  = 27;
const int LORA_PIN_CS   = 8;
const int LORA_PIN_BUSY = 7;
const int LORA_PIN_RST  = 5;
const int LORA_PIN_DIO1 = 6;

// Barometer reference for altitude calc (Pa). Update to local QNH before flight.
constexpr float SEA_LEVEL_PRESSURE_PA = 101325.0f;

// Telemetry TX interval (ms) - matches the original LoRa test cadence
constexpr uint32_t TELEMETRY_INTERVAL_MS = 200;

constexpr int PWM_FREQUENCY   = 50;    // 50 Hz standard ESC update rate
constexpr int PWM_RESOLUTION  = 16;    // 16-bit PWM resolution (0 - 65535)
constexpr int PWM_MAX_DUTY    = 65535;

constexpr float ESC_MIN_US    = 1000.0f; // 0% Throttle (Arm / Disarm Pulse)
constexpr float ESC_MAX_US    = 2000.0f; // 100% Full Throttle

constexpr float THROTTLE_ALPHA    = 0.15f;
constexpr float MAX_BASE_THROTTLE = 100.0f;
constexpr float MAX_MOTOR_LIMIT   = 100.0f;

// Madgwick Algorithm Tuning Gain (used for BOTH per-sensor filters)
constexpr float MADGWICK_BETA = 0.1f;

// EKF measurement-noise tuning: how much the EKF trusts each
// sensor's Madgwick quaternion. Lower = trusted more.
constexpr float EKF_GYRO_PROCESS_NOISE = 0.005f;
constexpr float EKF_R_MPU = 0.05f;
constexpr float EKF_R_ICM = 0.05f;

// PPM Receiver Setup
constexpr uint8_t PPM_MAX_CHANNELS = 10;
volatile uint32_t ppm_last_edge   = 0;
volatile uint8_t  ppm_channel     = 0;
volatile uint16_t ppm_channels[PPM_MAX_CHANNELS];
volatile uint32_t ppm_last_signal = 0;

// State Variables
float baseThrottlePercent = 0.0f;
float m1_throttle = 0.0f, m2_throttle = 0.0f, m3_throttle = 0.0f, m4_throttle = 0.0f;
float filtered_m1 = 0.0f, filtered_m2 = 0.0f, filtered_m3 = 0.0f, filtered_m4 = 0.0f;

// --------------------------------------------------
// Per-sensor Madgwick quaternion states.
// Each sensor now runs its OWN Madgwick filter on its OWN raw data:
//   MPU raw  -> madgwickUpdate6DOF -> mpu_q  (q_MPU)
//   ICM raw  -> madgwickUpdate6DOF -> icm_q  (q_ICM)
// These two quaternions are what get fed into the EKF below.
// --------------------------------------------------
float mpu_q[4] = {1.0f, 0.0f, 0.0f, 0.0f};
float icm_q[4] = {1.0f, 0.0f, 0.0f, 0.0f};

// Optional: per-sensor Euler angles, purely for logging/comparison.
float mpu_roll = 0.0f, mpu_pitch = 0.0f, mpu_yaw = 0.0f;
float icm_roll = 0.0f, icm_pitch = 0.0f, icm_yaw = 0.0f;

// Which IMU's quaternion the EKF actually used last cycle ('M' or 'I') -
// kept purely for telemetry so you can see how often each sensor "wins".
char lastChosenIMU = 'M';

File logFile;
uint32_t logStartTime = 0;
unsigned long previousTime = 0;

PID_struct roll_pid;
PID_struct pitch_pid;
PID_struct yaw_pid;

SPISettings spiSettings(1000000, MSBFIRST, SPI_MODE0);
MPU9250 mpu(MPU_CS_PIN, spiSettings);
ICM20948 icm(ICM_CS_PIN, spiSettings);

PM02D powerMonitor(Wire, PM02D_I2C_ADDR);
PM02D_Data powerData = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, false};

// EKF now fuses the two Madgwick QUATERNIONS (not raw accel).
// Constructor args: gyro process noise, R for q_MPU, R for q_ICM.
EKF_IMU fusedEKF(EKF_GYRO_PROCESS_NOISE, EKF_R_MPU, EKF_R_ICM);

// GPS (kept as-provided)
L89H_GPS gps;

// Barometer telemetry (BMP581 driver is a flat C API, no class instance needed)
float baro_altitude_m    = 0.0f;
float baro_temperature_c = 0.0f;

// ==================================================
// LoRa (SX1262) - CUSTOM BIT-BANG HAL, kept as provided
// ==================================================
class BitBangHal : public RadioLibHal {
  public:
    BitBangHal() : RadioLibHal(INPUT, OUTPUT, LOW, HIGH, RISING, FALLING) {}

    void init() override { spiBegin(); }
    void term() override { spiEnd(); }
    void pinMode(uint32_t pin, uint32_t mode) override { ::pinMode(pin, mode); }
    void digitalWrite(uint32_t pin, uint32_t value) override { ::digitalWriteFast(pin, value); }
    uint32_t digitalRead(uint32_t pin) override { return ::digitalReadFast(pin); }
    void attachInterrupt(uint32_t interruptNum, void (*interruptCb)(void), uint32_t mode) override { ::attachInterrupt(interruptNum, interruptCb, mode); }
    void detachInterrupt(uint32_t interruptNum) override { ::detachInterrupt(interruptNum); }
    void delay(unsigned long ms) override { ::delay(ms); }
    void delayMicroseconds(unsigned long us) override { ::delayMicroseconds(us); }
    unsigned long millis() override { return ::millis(); }
    unsigned long micros() override { return ::micros(); }
    void tone(uint32_t pin, unsigned int frequency, unsigned long duration = 0) override { ::tone(pin, frequency, duration); }
    void noTone(uint32_t pin) override { ::noTone(pin); }
    void yield() override { ::yield(); }

    long pulseIn(uint32_t pin, uint32_t state, uint32_t timeout) override {
        return ::pulseIn(pin, state, timeout);
    }

    // --- Soft SPI Implementation ---
    void spiBegin() override {
        ::pinMode(LORA_PIN_MOSI, OUTPUT);
        ::pinMode(LORA_PIN_SCK, OUTPUT);
        ::pinMode(LORA_PIN_MISO, INPUT_PULLUP);
        ::digitalWriteFast(LORA_PIN_SCK, LOW);
    }

    void spiBeginTransaction() override {}
    void spiEndTransaction() override {}
    void spiEnd() override {}

    void spiTransfer(uint8_t* out, size_t len, uint8_t* in) override {
        for (size_t i = 0; i < len; i++) {
            uint8_t receivedByte = transferByte(out ? out[i] : 0x00);
            if (in) {
                in[i] = receivedByte;
            }
        }
    }

  private:
    uint8_t transferByte(uint8_t data) {
        uint8_t readByte = 0;
        for (int i = 7; i >= 0; i--) {
            ::digitalWriteFast(LORA_PIN_MOSI, (data >> i) & 0x01);
            ::delayMicroseconds(2);
            ::digitalWriteFast(LORA_PIN_SCK, HIGH);
            ::delayMicroseconds(2);
            readByte |= (::digitalReadFast(LORA_PIN_MISO) << i);
            ::digitalWriteFast(LORA_PIN_SCK, LOW);
            ::delayMicroseconds(2);
        }
        return readByte;
    }
};

BitBangHal loraHal;
SX1262 radio = new Module(&loraHal, LORA_PIN_CS, LORA_PIN_DIO1, LORA_PIN_RST, LORA_PIN_BUSY);

volatile bool lora_dio1_triggered = false;
bool loraTxInProgress = false; // small addition: avoid starting a new TX before the previous one signals done
uint32_t lastTelemetryTxTime = 0;

void lora_dio1_isr() {
    lora_dio1_triggered = true;
}

float invSqrt(float x) {
    return 1.0f / sqrtf(x);
}

// Selects the gyro reading closer to the previous frame's fused value.
// Still used to build one "best" gyro rate to drive the EKF's own
// predict() step (the EKF needs *some* angular rate to propagate its
// state forward between quaternion measurements).
float chooseGyro(float mpu_val, float icm_val, float prev_val) {
    float diff_mpu = fabsf(mpu_val - prev_val);
    float diff_icm = fabsf(icm_val - prev_val);
    return (diff_mpu < diff_icm) ? mpu_val : icm_val;
}

void madgwickUpdate6DOF(float gx, float gy, float gz, float ax, float ay, float az, float dt, float &q1, float &q2, float &q3, float &q4) {
    float recipNorm, s1, s2, s3, s4, qDot1, qDot2, qDot3, qDot4;
    float _2q1, _2q2, _2q3, _2q4, _4q1, _4q2, _4q3, _4q4, _8q2, _8q3, q1q1, q2q2, q3q3, q4q4;

    qDot1 = 0.5f * (-q2 * gx - q3 * gy - q4 * gz);
    qDot2 = 0.5f * (q1 * gx + q3 * gz - q4 * gy);
    qDot3 = 0.5f * (q1 * gy - q2 * gz + q4 * gx);
    qDot4 = 0.5f * (q1 * gz + q2 * gy - q3 * gx);

    if (!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {
        recipNorm = invSqrt(ax * ax + ay * ay + az * az);
        ax *= recipNorm; ay *= recipNorm; az *= recipNorm;

        _2q1 = 2.0f * q1; _2q2 = 2.0f * q2; _2q3 = 2.0f * q3; _2q4 = 2.0f * q4;
        _4q1 = 4.0f * q1; _4q2 = 4.0f * q2; _4q3 = 4.0f * q3; _4q4 = 4.0f * q4;
        _8q2 = 8.0f * q2; _8q3 = 8.0f * q3;
        q1q1 = q1 * q1;   q2q2 = q2 * q2;   q3q3 = q3 * q3;   q4q4 = q4 * q4;

        s1 = _4q1 * q3q3 + _2q3 * ax + _4q1 * q2q2 - _2q2 * ay;
        s2 = _4q2 * q4q4 - _2q4 * ax + _4q2 * q1q1 - _2q1 * ay - _4q2 + _8q2 * q2q2 + _8q3 * q3q3 + _4q2 * az;
        s3 = _4q3 * q1q1 + _2q1 * ax + _4q3 * q4q4 - _2q4 * ay - _4q3 + _8q2 * q2q2 + _8q3 * q3q3 + _4q3 * az;
        s4 = _4q4 * q2q2 - _2q2 * ax + _4q4 * q3q3 - _2q3 * ay;

        recipNorm = invSqrt(s1 * s1 + s2 * s2 + s3 * s3 + s4 * s4);
        s1 *= recipNorm; s2 *= recipNorm; s3 *= recipNorm; s4 *= recipNorm;

        qDot1 -= MADGWICK_BETA * s1;
        qDot2 -= MADGWICK_BETA * s2;
        qDot3 -= MADGWICK_BETA * s3;
        qDot4 -= MADGWICK_BETA * s4;
    }

    q1 += qDot1 * dt; q2 += qDot2 * dt; q3 += qDot3 * dt; q4 += qDot4 * dt;
    recipNorm = invSqrt(q1 * q1 + q2 * q2 + q3 * q3 + q4 * q4);
    q1 *= recipNorm; q2 *= recipNorm; q3 *= recipNorm; q4 *= recipNorm;
}

void convertQuaternionToEuler(float q1, float q2, float q3, float q4, float &r, float &p, float &y) {
    float roll_rad  = atan2f(2.0f * (q1 * q2 + q3 * q4), 1.0f - 2.0f * (q2 * q2 + q3 * q3));
    float pitch_rad = asinf(2.0f * (q1 * q3 - q4 * q2));
    float yaw_rad   = atan2f(2.0f * (q1 * q4 + q2 * q3), 1.0f - 2.0f * (q3 * q3 + q4 * q4));

    r = roll_rad  * (180.0f / M_PI);
    p = pitch_rad * (180.0f / M_PI);
    y = yaw_rad   * (180.0f / M_PI);
}

// ==================================================
// INTERRUPTS & ACTUATION
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

float read_throttle() {
  uint16_t throttlePulse;
  uint32_t lastSignal;

  noInterrupts();
  throttlePulse = ppm_channels[2]; // CH3 = Throttle
  lastSignal    = ppm_last_signal;
  interrupts();

  if ((micros() - lastSignal) > 100000 || throttlePulse < 900 || throttlePulse > 2100) {
    return 0.0f;
  }

  float throttle = (throttlePulse - 1000.0f) / 10.0f;
  return constrain(throttle, 0.0f, 100.0f);
}

uint32_t microsecondsToDuty(float pulseUs) {
  float periodUs = 1000000.0f / (float)PWM_FREQUENCY;
  pulseUs = constrain(pulseUs, ESC_MIN_US, ESC_MAX_US);
  return (uint32_t)((pulseUs / periodUs) * (float)PWM_MAX_DUTY);
}

void writeRawPulseUs(int pin, float pulseUs) {
  analogWrite(pin, microsecondsToDuty(pulseUs));
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

float smoothThrottle(float target, float current, float alpha) {
  return current + alpha * (target - current);
}

// ==================================================
// SETUP
// ==================================================
void setup() {
  Wire.begin();
  Wire.setClock(400000);
  Serial.begin(115200);

  // SD Card Initialization
  if (!SD.begin(BUILTIN_SDCARD)) {
    Serial.println("SD CARD INIT FAILED!");
  } else {
    Serial.println("SD CARD INITIALIZED.");
    logFile = SD.open("flight.csv", FILE_WRITE);
    if (logFile) {
      if (logFile.size() == 0) {
        logFile.println("Time_ms,MPU_Roll,ICM_Roll,EKF_Roll,MPU_Pitch,ICM_Pitch,EKF_Pitch,Chosen_IMU,Altitude_m,Lat,Lon,Sats,Throttle,M1,M2,M3,M4,Voltage,Current,Power");
        logFile.flush();
      }
      logStartTime = millis();
      Serial.println("LOGGING STARTED.");
    } else {
      Serial.println("FAILED TO OPEN flight.csv");
    }
  }

  // Pin & Interrupt Config
  pinMode(RX_PIN, INPUT);
  attachInterrupt(digitalPinToInterrupt(RX_PIN), ppm_isr, RISING);

  analogWriteResolution(PWM_RESOLUTION);
  analogWriteFrequency(MOTOR1_PIN, PWM_FREQUENCY);
  analogWriteFrequency(MOTOR2_PIN, PWM_FREQUENCY);
  analogWriteFrequency(MOTOR3_PIN, PWM_FREQUENCY);
  analogWriteFrequency(MOTOR4_PIN, PWM_FREQUENCY);

  writeAllMotorsRaw(ESC_MIN_US);
  SPI.begin();

  // Hardware Initialization
  if (!mpu.begin()) {
    Serial.println("MPU9250 Init Failed!");
    while (1);
  }

  uint8_t icmWho = 0;
  if (!icm.begin(icmWho)) {
    Serial.println("ICM20948 Init Failed!");
    while (1);
  }

  if (!powerMonitor.begin()) {
    Serial.println("PM02D / INA228 NOT FOUND!");
  } else {
    Serial.println("PM02D / INA228 detected.");
  }

  mpu.calibrate();
  icm.calibrate();

  // Barometer Initialization
  if (!BMP581_Begin()) {
    Serial.println("BMP581 Init Failed!");
  } else {
    Serial.println("BMP581 Initialized.");
  }

  // GPS Initialization
  gps.begin();
  Serial.println("GPS Serial Started.");

  // LoRa (SX1262) Initialization
  // Freq: 867.5 MHz, BW: 125.0 kHz, SF: 7, CR: 4/5, SyncWord: 0x12, Pwr: 22 dBm
  {
    int state = radio.begin(867.5, 125.0, 7, 5, 0x12, 22, 8, 3.3);
    if (state != RADIOLIB_ERR_NONE) {
      Serial.print("LoRa Init Failed! Error code: ");
      Serial.println(state);
    } else {
      radio.setDio2AsRfSwitch(true);
      radio.setPacketSentAction(lora_dio1_isr);
      Serial.println("LoRa Initialized.");
    }
  }

  // Initialize PID Controllers
  PID_init(&roll_pid, 0.2f, 0.002f, 0.06f, -15.0f, 15.0f);
  PID_init(&pitch_pid, 0.2f, 0.001f, 0.03f, -15.0f, 15.0f);
  PID_init(&yaw_pid, 0.3f, 0.001f, 0.06f, -10.0f, 10.0f);

  // ESC Arming Delay
  uint32_t armStart = millis();
  while (millis() - armStart < 3000) {
    writeAllMotorsRaw(ESC_MIN_US);
    delay(20);
  }

  previousTime = micros();
}

// ==================================================
// MAIN EXECUTION LOOP
// ==================================================
void loop() {
  // 1. Process Receiver & Serial Commands
  baseThrottlePercent = read_throttle();

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

  // 2. Loop Timing Calculation
  unsigned long currentTime = micros();
  float dt = (currentTime - previousTime) / 1000000.0f;
  previousTime = currentTime;
  if (dt <= 0.0f || dt > 0.2f) dt = 0.01f;

  // 3. Read Dual Motion Data (each sensor's own raw readings)
  float mpu_ax, mpu_ay, mpu_az, mpu_gx, mpu_gy, mpu_gz;
  float icm_ax, icm_ay, icm_az, icm_gx, icm_gy, icm_gz;

  mpu.getMotion6(mpu_ax, mpu_ay, mpu_az, mpu_gx, mpu_gy, mpu_gz);
  icm.getMotion6(icm_ax, icm_ay, icm_az, icm_gx, icm_gy, icm_gz);

  // Orientation correction: ICM20948 is mounted with its Y axis flipped
  // relative to the MPU9250 (X axes agree). Negate both the accel and
  // gyro Y readings so both sensors report in a common frame. TODO:
  // verify Z axis agreement too - if the ICM is rotated 180 deg about X
  // rather than mirrored, Z will also need negating (icm_az, icm_gz).
  icm_ay = -icm_ay;
  icm_gy = -icm_gy;

  // --------------------------------------------------------------
  // 4. STAGE 1: independent Madgwick filter per sensor
  //    MPU raw  -> Madgwick -> q_MPU (mpu_q)
  //    ICM raw  -> Madgwick -> q_ICM (icm_q)
  // Each filter only ever sees its own sensor's data - no cross-mixing
  // happens here anymore. That mixing now happens in the EKF (step 6).
  // --------------------------------------------------------------
  madgwickUpdate6DOF(mpu_gx, mpu_gy, mpu_gz, mpu_ax, mpu_ay, mpu_az, dt,
                      mpu_q[0], mpu_q[1], mpu_q[2], mpu_q[3]);

  madgwickUpdate6DOF(icm_gx, icm_gy, icm_gz, icm_ax, icm_ay, icm_az, dt,
                      icm_q[0], icm_q[1], icm_q[2], icm_q[3]);

  // Per-sensor Euler angles, kept only for comparison/logging.
  convertQuaternionToEuler(mpu_q[0], mpu_q[1], mpu_q[2], mpu_q[3], mpu_roll, mpu_pitch, mpu_yaw);
  convertQuaternionToEuler(icm_q[0], icm_q[1], icm_q[2], icm_q[3], icm_roll, icm_pitch, icm_yaw);

  // --------------------------------------------------------------
  // 5. Gyro rate to drive the EKF's own predict() step.
  // The EKF still needs an angular rate to propagate its quaternion
  // state forward between the two measurement updates below - the
  // dynamic gyro-continuity selection is reused here.
  // --------------------------------------------------------------
  static float previous_gx = 0.0f;
  static float previous_gy = 0.0f;
  static float previous_gz = 0.0f;

  float fused_gx_rad = chooseGyro(mpu_gx, icm_gx, previous_gx);
  float fused_gy_rad = chooseGyro(mpu_gy, icm_gy, previous_gy);
  float fused_gz_rad = chooseGyro(mpu_gz, icm_gz, previous_gz);

  previous_gx = fused_gx_rad;
  previous_gy = fused_gy_rad;
  previous_gz = fused_gz_rad;

  // --------------------------------------------------------------
  // 6. STAGE 2: EKF picks WHICH IMU's quaternion to trust this cycle.
  //    predict() advances the state using gyro (as usual for an EKF).
  //    Then, instead of blending both q_MPU and q_ICM every step, we
  //    measure how far each one is from the just-predicted state
  //    (residualMagnitude: 0 = identical orientation, larger = more
  //    disagreement) and only call update() with whichever one is
  //    closer - i.e. more consistent with where the filter expected
  //    the attitude to be. The other sensor's reading is discarded
  //    for this cycle.
  // --------------------------------------------------------------
  fusedEKF.predict(fused_gx_rad, fused_gy_rad, fused_gz_rad, dt);

  float mpu_residual = fusedEKF.residualMagnitude(mpu_q);
  float icm_residual = fusedEKF.residualMagnitude(icm_q);

  if (mpu_residual <= icm_residual) {
    fusedEKF.update(mpu_q, fusedEKF.R_mpu);
    lastChosenIMU = 'M';
  } else {
    fusedEKF.update(icm_q, fusedEKF.R_icm);
    lastChosenIMU = 'I';
  }

  // 7. Update Power Telemetry (10 Hz)
  static uint32_t powerTimer = 0;
  if (millis() - powerTimer >= 100) {
    powerTimer = millis();
    powerMonitor.read(powerData);
  }

  // 7b. GPS: drain the serial buffer every loop (non-blocking, cheap)
  gps.read();

  // 7c. Barometer telemetry (10 Hz - I2C reads, don't need to run at 500 Hz)
  static uint32_t baroTimer = 0;
  if (millis() - baroTimer >= 100) {
    baroTimer = millis();
    baro_altitude_m    = BMP581_ReadAltitude(SEA_LEVEL_PRESSURE_PA);
    baro_temperature_c = BMP581_ReadTemperature();
  }

  // 7d. LoRa telemetry downlink (non-blocking, ~5 Hz)
  if (lora_dio1_triggered) {
    lora_dio1_triggered = false;
    radio.finishTransmit(); // clears IRQs, returns radio to standby
    loraTxInProgress = false;
  }

  if (!loraTxInProgress && (millis() - lastTelemetryTxTime >= TELEMETRY_INTERVAL_MS)) {
    lastTelemetryTxTime = millis();

    char telemBuffer[160];
    snprintf(telemBuffer, sizeof(telemBuffer),
             "R:%.1f,P:%.1f,Y:%.1f,IMU:%c,ALT:%.1f,LAT:%.6f,LON:%.6f,SAT:%lu,THR:%.0f,V:%.2f,I:%.2f",
             fusedEKF.roll, fusedEKF.pitch, fusedEKF.yaw, lastChosenIMU,
             baro_altitude_m, gps.getLatitude(), gps.getLongitude(),
             (unsigned long)gps.getSatellites(), baseThrottlePercent,
             powerData.voltageV, powerData.currentA);

    int txState = radio.startTransmit((uint8_t*)telemBuffer, strlen(telemBuffer));
    if (txState == RADIOLIB_ERR_NONE) {
      loraTxInProgress = true;
    }
  }

  // 8. Closed-Loop PID Flight Control (uses the EKF's final RPY, as before)
  if (baseThrottlePercent > 0.0f) {
    float roll_output  = PID_update(&roll_pid, 0.0f, fusedEKF.roll, dt);
    float pitch_output = PID_update(&pitch_pid, 0.0f, fusedEKF.pitch, dt);
    float yaw_output   = PID_update(&yaw_pid, 0.0f, fusedEKF.yaw, dt);

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

  // Smooth Low-Pass Filter Output
  if (baseThrottlePercent <= 0.0f) {
    filtered_m1 = filtered_m2 = filtered_m3 = filtered_m4 = 0.0f;
  } else {
    filtered_m1 = smoothThrottle(m1_throttle, filtered_m1, THROTTLE_ALPHA);
    filtered_m2 = smoothThrottle(m2_throttle, filtered_m2, THROTTLE_ALPHA);
    filtered_m3 = smoothThrottle(m3_throttle, filtered_m3, THROTTLE_ALPHA);
    filtered_m4 = smoothThrottle(m4_throttle, filtered_m4, THROTTLE_ALPHA);
  }

  // Write Motor Speeds
  writeMotorThrottle(MOTOR1_PIN, filtered_m1);
  writeMotorThrottle(MOTOR2_PIN, filtered_m2);
  writeMotorThrottle(MOTOR3_PIN, filtered_m3);
  writeMotorThrottle(MOTOR4_PIN, filtered_m4);

  // 9. Serial Plotting & SD Telemetry Logging (50 Hz)
  static uint32_t debugTimer = 0;
  if (millis() - debugTimer >= 20) {
    debugTimer = millis();

    char plotBuffer[256];
    snprintf(plotBuffer, sizeof(plotBuffer),
             "MPU_Roll:%.2f ICM_Roll:%.2f EKF_Roll:%.2f MPU_Pitch:%.2f ICM_Pitch:%.2f EKF_Pitch:%.2f Chosen:%c Alt:%.2f Sats:%lu Throttle:%.2f Voltage:%.2f Current:%.2f",
             mpu_roll, icm_roll, fusedEKF.roll, mpu_pitch, icm_pitch, fusedEKF.pitch, lastChosenIMU,
             baro_altitude_m, (unsigned long)gps.getSatellites(),
             baseThrottlePercent, powerData.voltageV, powerData.currentA);

    Serial.println(plotBuffer);

    if (logFile) {
      logFile.print(millis() - logStartTime); logFile.print(",");
      logFile.print(mpu_roll, 2);              logFile.print(",");
      logFile.print(icm_roll, 2);              logFile.print(",");
      logFile.print(fusedEKF.roll, 2);         logFile.print(",");
      logFile.print(mpu_pitch, 2);             logFile.print(",");
      logFile.print(icm_pitch, 2);             logFile.print(",");
      logFile.print(fusedEKF.pitch, 2);        logFile.print(",");
      logFile.print(lastChosenIMU);            logFile.print(",");
      logFile.print(baro_altitude_m, 2);       logFile.print(",");
      logFile.print(gps.getLatitude(), 6);     logFile.print(",");
      logFile.print(gps.getLongitude(), 6);    logFile.print(",");
      logFile.print(gps.getSatellites());      logFile.print(",");
      logFile.print(baseThrottlePercent, 2);   logFile.print(",");
      logFile.print(filtered_m1, 2);           logFile.print(",");
      logFile.print(filtered_m2, 2);           logFile.print(",");
      logFile.print(filtered_m3, 2);           logFile.print(",");
      logFile.print(filtered_m4, 2);           logFile.print(",");
      logFile.print(powerData.voltageV, 2);    logFile.print(",");
      logFile.print(powerData.currentA, 2);    logFile.print(",");
      logFile.println(powerData.powerW, 2);
    }

    static uint32_t flushTimer = 0;
    if (millis() - flushTimer >= 1000) {
      flushTimer = millis();
      if (logFile) logFile.flush();
    }
  }

  delay(2);
}
