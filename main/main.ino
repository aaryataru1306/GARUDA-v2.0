#include <Arduino.h>
#include <SPI.h>
#include <math.h>
#include <string.h>

#include "mpu.h"
#include "icm.h"
#include "pid.h"
#include "bmp581.h"
#include "L89H_GPS.h"
#include "Lora_radio.h"

// ============================================================
// CONSTANTS & TEAM CONFIG
// ============================================================
const char* TEAM_ID = "021";
#define led_pin 14
// ============================================================
// LORA RADIO INSTANTATION & HELPERS
// ============================================================
BitBangHal radioHal;
// SX1262 pin mapping passed to RadioLib module wrapper
Module loraModule(&radioHal, PIN_CS, PIN_DIO1, PIN_RST, PIN_BUSY);
SX1262 loraRadio = &loraModule;

bool loraAvailable = false;

// Telemetry is disabled until the GCS sends CMD:START over LoRa.
bool telemetryEnabled = false;
volatile bool radioPacketReceived = false;

void onRadioPacketReceived() {
    radioPacketReceived = true;
}

bool setupLoRa() {
    Serial.println("Initializing SX1262 LoRa via BitBang SPI...");
    int state = loraRadio.begin(867.0, 125.0, 7, 7, RADIOLIB_SX126X_SYNC_WORD_PRIVATE, 10, 8);
    
    if (state == RADIOLIB_ERR_NONE) {
        Serial.println("SX1262 LoRa initialized successfully.");
        return true;
    } else {
        Serial.print("SX1262 initialization failed, code: ");
        Serial.println(state);
        return false;
    }
}

bool transmitLoRaBuffer(const char* buffer) {
    if (!loraAvailable) return false;

    int state = loraRadio.transmit((uint8_t*)buffer, strlen(buffer));
    // The radio is half-duplex: always return to command/telemetry RX.
    loraRadio.startReceive();
    return (state == RADIOLIB_ERR_NONE);
}



// ============================================================
// SENSOR CONFIGURATION
// ============================================================
SPISettings sensorSPI(1000000, MSBFIRST, SPI_MODE0);

MPU9250 mpu(10, sensorSPI);
ICM20948 icm(9, sensorSPI);
L89H_GPS gpsSens(Serial2); // Hardware Serial2 on Teensy 4.1

// ============================================================
// FILTER & TELEMETRY TIMING CONFIGURATION
// ============================================================
constexpr float FILTER_TAU = 0.5f;
constexpr float MAX_DT = 0.05f;

constexpr uint32_t MAG_INTERVAL_MS = 20;
constexpr uint32_t PRINT_INTERVAL_MS = 500;
constexpr uint32_t LORA_TX_INTERVAL_MS = 500; // 2Hz Telemetry Rate

uint32_t previousLoraTxMillis = 0;
uint32_t packetCount = 0;

void processRadioCommand() {
    if (!radioPacketReceived || !loraAvailable) return;

    noInterrupts();
    radioPacketReceived = false;
    interrupts();

    String command;
    int state = loraRadio.readData(command);

    if (state != RADIOLIB_ERR_NONE) {
        loraRadio.startReceive();
        return;
    }

    command.trim();
    command.toUpperCase();

    if (command == "CMD:START") {
        telemetryEnabled = true;
        packetCount = 0;
        previousLoraTxMillis = millis() - LORA_TX_INTERVAL_MS;
        transmitLoRaBuffer("ACK:START");
    } else if (command == "CMD:STOP") {
        telemetryEnabled = false;
        transmitLoRaBuffer("ACK:STOP");
    } else {
        loraRadio.startReceive();
    }
}

// Voltage Sensing setup
constexpr int VOLTAGE_SENSE_PIN = A9;
float getVoltage() {
    float raw = analogRead(VOLTAGE_SENSE_PIN);
    return (raw / 1023.0f) * 3.3f * 4.0f; // Adjust resistor divider factor as needed
}

// ============================================================
// BMP581 / ASCENT DETECTION
// ============================================================
constexpr uint32_t BARO_SAMPLE_INTERVAL_MS = 100;
constexpr float BARO_MIN_ALTITUDE_CHANGE_M = 0.02f;
constexpr float BARO_ASCENT_VELOCITY_THRESHOLD_MPS = 0.25f;
constexpr uint8_t ASCENT_REQUIRED_SAMPLES = 7;
constexpr float BARO_VELOCITY_FILTER_TAU = 0.5f;

enum FlightState : uint8_t {
    WAITING_FOR_ASCENT = 0,
    ASCENT
};

FlightState flightState = WAITING_FOR_ASCENT;
bool bmp581Available = false;
bool baroAltitudeInitialized = false;
bool baroVelocityInitialized = false;
float baroReferencePressurePa = NAN;
float baroAltitudeM = NAN;
float previousBaroAltitudeM = NAN;
float baroVerticalVelocityMps = 0.0f;
uint8_t ascentConsecutiveSamples = 0;
uint32_t previousBaroSampleMillis = 0;
uint32_t previousBaroSampleMicros = 0;
uint32_t previousBaroStatusMillis = 0;

// ============================================================
// SENSOR DATA
// ============================================================
float mpuAx = 0.0f, mpuAy = 0.0f, mpuAz = 0.0f;
float mpuGx = 0.0f, mpuGy = 0.0f, mpuGz = 0.0f;

float icmAx = 0.0f, icmAy = 0.0f, icmAz = 0.0f;
float icmGx = 0.0f, icmGy = 0.0f, icmGz = 0.0f;

float ax = 0.0f, ay = 0.0f, az = 1.0f;
float gx = 0.0f, gy = 0.0f, gz = 0.0f;

float magX = 0.0f, magY = 0.0f, magZ = 0.0f;

bool accelAvailable = false;
bool gyroAvailable = false;
bool magAvailable = false;

// ============================================================
// ATTITUDE STATE
// ============================================================
float roll = 0.0f;
float pitch = 0.0f;
float yaw = 0.0f;

bool filterInitialized = false;

uint32_t previousMicros = 0;
uint32_t previousMagMillis = 0;
uint32_t previousPrintMillis = 0;

// ============================================================
// RECEIVER CONFIGURATION
// ============================================================
constexpr int RX_PIN = 37;
constexpr uint8_t PPM_MAX_CHANNELS = 10;
constexpr uint8_t THROTTLE_CHANNEL_INDEX = 2;

volatile uint32_t ppm_last_edge = 0;
volatile uint8_t ppm_channel = 0;
volatile uint16_t ppm_channels[PPM_MAX_CHANNELS] = {};
volatile uint32_t ppm_last_signal = 0;

// ============================================================
// MOTOR CONFIGURATION
// ============================================================
constexpr int MOTOR1_PIN = 22;
constexpr int MOTOR2_PIN = 2;
constexpr int MOTOR3_PIN = 3;
constexpr int MOTOR4_PIN = 4;

constexpr int PWM_FREQUENCY = 50;
constexpr int PWM_RESOLUTION = 16;
constexpr uint32_t PWM_MAX_DUTY = 65535UL;

constexpr float ESC_MIN_US = 1000.0f;
constexpr float ESC_MAX_US = 2000.0f;

constexpr float MAX_MOTOR_LIMIT = 100.0f;
constexpr float THROTTLE_ALPHA = 0.15f;

// ============================================================
// ATTITUDE CONTROLLER CONFIGURATION
// ============================================================
constexpr float ANGLE_ROLL_KP = 4.0f;
constexpr float ANGLE_PITCH_KP = 4.0f;
constexpr float ANGLE_YAW_KP = 4.0f;

constexpr float MAX_ROLL_RATE = 250.0f * PI / 180.0f;
constexpr float MAX_PITCH_RATE = 250.0f * PI / 180.0f;
constexpr float MAX_YAW_RATE = 250.0f * PI / 180.0f;

// ============================================================
// PID CONTROLLERS
// ============================================================
PID_struct rate_roll_pid;
PID_struct rate_pitch_pid;
PID_struct rate_yaw_pid;

float baseThrottlePercent = 0.0f;

float m1_throttle = 0.0f;
float m2_throttle = 0.0f;
float m3_throttle = 0.0f;
float m4_throttle = 0.0f;

float filtered_m1 = 0.0f;
float filtered_m2 = 0.0f;
float filtered_m3 = 0.0f;
float filtered_m4 = 0.0f;

float targetRoll = 0.0f;
float targetPitch = 0.0f;
float targetYaw = 0.0f;

bool pidWasArmed = false;

// ============================================================
// MATH HELPERS
// ============================================================
float wrapPi(float angle) {
    while (angle >= PI) angle -= 2.0f * PI;
    while (angle < -PI) angle += 2.0f * PI;
    return angle;
}

float complementaryAlpha(float dt) {
    return FILTER_TAU / (FILTER_TAU + dt);
}

float radiansToDegrees(float angle) {
    return angle * 180.0f / PI;
}

void mapSensorAxes() {
    ax = mpuAx;
    ay = -mpuAy;
    az = -icmAz;

    gx = mpuGx;
    gy = -mpuGy;
    gz = -icmGz;
}

bool readSensors() {
    mpu.getMotion6(mpuAx, mpuAy, mpuAz, mpuGx, mpuGy, mpuGz);
    bool icmOK = icm.getMotion6(icmAx, icmAy, icmAz, icmGx, icmGy, icmGz);

    if (!icmOK) {
        gyroAvailable = false;
        accelAvailable = false;
        return false;
    }

    gyroAvailable = true;
    accelAvailable = true;
    mapSensorAxes();
    return true;
}

void updateMagnetometer() {
    uint32_t now = millis();
    if (now - previousMagMillis < MAG_INTERVAL_MS) return;
    previousMagMillis = now;

    float rawMx, rawMy, rawMz;
    if (icm.getMag(rawMx, rawMy, rawMz)) {
        magX = rawMx;
        magY = rawMy;
        magZ = -rawMz;
        magAvailable = true;
    } else {
        magAvailable = false;
    }
}

void calculateAccelerometerAngles(float &rollAccel, float &pitchAccel) {
    rollAccel = atan2f(-ax, sqrtf(ay * ay + az * az));
    pitchAccel = atan2f(-ay, -az);
}

bool calculateMagHeading(float &heading) {
    if (!magAvailable) return false;
    float magMagnitude = sqrtf(magX * magX + magY * magY + magZ * magZ);
    if (magMagnitude < 1.0e-6f) return false;

    float cr = cosf(roll);
    float sr = sinf(roll);
    float cp = cosf(pitch);
    float sp = sinf(pitch);

    float horizontalX = magX * cr + magZ * sr;
    float horizontalY = magX * sp * sr + magY * cp - magZ * sp * cr;

    heading = wrapPi(atan2f(-horizontalY, horizontalX));
    return true;
}

void initializeFilter() {
    float rollAccel, pitchAccel;
    calculateAccelerometerAngles(rollAccel, pitchAccel);
    roll = rollAccel;
    pitch = pitchAccel;

    float initialHeading;
    if (calculateMagHeading(initialHeading)) {
        yaw = initialHeading;
    } else {
        yaw = 0.0f;
    }
    filterInitialized = true;
}

void updateAttitude(float dt) {
    if (dt <= 0.0f || dt > MAX_DT) return;

    float rollAccel, pitchAccel;
    calculateAccelerometerAngles(rollAccel, pitchAccel);

    if (!filterInitialized) {
        initializeFilter();
        return;
    }

    float alpha = complementaryAlpha(dt);

    float rollPredicted = wrapPi(roll + gy * dt);
    float rollError = wrapPi(rollAccel - rollPredicted);
    roll = wrapPi(rollPredicted + (1.0f - alpha) * rollError);

    float pitchPredicted = pitch + gx * dt;
    float pitchError = pitchAccel - pitchPredicted;
    pitch = pitchPredicted + (1.0f - alpha) * pitchError;

    float yawPredicted = wrapPi(yaw + gz * dt);
    float magneticHeading;

    if (calculateMagHeading(magneticHeading)) {
        float yawError = wrapPi(magneticHeading - yawPredicted);
        yaw = wrapPi(yawPredicted + (1.0f - alpha) * yawError);
    } else {
        yaw = yawPredicted;
    }
}

void initializeBarometer() {
    Serial.println("Initializing BMP581...");
    if (!BMP581_Begin()) {
        bmp581Available = false;
        Serial.println("BMP581 initialization failed; ascent detection unavailable.");
        return;
    }

    delay(100);
    float referencePressure = NAN;
    for (uint8_t attempt = 0; attempt < 10; ++attempt) {
        referencePressure = BMP581_ReadPressure();
        if (isfinite(referencePressure) && referencePressure > 0.0f) break;
        delay(20);
    }

    if (!isfinite(referencePressure) || referencePressure <= 0.0f) {
        bmp581Available = false;
        Serial.println("BMP581 pressure read failed.");
        return;
    }

    baroReferencePressurePa = referencePressure;
    bmp581Available = true;
    baroAltitudeInitialized = false;
    baroVelocityInitialized = false;
    baroVerticalVelocityMps = 0.0f;
    ascentConsecutiveSamples = 0;
    flightState = WAITING_FOR_ASCENT;
    previousBaroSampleMillis = millis();
    previousBaroSampleMicros = micros();
    previousBaroStatusMillis = millis();
}

void updateAscentDetection() {
    if (!bmp581Available || flightState == ASCENT) return;

    const uint32_t nowMillis = millis();
    if (nowMillis - previousBaroSampleMillis < BARO_SAMPLE_INTERVAL_MS) return;

    const uint32_t nowMicros = micros();
    const uint32_t elapsedMicros = nowMicros - previousBaroSampleMicros;
    previousBaroSampleMillis = nowMillis;
    previousBaroSampleMicros = nowMicros;

    if (elapsedMicros == 0U) {
        ascentConsecutiveSamples = 0;
        return;
    }

    const float sampleDt = static_cast<float>(elapsedMicros) * 1.0e-6f;
    const float newAltitudeM = BMP581_ReadAltitude(baroReferencePressurePa);

    if (!isfinite(newAltitudeM)) {
        ascentConsecutiveSamples = 0;
        return;
    }

    baroAltitudeM = newAltitudeM;

    if (!baroAltitudeInitialized) {
        previousBaroAltitudeM = baroAltitudeM;
        baroAltitudeInitialized = true;
        return;
    }

    const float altitudeChangeM = baroAltitudeM - previousBaroAltitudeM;
    const float rawVerticalVelocityMps = altitudeChangeM / sampleDt;
    previousBaroAltitudeM = baroAltitudeM;

    if (!baroVelocityInitialized) {
        baroVerticalVelocityMps = rawVerticalVelocityMps;
        baroVelocityInitialized = true;
    } else {
        const float alpha = sampleDt / (BARO_VELOCITY_FILTER_TAU + sampleDt);
        baroVerticalVelocityMps += alpha * (rawVerticalVelocityMps - baroVerticalVelocityMps);
    }

    const bool altitudeIncreasing = altitudeChangeM > BARO_MIN_ALTITUDE_CHANGE_M;
    const bool velocityIncreasing = baroVerticalVelocityMps > BARO_ASCENT_VELOCITY_THRESHOLD_MPS;

    if (altitudeIncreasing && velocityIncreasing) {
        if (ascentConsecutiveSamples < ASCENT_REQUIRED_SAMPLES) {
            ++ascentConsecutiveSamples;
        }
    } else {
        ascentConsecutiveSamples = 0;
    }

    if (ascentConsecutiveSamples >= ASCENT_REQUIRED_SAMPLES) {
        flightState = ASCENT;
        Serial.println("ASCENT DETECTED");
    }
}

void ppm_isr() {
    const uint32_t now = micros();
    const uint32_t pulseWidth = now - ppm_last_edge;
    ppm_last_edge = now;

    if (pulseWidth > 3000) {
        ppm_channel = 0;
    } else if (pulseWidth >= 900 && pulseWidth <= 2200) {
        if (ppm_channel < PPM_MAX_CHANNELS) {
            ppm_channels[ppm_channel] = static_cast<uint16_t>(pulseWidth);
            ppm_channel++;
        }
        ppm_last_signal = now;
    }
}

float read_throttle() {
    uint16_t throttlePulse;
    uint32_t lastSignal;

    noInterrupts();
    throttlePulse = ppm_channels[THROTTLE_CHANNEL_INDEX];
    lastSignal = ppm_last_signal;
    interrupts();

    if (lastSignal == 0 || (micros() - lastSignal) > 100000UL) return 0.0f;
    if (throttlePulse < 900 || throttlePulse > 2100) return 0.0f;

    return constrain((throttlePulse - 1000.0f) / 10.0f, 0.0f, 100.0f);
}

uint32_t microsecondsToDuty(float pulseUs) {
    const float periodUs = 1000000.0f / static_cast<float>(PWM_FREQUENCY);
    pulseUs = constrain(pulseUs, ESC_MIN_US, ESC_MAX_US);
    return static_cast<uint32_t>((pulseUs / periodUs) * PWM_MAX_DUTY);
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

void resetMotorCommands() {
    m1_throttle = m2_throttle = m3_throttle = m4_throttle = 0.0f;
    filtered_m1 = filtered_m2 = filtered_m3 = filtered_m4 = 0.0f;
}

void initializeRatePIDs() {
    PID_init(&rate_roll_pid, 0.0f * RAD_TO_DEG, 0.00f * RAD_TO_DEG, 0.00f * RAD_TO_DEG, -15.0f, 15.0f);
    PID_init(&rate_pitch_pid, 0.0f * RAD_TO_DEG, 0.0f * RAD_TO_DEG, 0.0f * RAD_TO_DEG, -15.0f, 15.0f);
    PID_init(&rate_yaw_pid, 0.35f * RAD_TO_DEG, 0.001f * RAD_TO_DEG, 0.1f * RAD_TO_DEG, -10.0f, 10.0f);
}

void resetRatePIDs() {
    PID_reset(&rate_roll_pid);
    PID_reset(&rate_pitch_pid);
    PID_reset(&rate_yaw_pid);
}

void updateMotorControl(float dt) {
    baseThrottlePercent = read_throttle();

    const float rollRateRad = gy;
    const float pitchRateRad = gx;
    const float yawRateRad = gz;

    if (baseThrottlePercent <= 0.0f) {
        resetRatePIDs();
        resetMotorCommands();
        targetYaw = yaw;
        pidWasArmed = false;
        writeAllMotorsRaw(ESC_MIN_US);
        return;
    }

    if (!pidWasArmed) {
        targetYaw = yaw;
        pidWasArmed = true;
    }

    const float rollError = wrapPi(targetRoll - roll);
    const float pitchError = wrapPi(targetPitch - pitch);
    const float yawError = wrapPi(targetYaw - yaw);

    const float desiredRollRate = constrain(ANGLE_ROLL_KP * rollError, -MAX_ROLL_RATE, MAX_ROLL_RATE);
    const float desiredPitchRate = constrain(ANGLE_PITCH_KP * pitchError, -MAX_PITCH_RATE, MAX_PITCH_RATE);
    const float desiredYawRate = constrain(ANGLE_YAW_KP * yawError, -MAX_YAW_RATE, MAX_YAW_RATE);

    const float rollOutput = PID_update(&rate_roll_pid, desiredRollRate, rollRateRad, dt);
    const float pitchOutput = PID_update(&rate_pitch_pid, desiredPitchRate, pitchRateRad, dt);
    const float yawOutput = PID_update(&rate_yaw_pid, desiredYawRate, yawRateRad, dt);

    m1_throttle = baseThrottlePercent - pitchOutput + rollOutput + yawOutput;
    m2_throttle = baseThrottlePercent - pitchOutput - rollOutput - yawOutput;
    m3_throttle = baseThrottlePercent + pitchOutput - rollOutput + yawOutput;
    m4_throttle = baseThrottlePercent + pitchOutput + rollOutput - yawOutput;

    m1_throttle = constrain(m1_throttle, 0.0f, 100.0f);
    m2_throttle = constrain(m2_throttle, 0.0f, 100.0f);
    m3_throttle = constrain(m3_throttle, 0.0f, 100.0f);
    m4_throttle = constrain(m4_throttle, 0.0f, 100.0f);

    filtered_m1 = smoothThrottle(m1_throttle, filtered_m1, THROTTLE_ALPHA);
    filtered_m2 = smoothThrottle(m2_throttle, filtered_m2, THROTTLE_ALPHA);
    filtered_m3 = smoothThrottle(m3_throttle, filtered_m3, THROTTLE_ALPHA);
    filtered_m4 = smoothThrottle(m4_throttle, filtered_m4, THROTTLE_ALPHA);

    writeMotorThrottle(MOTOR1_PIN, filtered_m1);
    writeMotorThrottle(MOTOR2_PIN, filtered_m2);
    writeMotorThrottle(MOTOR3_PIN, filtered_m3);
    writeMotorThrottle(MOTOR4_PIN, filtered_m4);
}

// ============================================================
// TELEMETRY BROADCAST
// ============================================================
void sendTelemetry() {
    uint32_t now = millis();
    if (now - previousLoraTxMillis < LORA_TX_INTERVAL_MS) return;
    previousLoraTxMillis = now;

    packetCount++;

    // Altitude, Pressure, Temperature readouts from BMP581
    float bmpAltitudeM = baroAltitudeM;
    float bmpPressurePa = BMP581_ReadPressure();
    float bmpTemperatureC = BMP581_ReadTemperature();

    // Voltage
    struct { float voltageV; } powerData = { getVoltage() };

    // GPS Data
    uint32_t gnssTime = gpsSens.getGNSSTime();
    double gpsLatitude = gpsSens.getLatitude();
    double gpsLongitude = gpsSens.getLongitude();
    float gpsAltitudeM = gpsSens.getAltitudeMeters();
    uint32_t gpsSatellites = gpsSens.getSatellites();

    // Rates converted to deg/s
    float fused_gx_deg = radiansToDegrees(gx);
    float fused_gy_deg = radiansToDegrees(gy);
    float fused_gz_deg = radiansToDegrees(gz);

    // EKF/Filter Attitude converted to deg
    struct { float roll; float pitch; float yaw; } fusedEKF = {
        radiansToDegrees(roll),
        radiansToDegrees(pitch),
        radiansToDegrees(yaw)
    };

    char loraBuffer[300];
    snprintf(
        loraBuffer, sizeof(loraBuffer),
        "%s,%lu,%lu,%.1f,%.1f,%.1f,%.2f,"
        "%lu,%.6f,%.6f,%.1f,%lu,"
        "%.3f,%.3f,%.3f,"
        "%.1f,%.1f,%.1f,"
        "%.1f,%.1f,%.1f,%d",
        TEAM_ID, (unsigned long)now, (unsigned long)packetCount,
        isnan(bmpAltitudeM) ? -999.0f : bmpAltitudeM,
        isnan(bmpPressurePa) ? -999.0f : bmpPressurePa,
        isnan(bmpTemperatureC) ? -999.0f : bmpTemperatureC,
        powerData.voltageV, (unsigned long)gnssTime,
        gpsLatitude, gpsLongitude, gpsAltitudeM, (unsigned long)gpsSatellites,
        0.5f * (mpuAx + icmAx), 0.5f * (mpuAy + icmAy), 0.5f * (mpuAz + icmAz),
        fused_gx_deg, fused_gy_deg, fused_gz_deg,
        fusedEKF.roll, fusedEKF.pitch, fusedEKF.yaw,
        (int)flightState
    );

    if (transmitLoRaBuffer(loraBuffer)) {
        Serial.print("LoRa Tx OK: ");
        Serial.println(loraBuffer);
        digitalWrite(led_pin, HIGH);
        delay(10);
        digitalWrite(led_pin, LOW);

    } else {
        Serial.println("LoRa Tx Failed/Busy");
    }
}

// ============================================================
// SETUP
// ============================================================
void setup() {
    Serial.begin(115200);
    while (!Serial && millis() < 3000) {}

    Serial.println("\nTeensy 4.1 Dual-IMU Avionics System");

    SPI.begin();

    Serial.println("Initializing MPU9250...");
    mpu.begin();

    Serial.println("Initializing ICM20948...");
    uint8_t whoAmI = 0;
    if (!icm.begin(whoAmI)) {
        Serial.print("ICM20948 init failed. WHO_AM_I=0x");
        Serial.println(whoAmI, HEX);
        while (true) {
            writeAllMotorsRaw(ESC_MIN_US);
            delay(1000);
        }
    }

    if (icm.beginMag()) {
        Serial.println("Magnetometer initialized.");
    }

    mpu.calibrate(2000);
    icm.calibrate(2000);

    initializeBarometer();
    gpsSens.begin(9600);
    loraAvailable = setupLoRa();

    if (loraAvailable) {
        loraRadio.setDio2AsRfSwitch(true);
        loraRadio.setPacketReceivedAction(onRadioPacketReceived);

        int rxState = loraRadio.startReceive();
        if (rxState != RADIOLIB_ERR_NONE) {
            Serial.print("LoRa RX start failed, code: ");
            Serial.println(rxState);
            loraAvailable = false;
        } else {
            Serial.println("LoRa command RX ready; telemetry waits for CMD:START.");
        }
    }

    telemetryEnabled = false;

    pinMode(RX_PIN, INPUT);
    attachInterrupt(digitalPinToInterrupt(RX_PIN), ppm_isr, RISING);

    analogWriteResolution(PWM_RESOLUTION);
    analogWriteFrequency(MOTOR1_PIN, PWM_FREQUENCY);
    analogWriteFrequency(MOTOR2_PIN, PWM_FREQUENCY);
    analogWriteFrequency(MOTOR3_PIN, PWM_FREQUENCY);
    analogWriteFrequency(MOTOR4_PIN, PWM_FREQUENCY);

    writeAllMotorsRaw(ESC_MIN_US);
    initializeRatePIDs();

    previousMicros = micros();
    previousMagMillis = millis();
    previousPrintMillis = millis();

    Serial.println("Setup complete.");
}

// ============================================================
// MAIN LOOP
// ============================================================
void loop() {
    // Poll radio commands before sensor/control processing so START/STOP
    // still work when telemetry is disabled.
    processRadioCommand();

    const uint32_t nowMicros = micros();
    const float dt = (nowMicros - previousMicros) * 1.0e-6f;
    previousMicros = nowMicros;

    // Non-blocking read from GPS UART FIFO
    gpsSens.read();

    if (!readSensors() || dt <= 0.0f || dt > MAX_DT) {
        resetRatePIDs();
        resetMotorCommands();
        pidWasArmed = false;
        writeAllMotorsRaw(ESC_MIN_US);
        return;
    }

    updateMagnetometer();
    updateAscentDetection();

    if (!filterInitialized) {
        initializeFilter();
    } else {
        updateAttitude(dt);
    }

    updateMotorControl(dt);

    if (telemetryEnabled) {
        sendTelemetry();
    }
}
