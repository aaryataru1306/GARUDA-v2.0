#ifndef FSM_H
#define FSM_H

#include <Arduino.h>
#include <math.h>

// Event Flags
constexpr uint8_t EVENT_NONE = 0;
constexpr uint8_t EVENT_ASCENT_DETECTED = 1;
constexpr uint8_t EVENT_APOGEE_DETECTED = 2;
constexpr uint8_t EVENT_DESCENT_DETECTED = 3;
constexpr uint8_t EVENT_ARMS_OPEN = 4;
constexpr uint8_t EVENT_ARM_DEPLOYED = 5;
constexpr uint8_t EVENT_MOTOR_HEALTH_CHECK = 6;
constexpr uint8_t EVENT_TOUCHDOWN = 7;
constexpr uint8_t EVENT_LANDED = 8;

// Flight States
enum FlightState : uint8_t {
    STATE_PAD_IDLE = 0,
    STATE_ASCENT = 1,
    STATE_APOGEE = 2,
    STATE_DESCENT = 3,
    STATE_ARMS_OPENING = 4,
    STATE_ARM_DEPLOYED = 5,
    STATE_MOTOR_HEALTH_CHECK = 6,
    STATE_TOUCHDOWN = 7,
    STATE_LANDED = 8
};

// Hardware Pin Assignments
constexpr int NICHROME_MOSFET_PIN = 21;
constexpr int ARM_FEEDBACK_PIN = 36; // Configured for Teensy 4.1 GPIO 36
constexpr int BUZZER_PIN = 23;       // Configured for Teensy 4.1 GPIO 23

// Thresholds & Timings
constexpr float ASCENT_ALTITUDE_DELTA_M = 0.05f;       // Relative altitude >= 0.05 m
constexpr float ASCENT_VELOCITY_THRESH_MPS = 0.20f;    // Vz >= +0.20 m/s

constexpr float MIN_ASCENT_ALTITUDE_FOR_APOGEE_M = 0.5f;
constexpr float APOGEE_VELOCITY_THRESH_MPS = 0.1f;
constexpr uint8_t APOGEE_CONFIRM_SAMPLES = 5;

constexpr float DESCENT_VELOCITY_THRESH_MPS = -0.2f;
constexpr uint8_t DESCENT_CONFIRM_SAMPLES = 4;

// Arm Deployment Settings
constexpr float DEPLOYMENT_TRIGGER_ALTITUDE_M = 15.0f;  // Trigger altitude AGL (m)
constexpr uint8_t DEPLOYMENT_CONFIRM_SAMPLES = 5;
constexpr uint32_t NICHROME_BURN_DURATION_MS = 3000;   // 3 seconds burn pulse

// PM02D Motor Verification Parameters
constexpr float TEST_THROTTLE_PERCENT = 30.0f;         // 30% spin-up command (~1300 us)
constexpr uint32_t SPINUP_DURATION_MS = 500;          // 500 ms spin-up period
constexpr uint8_t REQUIRED_CURRENT_SAMPLES = 5;        // 5 total readings
constexpr uint32_t SAMPLE_INTERVAL_MS = 100;          // 100 ms spacing between readings
constexpr float MIN_TOTAL_CURRENT_A = 0.17f;           // Acceptable lower total current bound
constexpr float MAX_TOTAL_CURRENT_A = 0.20f;           // Acceptable upper total current bound
constexpr float MAX_CONSECUTIVE_DIFF_A = 0.03f;        // Max allowed delta between consecutive readings

// Landing Approach & Touchdown Thresholds
constexpr float TOUCHDOWN_VELOCITY_THRESH_MPS = -0.10f; // Vz <= -0.10 m/s
constexpr float TOUCHDOWN_ALTITUDE_THRESH_M = 0.30f;    // Relative altitude < 0.30 m

constexpr float LANDED_VELOCITY_MAG_THRESH_MPS = 0.08f; // |Vz| <= 0.08 m/s
constexpr float LANDED_ALTITUDE_THRESH_M = 0.05f;       // Relative altitude < 0.05 m
constexpr uint8_t LANDED_CONFIRM_SAMPLES = 5;          // Requires 5 consecutive stable readings

constexpr float BARO_VZ_TAU = 0.3f;                     // Low-pass filter time constant for Vz

class FlightFSM {
private:
    FlightState currentState = STATE_PAD_IDLE;
    uint8_t currentEvent = EVENT_NONE;

    float referenceAltitudeM = 0.0f;
    float previousAltitudeM = 0.0f;
    float filteredVzMps = 0.0f;
    float maxAltitudeReachedM = 0.0f;

    uint8_t sampleCounter = 0;
    uint8_t deploymentCounter = 0;
    uint8_t landedCounter = 0;
    bool altitudeInitialized = false;

    // Nichrome firing state tracking
    uint32_t burnStartMillis = 0;
    bool burnCompleted = false;

    // PM02D Motor Health Check tracking
    uint32_t motorCheckStartMillis = 0;
    uint32_t lastCurrentSampleMillis = 0;
    uint8_t currentSampleCount = 0;
    float prevCurrentReadingA = 0.0f;
    bool spinupPhaseComplete = false;
    bool motorTestPassed = false;
    bool pidUnlocked = false;

    // Non-blocking siren variables
    int sirenFreq = 600;
    bool sirenRising = true;
    uint32_t previousSirenMillis = 0;

    void updateSiren() {
        uint32_t now = millis();
        if (now - previousSirenMillis >= 5) {
            previousSirenMillis = now;

            tone(BUZZER_PIN, sirenFreq);

            if (sirenRising) {
                sirenFreq += 10;
                if (sirenFreq >= 2500) {
                    sirenRising = false;
                }
            } else {
                sirenFreq -= 10;
                if (sirenFreq <= 600) {
                    sirenRising = true;
                }
            }
        }
    }

public:
    FlightFSM() = default;

    void init(float initialAltitudeM) {
        referenceAltitudeM = initialAltitudeM;
        previousAltitudeM = initialAltitudeM;
        maxAltitudeReachedM = initialAltitudeM;
        filteredVzMps = 0.0f;
        currentState = STATE_PAD_IDLE;
        currentEvent = EVENT_NONE;
        sampleCounter = 0;
        deploymentCounter = 0;
        landedCounter = 0;
        burnCompleted = false;
        altitudeInitialized = true;

        // PM02D Motor check reset
        motorCheckStartMillis = 0;
        lastCurrentSampleMillis = 0;
        currentSampleCount = 0;
        prevCurrentReadingA = 0.0f;
        spinupPhaseComplete = false;
        motorTestPassed = false;
        pidUnlocked = false;

        sirenFreq = 600;
        sirenRising = true;
        previousSirenMillis = millis();

        // Force Nichrome MOSFET gate LOW at boot
        pinMode(NICHROME_MOSFET_PIN, OUTPUT);
        digitalWrite(NICHROME_MOSFET_PIN, LOW);

        // Configure Buzzer Pin on GPIO 23 and ensure quiet at boot
        pinMode(BUZZER_PIN, OUTPUT);
        noTone(BUZZER_PIN);

        // Configure GPIO 36 with internal pull-up resistor
        pinMode(ARM_FEEDBACK_PIN, INPUT_PULLUP);
    }

    void update(float currentAltitudeM, float dt, float currentAmpsPM02D = 0.0f) {
        if (!altitudeInitialized || dt <= 0.0f) {
            init(currentAltitudeM);
            return;
        }

        currentEvent = EVENT_NONE;

        // Calculate raw vertical velocity and low-pass filter
        float rawVz = (currentAltitudeM - previousAltitudeM) / dt;
        previousAltitudeM = currentAltitudeM;

        float alpha = dt / (BARO_VZ_TAU + dt);
        filteredVzMps += alpha * (rawVz - filteredVzMps);

        if (currentAltitudeM > maxAltitudeReachedM) {
            maxAltitudeReachedM = currentAltitudeM;
        }

        // Relative altitude AGL
        float altitudeAGL = currentAltitudeM - referenceAltitudeM;
        if (altitudeAGL < 0.0f) altitudeAGL = 0.0f;

        uint32_t nowMs = millis();

        switch (currentState) {
            case STATE_PAD_IDLE: {
                if (filteredVzMps >= ASCENT_VELOCITY_THRESH_MPS && altitudeAGL >= ASCENT_ALTITUDE_DELTA_M) {
                    currentState = STATE_ASCENT;
                    currentEvent = EVENT_ASCENT_DETECTED;
                }
                break;
            }

            case STATE_ASCENT: {
                bool reachedMinAltitude = altitudeAGL >= MIN_ASCENT_ALTITUDE_FOR_APOGEE_M;
                bool vzNearZero = filteredVzMps < APOGEE_VELOCITY_THRESH_MPS;
                bool altitudeDropping = (maxAltitudeReachedM - currentAltitudeM) > 0.2f;

                if (reachedMinAltitude && (vzNearZero || altitudeDropping)) {
                    sampleCounter++;
                    if (sampleCounter >= APOGEE_CONFIRM_SAMPLES) {
                        currentState = STATE_APOGEE;
                        currentEvent = EVENT_APOGEE_DETECTED;
                        sampleCounter = 0;
                    }
                } else {
                    sampleCounter = 0;
                }
                break;
            }

            case STATE_APOGEE: {
                if (filteredVzMps < DESCENT_VELOCITY_THRESH_MPS) {
                    sampleCounter++;
                    if (sampleCounter >= DESCENT_CONFIRM_SAMPLES) {
                        currentState = STATE_DESCENT;
                        currentEvent = EVENT_DESCENT_DETECTED;
                        sampleCounter = 0;
                    }
                } else {
                    sampleCounter = 0;
                }
                break;
            }

            case STATE_DESCENT: {
                bool velocityValid = filteredVzMps < 0.0f;
                bool altitudeValid = altitudeAGL <= DEPLOYMENT_TRIGGER_ALTITUDE_M;

                if (velocityValid && altitudeValid && !burnCompleted) {
                    deploymentCounter++;
                    if (deploymentCounter >= DEPLOYMENT_CONFIRM_SAMPLES) {
                        digitalWrite(NICHROME_MOSFET_PIN, HIGH);
                        burnStartMillis = nowMs;
                        currentState = STATE_ARMS_OPENING;
                        currentEvent = EVENT_ARMS_OPEN;
                        deploymentCounter = 0;
                    }
                } else {
                    deploymentCounter = 0;
                }
                break;
            }

            case STATE_ARMS_OPENING: {
                if (nowMs - burnStartMillis >= NICHROME_BURN_DURATION_MS) {
                    digitalWrite(NICHROME_MOSFET_PIN, LOW);
                    burnCompleted = true;
                }

                if (burnCompleted && (digitalRead(ARM_FEEDBACK_PIN) == LOW)) {
                    currentState = STATE_ARM_DEPLOYED;
                    currentEvent = EVENT_ARM_DEPLOYED;
                }
                break;
            }

            case STATE_ARM_DEPLOYED: {
                digitalWrite(NICHROME_MOSFET_PIN, LOW);

                // Transition to PM02D Motor Health Verification State
                currentState = STATE_MOTOR_HEALTH_CHECK;
                currentEvent = EVENT_MOTOR_HEALTH_CHECK;
                motorCheckStartMillis = nowMs;
                lastCurrentSampleMillis = nowMs;
                currentSampleCount = 0;
                spinupPhaseComplete = false;
                motorTestPassed = true;
                break;
            }

            case STATE_MOTOR_HEALTH_CHECK: {
                digitalWrite(NICHROME_MOSFET_PIN, LOW);

                // Phase 1: 500 ms Spin-up window
                if (!spinupPhaseComplete) {
                    if (nowMs - motorCheckStartMillis >= SPINUP_DURATION_MS) {
                        spinupPhaseComplete = true;
                        lastCurrentSampleMillis = nowMs;
                    }
                } else {
                    // Phase 2: Take 5 samples 100 ms apart while holding 30% test throttle
                    if (currentSampleCount < REQUIRED_CURRENT_SAMPLES) {
                        if (nowMs - lastCurrentSampleMillis >= SAMPLE_INTERVAL_MS) {
                            lastCurrentSampleMillis = nowMs;

                            // Threshold check: total current range (0.17 A – 0.20 A)
                            if (currentAmpsPM02D < MIN_TOTAL_CURRENT_A || currentAmpsPM02D > MAX_TOTAL_CURRENT_A) {
                                motorTestPassed = false;
                            }

                            // Consecutive reading noise check (<= 0.03 A delta)
                            if (currentSampleCount > 0) {
                                float delta = fabs(currentAmpsPM02D - prevCurrentReadingA);
                                if (delta > MAX_CONSECUTIVE_DIFF_A) {
                                    motorTestPassed = false;
                                }
                            }

                            prevCurrentReadingA = currentAmpsPM02D;
                            currentSampleCount++;
                        }
                    } else {
                        // Decide: Stop test throttle and check result
                        if (motorTestPassed) {
                            pidUnlocked = true;
                            currentState = STATE_TOUCHDOWN;
                            currentEvent = EVENT_TOUCHDOWN;
                            Serial.println("PM02D MOTOR HEALTH CHECK PASSED - PID UNLOCKED");
                        } else {
                            pidUnlocked = false;
                            Serial.println("PM02D MOTOR HEALTH CHECK FAILED - PID LOCKED");
                        }
                    }
                }
                break;
            }

            case STATE_TOUCHDOWN: {
                digitalWrite(NICHROME_MOSFET_PIN, LOW);

                bool velocitySettled = fabs(filteredVzMps) <= LANDED_VELOCITY_MAG_THRESH_MPS;
                bool altitudeSettled = altitudeAGL < LANDED_ALTITUDE_THRESH_M;

                if (velocitySettled && altitudeSettled) {
                    landedCounter++;
                    if (landedCounter >= LANDED_CONFIRM_SAMPLES) {
                        currentState = STATE_LANDED;
                        currentEvent = EVENT_LANDED;
                        Serial.println("LANDED SUCCESSFULLY - SIREN ACTIVATED");
                    }
                } else {
                    landedCounter = 0;
                }
                break;
            }

            case STATE_LANDED:
                digitalWrite(NICHROME_MOSFET_PIN, LOW);
                updateSiren(); // Non-blocking siren sweep
                break;
        }
    }

    FlightState getState() const { return currentState; }
    uint8_t getEvent() const { return currentEvent; }
    float getVerticalVelocity() const { return filteredVzMps; }
    float getMaxAltitude() const { return maxAltitudeReachedM; }
    bool isArmDeployedConfirmed() const { return digitalRead(ARM_FEEDBACK_PIN) == LOW; }
    bool isPidUnlocked() const { return pidUnlocked; }
    bool isMotorTestingActive() const { return (currentState == STATE_MOTOR_HEALTH_CHECK); }
};

#endif
