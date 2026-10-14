
#include <Arduino.h>
#include <math.h>

// ============================================================
// CAN-7USAT NAVIGATION CONTROLLER
// MCU: Teensy 4.1
//
// All angles: RADIANS
// Coordinates: North-East relative to fixed HOME
// Yaw: 0 = North, +pi/2 = East, +pi = South
//      positive yaw rotation = clockwise
//
// Vertical convention:
//   altitude: metres above the selected reference
//   vertical velocity: positive DOWN, m/s
//   vertical acceleration command: positive DOWN, m/s^2
//
// IMPORTANT:
// Output references must be integrated with your validated
// attitude controller, altitude controller and motor mixer.
// ============================================================

// -------------------- CONFIGURATION --------------------------

constexpr float PI_F = 3.14159265358979323846f;

constexpr float NAV_ALTITUDE_M = 50.0f;
constexpr float ZONE_INNER_M   = 50.0f;
constexpr float ZONE_OUTER_M   = 70.0f;

// Navigation mode exit thresholds used to reduce GPS chatter.
// These are not changes to the desired 50-70 m landing band.
constexpr float ZONE_EXIT_INNER_M = 45.0f;
constexpr float ZONE_EXIT_OUTER_M = 75.0f;

constexpr float LANDING_VZ_MIN = 1.0f;  // downward m/s
constexpr float LANDING_VZ_MAX = 3.0f;  // downward m/s
constexpr float LANDING_VZ_REF = 2.0f;  // tune and validate

constexpr float ALTITUDE_TOLERANCE_M = 1.5f;
constexpr float YAW_TOLERANCE_RAD = 0.12f;
constexpr float POSITION_FILTER_ALPHA = 0.20f;

constexpr uint8_t ZONE_CONFIRM_SAMPLES = 5;
constexpr uint8_t GPS_INVALID_LIMIT = 10;

// Output limits are starting interface limits, not flight-tested
// values. Tune on a bench/tethered setup before any flight.
constexpr float MAX_YAW_RATE_RAD_S = 0.5f;
constexpr float MAX_PITCH_REFERENCE = 0.20f;
constexpr float MAX_VERTICAL_ACCEL_M_S2 = 2.0f;

// -------------------- UTILITIES ------------------------------

static float clampf(float x, float lo, float hi) {
  if (x < lo) return lo;
  if (x > hi) return hi;
  return x;
}

// Wrap an angle to [-pi, pi).
static float wrapPi(float angle) {
  while (angle >= PI_F)  angle -= 2.0f * PI_F;
  while (angle < -PI_F)  angle += 2.0f * PI_F;
  return angle;
}

static float radiansFromDegrees(float degrees) {
  return degrees * (PI_F / 180.0f);
}

// -------------------- PID -------------------------------------

struct PID {
  float kp, ki, kd;
  float integral;
  float previousError;
  float integralLimit;
  float outputLimit;
  bool initialized;

  PID(float p = 0.0f,
      float i = 0.0f,
      float d = 0.0f,
      float iLimit = 1.0f,
      float outLimit = 1.0f)
    : kp(p), ki(i), kd(d),
      integral(0.0f), previousError(0.0f),
      integralLimit(iLimit), outputLimit(outLimit),
      initialized(false) {}

  void reset() {
    integral = 0.0f;
    previousError = 0.0f;
    initialized = false;
  }

  float update(float error, float dt) {
    if (dt <= 0.0f || dt > 0.2f) {
      reset();
      return 0.0f;
    }

    integral += error * dt;
    integral = clampf(integral, -integralLimit, integralLimit);

    float derivative = 0.0f;
    if (initialized) {
      derivative = (error - previousError) / dt;
    }

    previousError = error;
    initialized = true;

    float output = kp * error
                 + ki * integral
                 + kd * derivative;

    return clampf(output, -outputLimit, outputLimit);
  }
};

// -------------------- STATES ----------------------------------

enum NavState : uint8_t {
  NAV_WAIT_FOR_HOME,
  NAV_WAIT_FOR_DEPLOYMENT,
  NAV_DESCEND_TO_NAV_ALTITUDE,
  NAV_HOLD_NAV_ALTITUDE,
  NAV_ALIGN_TO_HOME,
  NAV_MOVE_TOWARD_HOME,
  NAV_MOVE_AWAY_FROM_HOME,
  NAV_CONFIRM_LANDING_ZONE,
  NAV_FINAL_DESCENT,
  NAV_LANDED,
  NAV_GPS_FAILSAFE
};

// -------------------- INPUTS ----------------------------------

struct NavInput {
  // GPS latitude/longitude are supplied in DEGREES by the GPS driver.
  float latitudeDeg;
  float longitudeDeg;
  bool gpsValid;

  // Must come from your Madgwick attitude estimator.
  // Yaw must already be converted to RADIANS.
  float yawRad;
  bool attitudeValid;

  // Relative altitude in metres, referenced to the chosen
  // deployment/navigation altitude reference.
  float altitudeM;
  bool altitudeValid;

  // Vertical velocity, positive DOWN, m/s.
  // Must be estimated from your validated vertical-state estimator.
  float verticalVelocityDownMps;
  bool verticalVelocityValid;

  // Optional safety and mission gates.
  bool deploymentConfirmed;
  bool motorsReady;
  bool groundDetected;

  // Supply the actual loop interval.
  float dtSeconds;
};

// -------------------- OUTPUTS ---------------------------------

struct NavOutput {
  NavState state;

  // Front-facing pitch reference:
  // positive = move toward the direction the front faces
  // negative = move opposite the front-facing direction
  float pitchReference;

  // Desired yaw rate, positive clockwise, in rad/s.
  float yawRateReferenceRadS;

  // Vertical acceleration command, positive DOWN, m/s^2.
  // Connect to your vertical controller/mixer after validation.
  float verticalAccelerationDownMps2;

  float northM;
  float eastM;
  float radiusM;

  float desiredYawRad;
  float yawErrorRad;

  bool homeValid;
  bool horizontalNavigationEnabled;
  bool finalDescentEnabled;
  bool landed;
  bool failsafe;
};

// -------------------- CONTROLLER ------------------------------

class NavigationController {
public:
  NavigationController()
    : yawPID(0.8f, 0.0f, 0.03f,
             0.5f, MAX_YAW_RATE_RAD_S),
      altitudePID(0.25f, 0.0f, 0.0f,
                  1.0f, 1.0f),
      verticalVelocityPID(0.8f, 0.05f, 0.0f,
                          2.0f, MAX_VERTICAL_ACCEL_M_S2) {
    state = NAV_WAIT_FOR_HOME;
    homeValid = false;
    filteredNorth = 0.0f;
    filteredEast = 0.0f;
    positionFilterInitialized = false;
    zoneCounter = 0;
    gpsInvalidCounter = 0;
    landingZoneConfirmed = false;
    previousState = NAV_WAIT_FOR_HOME;
  }

  NavOutput update(const NavInput &in) {
    NavOutput out = {};
    out.state = state;
    out.homeValid = homeValid;

    const float dt = in.dtSeconds;

    if (dt <= 0.0f || dt > 0.2f) {
      out.state = state;
      out.failsafe = true;
      return out;
    }

    // GPS validity watchdog.
    if (!in.gpsValid) {
      if (gpsInvalidCounter < 255) gpsInvalidCounter++;
    } else {
      gpsInvalidCounter = 0;
    }

    // Capture HOME only after a valid fix.
    // For real deployment, add a multi-sample stability check
    // before accepting HOME.
    if (!homeValid &&
        in.gpsValid &&
        isfinite(in.latitudeDeg) &&
        isfinite(in.longitudeDeg)) {
      homeLatitudeRad = radiansFromDegrees(in.latitudeDeg);
      homeLongitudeRad = radiansFromDegrees(in.longitudeDeg);
      homeValid = true;
    }

    if (homeValid && in.gpsValid) {
      calculateRelativePosition(in.latitudeDeg, in.longitudeDeg);
    }

    // Reset controller integrators when changing modes.
    if (state != previousState) {
      yawPID.reset();
      altitudePID.reset();
      verticalVelocityPID.reset();
      previousState = state;
    }

    // Safe neutral references by default.
    out.state = state;
    out.homeValid = homeValid;
    out.northM = filteredNorth;
    out.eastM = filteredEast;
    out.radiusM = sqrtf(filteredNorth * filteredNorth
                      + filteredEast * filteredEast);

    out.pitchReference = 0.0f;
    out.yawRateReferenceRadS = 0.0f;
    out.verticalAccelerationDownMps2 = 0.0f;

    out.desiredYawRad = in.yawRad;
    out.yawErrorRad = 0.0f;

    // No automatic motor command is generated by this module.
    if (!homeValid) {
      state = NAV_WAIT_FOR_HOME;
      out.state = state;
      return out;
    }

    // Loss of GPS while horizontal navigation is active:
    // stop generating navigation commands and signal failsafe.
    if (gpsInvalidCounter >= GPS_INVALID_LIMIT &&
        state != NAV_LANDED &&
        state != NAV_WAIT_FOR_DEPLOYMENT &&
        state != NAV_WAIT_FOR_HOME) {
      state = NAV_GPS_FAILSAFE;
    }

    switch (state) {
      case NAV_WAIT_FOR_HOME:
        state = homeValid ? NAV_WAIT_FOR_DEPLOYMENT
                          : NAV_WAIT_FOR_HOME;
        break;

      case NAV_WAIT_FOR_DEPLOYMENT:
        if (in.deploymentConfirmed && in.motorsReady &&
            in.attitudeValid && in.altitudeValid) {
          state = NAV_DESCEND_TO_NAV_ALTITUDE;
        }
        break;

      case NAV_DESCEND_TO_NAV_ALTITUDE:
        if (!in.altitudeValid || !in.verticalVelocityValid) break;

        // Positive vertical velocity means descending.
        // Above 50 m, regulate altitude toward 50 m.
        if (in.altitudeM > NAV_ALTITUDE_M + ALTITUDE_TOLERANCE_M) {
          commandAltitude(in, NAV_ALTITUDE_M, out);
        } else {
          state = NAV_HOLD_NAV_ALTITUDE;
        }
        break;

      case NAV_HOLD_NAV_ALTITUDE:
        if (!in.gpsValid || !in.attitudeValid ||
            !in.altitudeValid || !in.verticalVelocityValid) {
          break;
        }

        // Keep altitude near 50 m while selecting the horizontal
        // navigation mode.
        if (in.altitudeM > NAV_ALTITUDE_M + ALTITUDE_TOLERANCE_M ||
            in.altitudeM < NAV_ALTITUDE_M - ALTITUDE_TOLERANCE_M) {
          commandAltitude(in, NAV_ALTITUDE_M, out);
        }

        if (out.radiusM >= ZONE_INNER_M &&
            out.radiusM <= ZONE_OUTER_M) {
          state = NAV_CONFIRM_LANDING_ZONE;
          zoneCounter = 0;
        } else if (out.radiusM > ZONE_OUTER_M) {
          state = NAV_ALIGN_TO_HOME;
        } else {
          state = NAV_ALIGN_TO_HOME;
        }
        break;

      case NAV_ALIGN_TO_HOME: {
        if (!in.gpsValid || !in.attitudeValid) break;

        // HOME direction:
        // N_H = -N, E_H = -E
        const float northHome = -filteredNorth;
        const float eastHome  = -filteredEast;

        // Yaw convention: zero North, positive clockwise.
        // atan2(East, North) returns radians.
        const float desiredYaw = atan2f(eastHome, northHome);
        const float yawError = wrapPi(desiredYaw - in.yawRad);

        out.desiredYawRad = desiredYaw;
        out.yawErrorRad = yawError;
        out.yawRateReferenceRadS =
            yawPID.update(yawError, dt);

        // Wait until heading is sufficiently aligned.
        if (fabsf(yawError) <= YAW_TOLERANCE_RAD) {
          if (out.radiusM > ZONE_OUTER_M) {
            state = NAV_MOVE_TOWARD_HOME;
          } else if (out.radiusM < ZONE_INNER_M) {
            state = NAV_MOVE_AWAY_FROM_HOME;
          } else {
            state = NAV_CONFIRM_LANDING_ZONE;
            zoneCounter = 0;
          }
        }
        break;
      }

      case NAV_MOVE_TOWARD_HOME: {
        if (!in.gpsValid || !in.attitudeValid ||
            !in.altitudeValid || !in.verticalVelocityValid) break;

        // Recalculate desired heading as GPS position changes.
        const float desiredYaw =
            atan2f(-filteredEast, -filteredNorth);
        const float yawError = wrapPi(desiredYaw - in.yawRad);

        out.desiredYawRad = desiredYaw;
        out.yawErrorRad = yawError;
        out.yawRateReferenceRadS =
            yawPID.update(yawError, dt);

        if (in.altitudeM > NAV_ALTITUDE_M + ALTITUDE_TOLERANCE_M ||
            in.altitudeM < NAV_ALTITUDE_M - ALTITUDE_TOLERANCE_M) {
          commandAltitude(in, NAV_ALTITUDE_M, out);
        }

        if (out.radiusM >= ZONE_INNER_M &&
            out.radiusM <= ZONE_OUTER_M) {
          out.pitchReference = 0.0f;
          state = NAV_CONFIRM_LANDING_ZONE;
          zoneCounter = 0;
        } else if (out.radiusM < ZONE_INNER_M) {
          out.pitchReference = 0.0f;
          state = NAV_ALIGN_TO_HOME;
        } else if (fabsf(yawError) <= YAW_TOLERANCE_RAD) {
          // Positive pitch moves toward the front-facing direction.
          out.pitchReference = MAX_PITCH_REFERENCE;
        }
        break;
      }

      case NAV_MOVE_AWAY_FROM_HOME: {
        if (!in.gpsValid || !in.attitudeValid ||
            !in.altitudeValid || !in.verticalVelocityValid) break;

        // Still align the front toward HOME.
        const float desiredYaw =
            atan2f(-filteredEast, -filteredNorth);
        const float yawError = wrapPi(desiredYaw - in.yawRad);

        out.desiredYawRad = desiredYaw;
        out.yawErrorRad = yawError;
        out.yawRateReferenceRadS =
            yawPID.update(yawError, dt);

        if (in.altitudeM > NAV_ALTITUDE_M + ALTITUDE_TOLERANCE_M ||
            in.altitudeM < NAV_ALTITUDE_M - ALTITUDE_TOLERANCE_M) {
          commandAltitude(in, NAV_ALTITUDE_M, out);
        }

        if (out.radiusM >= ZONE_INNER_M &&
            out.radiusM <= ZONE_OUTER_M) {
          out.pitchReference = 0.0f;
          state = NAV_CONFIRM_LANDING_ZONE;
          zoneCounter = 0;
        } else if (out.radiusM > ZONE_OUTER_M) {
          out.pitchReference = 0.0f;
          state = NAV_ALIGN_TO_HOME;
        } else if (fabsf(yawError) <= YAW_TOLERANCE_RAD) {
          // Negative pitch moves away from HOME while the front
          // remains aligned toward HOME, as specified.
          out.pitchReference = -MAX_PITCH_REFERENCE;
        }
        break;
      }

      case NAV_CONFIRM_LANDING_ZONE:
        out.pitchReference = 0.0f;
        out.yawRateReferenceRadS = 0.0f;

        if (!in.gpsValid || !in.altitudeValid ||
            !in.verticalVelocityValid) {
          zoneCounter = 0;
          break;
        }

        // Confirm several consecutive samples inside the actual
        // target band before entering final descent.
        if (out.radiusM >= ZONE_INNER_M &&
            out.radiusM <= ZONE_OUTER_M) {
          if (zoneCounter < ZONE_CONFIRM_SAMPLES) zoneCounter++;
        } else {
          zoneCounter = 0;

          // Hysteresis prevents immediate mode switching around
          // the boundaries. These are exit thresholds only.
          if (out.radiusM < ZONE_EXIT_INNER_M ||
              out.radiusM > ZONE_EXIT_OUTER_M) {
            landingZoneConfirmed = false;
            state = NAV_HOLD_NAV_ALTITUDE;
          }
        }

        if (zoneCounter >= ZONE_CONFIRM_SAMPLES) {
          landingZoneConfirmed = true;

          // If above 50 m, regulate altitude toward 50 m first.
          // Once at 50 m, descend vertically without yaw correction.
          if (in.altitudeM > NAV_ALTITUDE_M + ALTITUDE_TOLERANCE_M) {
            commandAltitude(in, NAV_ALTITUDE_M, out);
          } else if (in.altitudeM <= NAV_ALTITUDE_M +
                                      ALTITUDE_TOLERANCE_M) {
            state = NAV_FINAL_DESCENT;
          }
        }
        break;

      case NAV_FINAL_DESCENT:
        out.pitchReference = 0.0f;
        out.yawRateReferenceRadS = 0.0f;
        out.finalDescentEnabled = true;

        // Keep the landing-radius condition checked. If the radius
        // exits the target band before descent is committed, stop
        // descent and return to navigation selection.
        if (!in.gpsValid || !in.altitudeValid ||
            !in.verticalVelocityValid) {
          out.finalDescentEnabled = false;
          break;
        }

        if (out.radiusM < ZONE_EXIT_INNER_M ||
            out.radiusM > ZONE_EXIT_OUTER_M) {
          out.finalDescentEnabled = false;
          landingZoneConfirmed = false;
          state = NAV_HOLD_NAV_ALTITUDE;
          break;
        }

        // Target a validated downward speed within 1-3 m/s.
        // PID output is a downward acceleration reference.
        out.verticalAccelerationDownMps2 =
            verticalVelocityPID.update(
              LANDING_VZ_REF - in.verticalVelocityDownMps, dt);

        // Ground detection must come from your validated logic,
        // not merely from GPS or altitude.
        if (in.groundDetected) {
          state = NAV_LANDED;
        }
        break;

      case NAV_LANDED:
        out.pitchReference = 0.0f;
        out.yawRateReferenceRadS = 0.0f;
        out.verticalAccelerationDownMps2 = 0.0f;
        out.landed = true;
        break;

      case NAV_GPS_FAILSAFE:
        // This state intentionally does not invent a hover or
        // motor-disarm action. Select a vehicle-specific, tested
        // failsafe in your flight-control system.
        out.pitchReference = 0.0f;
        out.yawRateReferenceRadS = 0.0f;
        out.verticalAccelerationDownMps2 = 0.0f;
        out.failsafe = true;
        break;
    }

    out.state = state;
    out.homeValid = homeValid;
    out.northM = filteredNorth;
    out.eastM = filteredEast;
    out.radiusM = sqrtf(filteredNorth * filteredNorth
                      + filteredEast * filteredEast);

    if (state == NAV_LANDED) out.landed = true;
    if (state == NAV_GPS_FAILSAFE) out.failsafe = true;

    return out;
  }

  void resetHome() {
    homeValid = false;
    positionFilterInitialized = false;
    state = NAV_WAIT_FOR_HOME;
    zoneCounter = 0;
    gpsInvalidCounter = 0;
    landingZoneConfirmed = false;
    yawPID.reset();
    altitudePID.reset();
    verticalVelocityPID.reset();
  }

  NavState getState() const {
    return state;
  }

private:
  NavState state;
  NavState previousState;

  bool homeValid;
  bool positionFilterInitialized;
  bool landingZoneConfirmed;

  float homeLatitudeRad = 0.0f;
  float homeLongitudeRad = 0.0f;

  float filteredNorth;
  float filteredEast;

  uint8_t zoneCounter;
  uint8_t gpsInvalidCounter;

  PID yawPID;

  // Altitude PID output is a desired DOWN velocity, m/s.
  // Its output limit is deliberately modest.
  PID altitudePID;

  // Vertical velocity PID output is desired DOWN acceleration.
  PID verticalVelocityPID;

  void calculateRelativePosition(float latitudeDeg,
                                 float longitudeDeg) {
    constexpr float EARTH_RADIUS_M = 6378137.0f;

    const float latRad = radiansFromDegrees(latitudeDeg);
    const float lonRad = radiansFromDegrees(longitudeDeg);

    const float meanLat = 0.5f * (latRad + homeLatitudeRad);

    const float north =
        (latRad - homeLatitudeRad) * EARTH_RADIUS_M;

    const float east =
        (lonRad - homeLongitudeRad) * EARTH_RADIUS_M
        * cosf(meanLat);

    if (!positionFilterInitialized) {
      filteredNorth = north;
      filteredEast = east;
      positionFilterInitialized = true;
    } else {
      filteredNorth += POSITION_FILTER_ALPHA *
                       (north - filteredNorth);
      filteredEast += POSITION_FILTER_ALPHA *
                      (east - filteredEast);
    }
  }

  void commandAltitude(const NavInput &in,
                       float targetAltitudeM,
                       NavOutput &out) {
    if (!in.altitudeValid || !in.verticalVelocityValid) return;

    // Positive altitude error means we are above target.
    // Positive requested velocity is DOWN.
    const float altitudeError = in.altitudeM - targetAltitudeM;

    const float desiredDownVelocity =
        clampf(altitudePID.update(altitudeError, in.dtSeconds),
               -1.0f, 1.0f);

    out.verticalAccelerationDownMps2 =
        verticalVelocityPID.update(
          desiredDownVelocity - in.verticalVelocityDownMps,
          in.dtSeconds);
  }
};
