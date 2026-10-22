#ifndef NAVIGATION_H
#define NAVIGATION_H

#include <Arduino.h>
#include <math.h>

/*
 * Two-zone GPS navigation guidance for Teensy 4.1.
 *
 * Coordinate convention:
 *   - GPS latitude/longitude inputs are in DEGREES.
 *   - All headings and attitude targets are in RADIANS.
 *   - N is positive north; E is positive east.
 *   - currentYawRad must be an ABSOLUTE compass heading:
 *       0 = north, +PI/2 = east, +/-PI = south, -PI/2 = west.
 *     It must use the same north reference as the GPS bearing.
 *
 * This header calculates guidance targets only. It does not read sensors,
 * run PID controllers, command motors, or implement vertical descent control.
 *
 * Pass gpsValid=false when the GPS fix is invalid OR stale.
 */

class Navigation {
public:
    enum State : uint8_t {
        WAITING_FOR_HOME = 0,
        NAVIGATE_TOWARD_HOME,
        DESCEND_DIRECTLY,
        GPS_INVALID
    };

    struct Output {
        State state;
        bool homeSet;
        bool gpsUsable;
        bool requestDescent;
        bool headingAligned;
        double northMeters;
        double eastMeters;
        double distanceMeters;
        float thetaRad;             // atan(|N/E|), in radians
        float desiredHeadingRad;    // 0 north, positive clockwise
        float yawErrorRad;          // wrapped to [-PI, PI)
        float targetYawRad;
        float targetPitchRad;
    };

    explicit Navigation(
        float navigationPitchRad = 0.12f,
        float yawAlignToleranceRad = 0.15f,
        double zoneDistanceMeters = 70.0
    ) :
        _pitchCommand(navigationPitchRad),
        _yawTolerance(yawAlignToleranceRad),
        _zoneDistance(zoneDistanceMeters),
        _homeSet(false),
        _candidateCount(0),
        _candidateLat(0.0),
        _candidateLon(0.0),
        _homeLat(0.0),
        _homeLon(0.0)
    {
    }

    void resetHome() {
        _homeSet = false;
        _candidateCount = 0;
        _candidateLat = 0.0;
        _candidateLon = 0.0;
        _homeLat = 0.0;
        _homeLon = 0.0;
    }

    bool isHomeSet() const { return _homeSet; }
    double homeLatitudeDeg() const { return _homeLat; }
    double homeLongitudeDeg() const { return _homeLon; }

    /*
     * HOME is accepted after 5 consecutive valid fixes that remain within
     * 3 m of the running candidate position. GPS freshness should be checked
     * by the caller before setting gpsValid=true.
     */
    Output update(
        double latitudeDeg,
        double longitudeDeg,
        bool gpsValid,
        float currentYawRad
    ) {
        Output out = makeDefaultOutput(currentYawRad);

        if (!gpsValid ||
            !isfinite(latitudeDeg) ||
            !isfinite(longitudeDeg) ||
            latitudeDeg < -90.0 || latitudeDeg > 90.0 ||
            longitudeDeg < -180.0 || longitudeDeg > 180.0 ||
            !isfinite(currentYawRad)) {
            out.state = _homeSet ? GPS_INVALID : WAITING_FOR_HOME;
            out.homeSet = _homeSet;
            out.gpsUsable = false;
            return out;
        }

        out.gpsUsable = true;

        if (!_homeSet) {
            collectHomeFix(latitudeDeg, longitudeDeg);
            out.homeSet = _homeSet;
            out.state = WAITING_FOR_HOME;
            out.requestDescent = false;
            out.targetPitchRad = 0.0f;
            out.targetYawRad = wrapPi(currentYawRad);
            return out;
        }

        const double latRad = degreesToRadians(latitudeDeg);
        const double lonRad = degreesToRadians(longitudeDeg);
        const double homeLatRad = degreesToRadians(_homeLat);
        const double homeLonRad = degreesToRadians(_homeLon);

        const double earthRadiusM = 6371000.0;
        out.northMeters = earthRadiusM * (latRad - homeLatRad);
        out.eastMeters =  earthRadiusM * cos(homeLatRad) * (lonRad - homeLonRad);

        const double N = out.northMeters;
        const double E = out.eastMeters;

        out.distanceMeters = sqrt(N * N + E * E);

        // Two-zone rule: at or below 70 m, request controlled direct descent.
        if (out.distanceMeters <= _zoneDistance) {
            out.state = DESCEND_DIRECTLY;
            out.requestDescent = true;
            out.headingAligned = true;
            out.desiredHeadingRad = wrapPi(currentYawRad);
            out.yawErrorRad = 0.0f;
            out.targetYawRad = wrapPi(currentYawRad);
            out.targetPitchRad = 0.0f;
            return out;
        }

        out.state = NAVIGATE_TOWARD_HOME;
        out.requestDescent = false;

        /*
         * User-requested angle:
         *   theta = atan(|N/E|)
         *
         * atan2(|N|, |E|) is the safe equivalent and handles E = 0.
         *
         * N,E describe the CURRENT position relative to HOME. The heading
         * below points from that position BACK toward HOME.
         *
         * Compass convention: 0 = north, positive clockwise.
         */
        const float theta = (float)atan2(fabs(N), fabs(E));
        out.thetaRad = theta;

        if (N >= 0.0 && E >= 0.0) {
            // Current position is north-east of HOME: head south-west.
            out.desiredHeadingRad = PI / 2.0f + theta;
        } else if (N < 0.0 && E >= 0.0) {
            // Current position is south-east of HOME: head north-west.
            out.desiredHeadingRad = +PI / 2.0f - theta;
        } else if (N < 0.0 && E < 0.0) {
            // Current position is south-west of HOME: head north-east.
            out.desiredHeadingRad = -PI / 2.0f + theta;
        } else {
            // N >= 0, E < 0; current position is north-west: head south-east.
            out.desiredHeadingRad = -PI / 2.0f - theta;
        }

        out.desiredHeadingRad = wrapPi(out.desiredHeadingRad);
        out.yawErrorRad = wrapPi(
            out.desiredHeadingRad - currentYawRad
        );

        out.headingAligned =
            fabsf(out.yawErrorRad) <= _yawTolerance;
        out.targetYawRad = out.desiredHeadingRad;

        // Move forward only after aligning with the HOME bearing.
        // Verify positive pitch means forward on the actual airframe.
        out.targetPitchRad =
            out.headingAligned ? _pitchCommand : 0.0f;

        return out;
    }

    static float wrapPi(float angleRad) {
        if (!isfinite(angleRad)) return 0.0f;

        while (angleRad >= PI) angleRad -= 2.0f * PI;
        while (angleRad < -PI) angleRad += 2.0f * PI;

        return angleRad;
    }

private:
    static constexpr uint8_t HOME_FIXES_REQUIRED = 5;
    static constexpr double HOME_STABILITY_RADIUS_M = 3.0;

    float _pitchCommand;
    float _yawTolerance;
    double _zoneDistance;

    bool _homeSet;
    uint8_t _candidateCount;
    double _candidateLat;
    double _candidateLon;
    double _homeLat;
    double _homeLon;

    static double degreesToRadians(double degrees) {
        return degrees * (double)PI / 180.0;
    }

    Output makeDefaultOutput(float currentYawRad) const {
        Output out;
        out.state = WAITING_FOR_HOME;
        out.homeSet = _homeSet;
        out.gpsUsable = false;
        out.requestDescent = false;
        out.headingAligned = false;
        out.northMeters = 0.0;
        out.eastMeters = 0.0;
        out.distanceMeters = 0.0;
        out.thetaRad = 0.0f;
        out.desiredHeadingRad = 0.0f;
        out.yawErrorRad = 0.0f;
        out.targetYawRad =
            isfinite(currentYawRad) ? wrapPi(currentYawRad) : 0.0f;
        out.targetPitchRad = 0.0f;
        return out;
    }

    void collectHomeFix(double latitudeDeg, double longitudeDeg) {
        if (_candidateCount == 0) {
            _candidateLat = latitudeDeg;
            _candidateLon = longitudeDeg;
            _candidateCount = 1;
            return;
        }

        const double meanLatRad = degreesToRadians(_candidateLat);
        const double dLat = degreesToRadians(latitudeDeg - _candidateLat);
        const double dLon = degreesToRadians(longitudeDeg - _candidateLon);
        const double north = 6371000.0 * dLat;
        const double east = 6371000.0 * cos(meanLatRad) * dLon;
        const double drift = sqrt(north * north + east * east);

        if (drift > HOME_STABILITY_RADIUS_M) {
            _candidateLat = latitudeDeg;
            _candidateLon = longitudeDeg;
            _candidateCount = 1;
            return;
        }

        const double count = (double)_candidateCount;
        _candidateLat =
            (_candidateLat * count + latitudeDeg) / (count + 1.0);
        _candidateLon =
            (_candidateLon * count + longitudeDeg) / (count + 1.0);

        if (_candidateCount < HOME_FIXES_REQUIRED) {
            ++_candidateCount;
        }

        if (_candidateCount >= HOME_FIXES_REQUIRED) {
            _homeLat = _candidateLat;
            _homeLon = _candidateLon;
            _homeSet = true;
        }
    }
};

#endif // NAVIGATION_H
