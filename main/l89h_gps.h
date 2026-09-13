#ifndef L89H_GPS_H
#define L89H_GPS_H

#include <Arduino.h>
#include <TinyGPS++.h>

// Define hardware serial port and baud rate
#define GPS_SERIAL Serial1
#define GPS_BAUD 9600

class L89H_GPS {
private:
    TinyGPSPlus gps;

public:
    void begin() {
        GPS_SERIAL.begin(GPS_BAUD);
    }

    void read() {
        while (GPS_SERIAL.available() > 0) {
            gps.encode(GPS_SERIAL.read());
        }
    }

    double getLatitude() {
        return gps.location.isValid() ? gps.location.lat() : 0.0;
    }

    double getLongitude() {
        return gps.location.isValid() ? gps.location.lng() : 0.0;
    }

    double getAltitudeMeters() {
        return gps.altitude.isValid() ? gps.altitude.meters() : 0.0;
    }

    uint32_t getSatellites() {
        return gps.satellites.isValid() ? gps.satellites.value() : 0;
    }

    bool isLocationValid() {
        return gps.location.isValid();
    }

    void getTimeFormatted(char* buffer, size_t size) {
        if (gps.time.isValid()) {
            snprintf(buffer, size, "%02d:%02d:%02d UTC",
                     gps.time.hour(),
                     gps.time.minute(),
                     gps.time.second());
        } else {
            snprintf(buffer, size, "INVALID");
        }
    }
};

#endif