#ifndef MPU9250_H
#define MPU9250_H

#include <Arduino.h>
#include <Wire.h>
class MPU9250 {
public:
    // Default I2C Address is 0x68 (AD0 pin connected to GND)
    // Pass 0x69 if AD0 is connected to 3.3V
    MPU9250(uint8_t address = 0x68, TwoWire &wirePort = Wire) 
        : _addr(address), _wire(&wirePort) {}

    bool begin() {
        uint8_t whoAmI = 0;

        readRegisters(MPU9250_WHO_AM_I, 1, &whoAmI);

        Serial.print("MPU WHO_AM_I = 0x");
        Serial.println(whoAmI, HEX);

        if (whoAmI != 0x70 && whoAmI != 0x71 && whoAmI != 0x73) {
            return false;
        }

        writeRegister(MPU9250_PWR_MGMT_1, 0x01);
        delay(10);

        writeRegister(MPU9250_CONFIG, 0x03);
        writeRegister(MPU9250_GYRO_CONFIG, 0x18);
        writeRegister(MPU9250_ACCEL_CONFIG, 0x10);

        return true;
    }

    void calibrate() {
        int samples = 200;
        long raw_ax = 0, raw_ay = 0, raw_az = 0;
        long raw_gx = 0, raw_gy = 0, raw_gz = 0;

        for (int i = 0; i < samples; i++) {
            uint8_t buffer[14];
            readRegisters(MPU9250_ACCEL_XOUT_H, 14, buffer);

            raw_ax += (int16_t)((buffer[0]  << 8) | buffer[1]);
            raw_ay += (int16_t)((buffer[2]  << 8) | buffer[3]);
            raw_az += (int16_t)((buffer[4]  << 8) | buffer[5]);
            raw_gx += (int16_t)((buffer[8]  << 8) | buffer[9]);
            raw_gy += (int16_t)((buffer[10] << 8) | buffer[11]);
            raw_gz += (int16_t)((buffer[12] << 8) | buffer[13]);
            delay(2);
        }

        // Accel +/-8g sensitivity = 4096 LSB/g
        ax_offset = (float)(raw_ax / samples) / 4096.0f;
        ay_offset = (float)(raw_ay / samples) / 4096.0f;
        az_offset = ((float)(raw_az / samples) / 4096.0f) - 1.0f; // Subtract 1g for gravity

        // Gyro +/-2000dps sensitivity = 16.4 LSB/(deg/s)
        gx_offset = (float)(raw_gx / samples) / 16.4f;
        gy_offset = (float)(raw_gy / samples) / 16.4f;
        gz_offset = (float)(raw_gz / samples) / 16.4f;
    }

    void getMotion6(float &ax, float &ay, float &az, float &gx, float &gy, float &gz) {
        uint8_t buffer[14];
        readRegisters(MPU9250_ACCEL_XOUT_H, 14, buffer);

        int16_t raw_ax = (int16_t)((buffer[0]  << 8) | buffer[1]);
        int16_t raw_ay = (int16_t)((buffer[2]  << 8) | buffer[3]);
        int16_t raw_az = (int16_t)((buffer[4]  << 8) | buffer[5]);

        int16_t raw_gx = (int16_t)((buffer[8]  << 8) | buffer[9]);
        int16_t raw_gy = (int16_t)((buffer[10] << 8) | buffer[11]);
        int16_t raw_gz = (int16_t)((buffer[12] << 8) | buffer[13]);

        // Scale raw values and subtract zero-motion offset
        ax = ((float)raw_ax / 4096.0f) - ax_offset;
        ay = ((float)raw_ay / 4096.0f) - ay_offset;
        az = ((float)raw_az / 4096.0f) - az_offset;

        gx = ((float)raw_gx / 16.4f) - gx_offset;
        gy = ((float)raw_gy / 16.4f) - gy_offset;
        gz = ((float)raw_gz / 16.4f) - gz_offset;
    }

private:
    uint8_t _addr;
    TwoWire *_wire;

    // Register Map Definitions
    static constexpr uint8_t MPU9250_WHO_AM_I     = 0x75;
    static constexpr uint8_t MPU9250_PWR_MGMT_1   = 0x6B;
    static constexpr uint8_t MPU9250_CONFIG       = 0x1A;
    static constexpr uint8_t MPU9250_GYRO_CONFIG  = 0x1B;
    static constexpr uint8_t MPU9250_ACCEL_CONFIG = 0x1C;
    static constexpr uint8_t MPU9250_ACCEL_XOUT_H = 0x3B;

    // Calibration offsets
    float ax_offset = 0.0f, ay_offset = 0.0f, az_offset = 0.0f;
    float gx_offset = 0.0f, gy_offset = 0.0f, gz_offset = 0.0f;

    void writeRegister(uint8_t subAddress, uint8_t data) {
        _wire->beginTransmission(_addr);
        _wire->write(subAddress);
        _wire->write(data);
        _wire->endTransmission();
    }

    void readRegisters(uint8_t subAddress, uint8_t count, uint8_t* dest) {
        _wire->beginTransmission(_addr);
        _wire->write(subAddress);
        _wire->endTransmission(false);

        _wire->requestFrom(_addr, count);
        uint8_t i = 0;
        while (_wire->available() && i < count) {
            dest[i++] = _wire->read();
        }
    }
};

#endif // MPU9250_H