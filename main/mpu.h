#pragma once
#include <Arduino.h>
#include <SPI.h>

class MPU9250 {
private:
  int _csPin;
  SPISettings _spiSettings;

  static const uint8_t REG_PWR_MGMT_1   = 0x6B;
  static const uint8_t REG_WHO_AM_I     = 0x75;
  static const uint8_t REG_ACCEL_XOUT_H = 0x3B;

  float off_ax = 0, off_ay = 0, off_az = 0;
  float off_gx = 0, off_gy = 0, off_gz = 0;

  uint8_t readRegister(uint8_t reg) {
    SPI.beginTransaction(_spiSettings);
    digitalWrite(_csPin, LOW);
    SPI.transfer(reg | 0x80);
    uint8_t val = SPI.transfer(0x00);
    digitalWrite(_csPin, HIGH);
    SPI.endTransaction();
    return val;
  }

  void writeRegister(uint8_t reg, uint8_t val) {
    SPI.beginTransaction(_spiSettings);
    digitalWrite(_csPin, LOW);
    SPI.transfer(reg & 0x7F);
    SPI.transfer(val);
    digitalWrite(_csPin, HIGH);
    SPI.endTransaction();
  }

  void readRegisters(uint8_t reg, uint8_t count, uint8_t *dest) {
    SPI.beginTransaction(_spiSettings);
    digitalWrite(_csPin, LOW);
    SPI.transfer(reg | 0x80);
    for (uint8_t i = 0; i < count; i++) {
      dest[i] = SPI.transfer(0x00);
    }
    digitalWrite(_csPin, HIGH);
    SPI.endTransaction();
  }

public:
  MPU9250(int csPin, SPISettings spiSettings) : _csPin(csPin), _spiSettings(spiSettings) {}

  bool begin() {
    pinMode(_csPin, OUTPUT);
    digitalWrite(_csPin, HIGH);

    writeRegister(REG_PWR_MGMT_1, 0x80);
    delay(50);
    writeRegister(REG_PWR_MGMT_1, 0x01);
    delay(50);

    uint8_t who = readRegister(REG_WHO_AM_I);
    return (who != 0x00 && who != 0xFF);
  }

  void calibrate(int samples = 2000) {
    long sum_ax = 0, sum_ay = 0, sum_az = 0;
    long sum_gx = 0, sum_gy = 0, sum_gz = 0;

    for (int i = 0; i < samples; i++) {
      uint8_t buffer[14];
      readRegisters(REG_ACCEL_XOUT_H, 14, buffer);
      sum_ax += (int16_t)(buffer[0] << 8 | buffer[1]);
      sum_ay += (int16_t)(buffer[2] << 8 | buffer[3]);
      sum_az += (int16_t)(buffer[4] << 8 | buffer[5]);
      sum_gx += (int16_t)(buffer[8] << 8 | buffer[9]);
      sum_gy += (int16_t)(buffer[10] << 8 | buffer[11]);
      sum_gz += (int16_t)(buffer[12] << 8 | buffer[13]);
      delayMicroseconds(500);
    }

    off_ax = (float)sum_ax / samples;
    off_ay = (float)sum_ay / samples;
    off_az = ((float)sum_az / samples) - 16384.0f;
    off_gx = (float)sum_gx / samples;
    off_gy = (float)sum_gy / samples;
    off_gz = (float)sum_gz / samples;
  }

  void getMotion6(float &ax, float &ay, float &az, float &gx, float &gy, float &gz) {
    uint8_t buffer[14];
    readRegisters(REG_ACCEL_XOUT_H, 14, buffer);

    int16_t rawAX = (int16_t)(buffer[0] << 8 | buffer[1]);
    int16_t rawAY = (int16_t)(buffer[2] << 8 | buffer[3]);
    int16_t rawAZ = (int16_t)(buffer[4] << 8 | buffer[5]);
    int16_t rawGX = (int16_t)(buffer[8] << 8 | buffer[9]);
    int16_t rawGY = (int16_t)(buffer[10] << 8 | buffer[11]);
    int16_t rawGZ = (int16_t)(buffer[12] << 8 | buffer[13]);

    ax = (rawAX - off_ax) / 16384.0f;
    ay = (rawAY - off_ay) / 16384.0f;
    az = (rawAZ - off_az) / 16384.0f;

    // Convert deg/s to rad/s for EKF
    gx = ((rawGX - off_gx) / 131.0f) * (M_PI / 180.0f);
    gy = ((rawGY - off_gy) / 131.0f) * (M_PI / 180.0f);
    gz = ((rawGZ - off_gz) / 131.0f) * (M_PI / 180.0f);
  }
};