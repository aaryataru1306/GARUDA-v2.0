#pragma once
#include <Arduino.h>
#include <SPI.h>

class ICM20948 {
private:
  int _csPin;
  SPISettings _spiSettings;

  static const uint8_t REG_BANK_SEL     = 0x7F;
  static const uint8_t REG_WHO_AM_I     = 0x00;
  static const uint8_t REG_USER_CTRL    = 0x03;
  static const uint8_t REG_PWR_MGMT_1   = 0x06;
  static const uint8_t REG_ACCEL_XOUT_H = 0x2D;

  float off_ax = 0, off_ay = 0, off_az = 0;
  float off_gx = 0, off_gy = 0, off_gz = 0;

  void selectBank(uint8_t bank) {
    SPI.beginTransaction(_spiSettings);
    digitalWrite(_csPin, LOW);
    SPI.transfer(REG_BANK_SEL & 0x7F);
    SPI.transfer((bank & 0x03) << 4);
    digitalWrite(_csPin, HIGH);
    SPI.endTransaction();
  }

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
  ICM20948(int csPin, SPISettings spiSettings) : _csPin(csPin), _spiSettings(spiSettings) {}

  bool begin(uint8_t &whoAmI) {
    pinMode(_csPin, OUTPUT);
    digitalWrite(_csPin, HIGH);

    selectBank(0);
    writeRegister(REG_PWR_MGMT_1, 0x80);
    delay(50);
    writeRegister(REG_PWR_MGMT_1, 0x01);
    delay(50);
    writeRegister(REG_USER_CTRL, 0x10);
    delay(10);

    whoAmI = readRegister(REG_WHO_AM_I);
    return (whoAmI == 0xEA);
  }

  void calibrate(int samples = 2000) {
    selectBank(0);
    long sum_ax = 0, sum_ay = 0, sum_az = 0;
    long sum_gx = 0, sum_gy = 0, sum_gz = 0;

    for (int i = 0; i < samples; i++) {
      uint8_t buffer[12];
      readRegisters(REG_ACCEL_XOUT_H, 12, buffer);
      sum_ax += (int16_t)(buffer[0] << 8 | buffer[1]);
      sum_ay += (int16_t)(buffer[2] << 8 | buffer[3]);
      sum_az += (int16_t)(buffer[4] << 8 | buffer[5]);
      sum_gx += (int16_t)(buffer[6] << 8 | buffer[7]);
      sum_gy += (int16_t)(buffer[8] << 8 | buffer[9]);
      sum_gz += (int16_t)(buffer[10] << 8 | buffer[11]);
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
    uint8_t buffer[12];
    readRegisters(REG_ACCEL_XOUT_H, 12, buffer);

    int16_t rawAX = (int16_t)(buffer[0] << 8 | buffer[1]);
    int16_t rawAY = (int16_t)(buffer[2] << 8 | buffer[3]);
    int16_t rawAZ = (int16_t)(buffer[4] << 8 | buffer[5]);
    int16_t rawGX = (int16_t)(buffer[6] << 8 | buffer[7]);
    int16_t rawGY = (int16_t)(buffer[8] << 8 | buffer[9]);
    int16_t rawGZ = (int16_t)(buffer[10] << 8 | buffer[11]);

    ax = (rawAX - off_ax) / 16384.0f;
    ay = (rawAY - off_ay) / 16384.0f;
    az = (rawAZ - off_az) / 16384.0f;

    // Convert deg/s to rad/s for EKF
    gx = ((rawGX - off_gx) / 131.0f) * (M_PI / 180.0f);
    gy = ((rawGY - off_gy) / 131.0f) * (M_PI / 180.0f);
    gz = ((rawGZ - off_gz) / 131.0f) * (M_PI / 180.0f);
  }
};