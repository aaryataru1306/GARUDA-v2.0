#ifndef BMP581_H
#define BMP581_H

#include <Arduino.h>
#include <Wire.h>
#include <stdint.h>
#include <math.h>

/*
   BMP581 header-only driver
   I2C address:
   - 0x46 when SDO is LOW
   - 0x47 when SDO is HIGH

   Unlike the BMP388, the BMP581 performs pressure/temperature
   compensation ON-CHIP. There is no calibration data to read or
   apply in software - the raw 24-bit registers already contain
   scaled, compensated values:
     temperature (degC) = raw_temp / 65536.0
     pressure    (Pa)   = raw_press / 64.0

   This file combines declarations + implementations.
   Include this file in your Arduino sketch directly.
*/

#define BMP581_I2C_ADDR        0x46

#define BMP581_REG_CHIP_ID     0x01
#define BMP581_REG_REV_ID      0x02
#define BMP581_REG_CHIP_STATUS 0x11

#define BMP581_REG_TEMP_XLSB   0x1D
#define BMP581_REG_TEMP_LSB    0x1E
#define BMP581_REG_TEMP_MSB    0x1F

#define BMP581_REG_PRESS_XLSB  0x20
#define BMP581_REG_PRESS_LSB   0x21
#define BMP581_REG_PRESS_MSB   0x22

#define BMP581_REG_INT_STATUS  0x27
#define BMP581_REG_STATUS      0x28

#define BMP581_REG_OSR_CONFIG  0x36
#define BMP581_REG_ODR_CONFIG  0x37
#define BMP581_REG_OSR_EFF     0x38

#define BMP581_REG_CMD         0x7E

#define BMP581_CHIP_ID_VALUE   0x50

#define BMP581_CMD_SOFT_RESET  0xB6

/* STATUS (0x28) bits */
#define BMP581_STATUS_DRDY_PRESS  0x10
#define BMP581_STATUS_DRDY_TEMP   0x01

static uint8_t bmp581_buffer[6];

static int32_t bmp581_raw_temperature = 0;
static uint32_t bmp581_raw_pressure = 0;

static inline bool BMP581_WriteRegister(uint8_t reg, uint8_t data)
{
    Wire.beginTransmission(BMP581_I2C_ADDR);
    Wire.write(reg);
    Wire.write(data);
    return (Wire.endTransmission() == 0);
}

static inline uint8_t BMP581_ReadRegister(uint8_t reg)
{
    Wire.beginTransmission(BMP581_I2C_ADDR);
    Wire.write(reg);
    if (Wire.endTransmission(false) != 0)
    {
        return 0;
    }

    Wire.requestFrom((uint8_t)BMP581_I2C_ADDR, (uint8_t)1);
    if (!Wire.available())
    {
        return 0;
    }

    return Wire.read();
}

static inline bool BMP581_ReadRegisters(uint8_t reg, uint8_t *buffer, uint8_t length)
{
    Wire.beginTransmission(BMP581_I2C_ADDR);
    Wire.write(reg);

    if (Wire.endTransmission(false) != 0)
    {
        return false;
    }

    uint8_t received = Wire.requestFrom((uint8_t)BMP581_I2C_ADDR, length);
    if (received != length)
    {
        return false;
    }

    uint8_t i = 0;
    while (Wire.available() && i < length)
    {
        buffer[i++] = Wire.read();
    }

    return (i == length);
}

static inline bool BMP581_Check(void)
{
    uint8_t id = BMP581_ReadRegister(BMP581_REG_CHIP_ID);
    return (id == BMP581_CHIP_ID_VALUE);
}

static inline void BMP581_Reset(void)
{
    BMP581_WriteRegister(BMP581_REG_CMD, BMP581_CMD_SOFT_RESET);
    delay(10);
}

static inline void BMP581_Init(void)
{
    /*
       OSR_CONFIG (0x36):
         bit6    press_en = 1  (enable pressure measurement)
         bits5:3 osr_p    = 3  (x8 oversampling)
         bits2:0 osr_t    = 0  (x1 oversampling)
    */
    BMP581_WriteRegister(BMP581_REG_OSR_CONFIG, 0x40 | (3 << 3) | 0);

    /*
       ODR_CONFIG (0x37):
         bit7    deep_dis = 1  (disable deep standby)
         bits6:2 odr      = 0  (highest output data rate)
         bits1:0 pwr_mode = 3  (continuous / normal streaming mode)
    */
    BMP581_WriteRegister(BMP581_REG_ODR_CONFIG, 0x80 | (0 << 2) | 0x03);
}

static inline bool BMP581_ReadRaw(void)
{
    if (!BMP581_ReadRegisters(BMP581_REG_TEMP_XLSB, bmp581_buffer, 6))
    {
        return false;
    }

    /* Temperature is a signed 24-bit value, sign-extend to int32_t */
    int32_t raw_t =
        ((uint32_t)bmp581_buffer[2] << 16) |
        ((uint32_t)bmp581_buffer[1] << 8)  |
        ((uint32_t)bmp581_buffer[0]);

    if (raw_t & 0x00800000)
    {
        raw_t |= 0xFF000000;   /* sign extend */
    }
    bmp581_raw_temperature = raw_t;

    /* Pressure is an unsigned 24-bit value */
    bmp581_raw_pressure =
        ((uint32_t)bmp581_buffer[5] << 16) |
        ((uint32_t)bmp581_buffer[4] << 8)  |
        ((uint32_t)bmp581_buffer[3]);

    return true;
}

static inline float BMP581_CompensateTemperature(int32_t raw_temp)
{
    return (float)raw_temp / 65536.0f;
}

static inline float BMP581_CompensatePressure(uint32_t raw_press)
{
    return (float)raw_press / 64.0f;
}

static inline float BMP581_ReadTemperature(void)
{
    if (!BMP581_ReadRaw())
    {
        return NAN;
    }

    return BMP581_CompensateTemperature(bmp581_raw_temperature);
}

static inline float BMP581_ReadPressure(void)
{
    if (!BMP581_ReadRaw())
    {
        return NAN;
    }

    return BMP581_CompensatePressure(bmp581_raw_pressure);
}

static inline float BMP581_ReadAltitude(float seaLevelPressure)
{
    float pressure = BMP581_ReadPressure();
    if (isnan(pressure) || seaLevelPressure <= 0.0f)
    {
        return NAN;
    }

    return 44330.0f * (1.0f - powf(pressure / seaLevelPressure, 0.1903f));
}

static inline bool BMP581_Begin(void)
{
    Wire.begin();

    BMP581_Reset();

    if (!BMP581_Check())
    {
        return false;
    }

    BMP581_Init();

    return true;
}

#endif