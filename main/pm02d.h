#pragma once
#include <Arduino.h>
#include <Wire.h>

#define PM02D_I2C_ADDR      0x45

// INA228 Register Map
#define INA228_REG_CONFIG      0x00
#define INA228_REG_ADC_CONFIG  0x01
#define INA228_REG_SHUNT_CAL   0x02
#define INA228_REG_VSHUNT      0x04
#define INA228_REG_VBUS        0x05
#define INA228_REG_DIETEMP     0x06
#define INA228_REG_CURRENT     0x07
#define INA228_REG_POWER       0x08
#define INA228_REG_ENERGY      0x09
#define INA228_REG_CHARGE      0x0A
#define INA228_REG_DIAG_ALRT   0x0B
#define INA228_REG_MANUF_ID    0x3E
#define INA228_REG_DEVICE_ID   0x3F

struct PM02D_Data {
    float voltageV;
    float currentA;
    float powerW;
    float shuntMilliV;
    float dieTempC;
    bool  valid;
};

class PM02D {
public:
    PM02D(TwoWire &wirePort = Wire, uint8_t address = PM02D_I2C_ADDR)
        : _wire(&wirePort), _address(address), _currentLSB(15.625e-6f),
          _vbusLSB(195.3125e-6f), _vshuntLSB(312.5e-9f), _tempLSB(7.8125e-3f) {
        _powerLSB = 3.2f * _currentLSB;
    }

    bool begin(uint16_t shuntCalValue = 1024) {
        // Test communication by reading Device ID
        uint16_t devId = read16(INA228_REG_DEVICE_ID);
        if ((devId >> 4) != 0x228) {
            return false;
        }

        // Soft-reset device
        write16(INA228_REG_CONFIG, 0x8000);
        delay(10);

        // Apply Shunt Calibration (Default ~1mOhm, 15.625uA LSB)
        return write16(INA228_REG_SHUNT_CAL, shuntCalValue);
    }

    bool read(PM02D_Data &data) {
        int32_t vshunt_raw  = readSigned20(INA228_REG_VSHUNT);
        uint32_t vbus_raw   = readUnsigned20(INA228_REG_VBUS);
        int32_t current_raw = readSigned20(INA228_REG_CURRENT);
        uint32_t power_raw  = readUnsigned20(INA228_REG_POWER);
        uint16_t temp_raw   = read16(INA228_REG_DIETEMP);

        if (vshunt_raw == 0x7FFFFFFF || vbus_raw == 0xFFFFFFFFUL) {
            data.valid = false;
            return false;
        }

        data.shuntMilliV = vshunt_raw * _vshuntLSB * 1000.0f;
        data.voltageV    = vbus_raw * _vbusLSB;
        data.currentA    = current_raw * _currentLSB;
        data.powerW      = power_raw * _powerLSB;
        data.dieTempC    = (int16_t)temp_raw * _tempLSB;
        data.valid       = true;

        return true;
    }

private:
    TwoWire *_wire;
    uint8_t  _address;
    float    _currentLSB;
    float    _vbusLSB;
    float    _vshuntLSB;
    float    _powerLSB;
    float    _tempLSB;

    bool write16(uint8_t reg, uint16_t val) {
        _wire->beginTransmission(_address);
        _wire->write(reg);
        _wire->write((uint8_t)(val >> 8));
        _wire->write((uint8_t)(val & 0xFF));
        return (_wire->endTransmission() == 0);
    }

    uint16_t read16(uint8_t reg) {
        uint8_t buf[2];
        if (!readN(reg, buf, 2)) return 0xFFFF;
        return ((uint16_t)buf[0] << 8) | buf[1];
    }

    bool readN(uint8_t reg, uint8_t *buf, uint8_t len) {
        _wire->beginTransmission(_address);
        _wire->write(reg);
        if (_wire->endTransmission(false) != 0) return false;
        if (_wire->requestFrom(_address, len) != len) return false;
        for (uint8_t i = 0; i < len; i++) {
            buf[i] = _wire->read();
        }
        return true;
    }

    uint32_t read24(uint8_t reg) {
        uint8_t b[3];
        if (!readN(reg, b, 3)) return 0xFFFFFFFFUL;
        return ((uint32_t)b[0] << 16) | ((uint32_t)b[1] << 8) | b[2];
    }

    int32_t signExtend20(uint32_t x) {
        x &= 0x000FFFFF;
        if (x & 0x00080000) x |= 0xFFF00000;
        return (int32_t)x;
    }

    int32_t readSigned20(uint8_t reg) {
        uint32_t raw24 = read24(reg);
        if (raw24 == 0xFFFFFFFFUL) return 0x7FFFFFFF;
        return signExtend20(raw24 >> 4);
    }

    uint32_t readUnsigned20(uint8_t reg) {
        uint32_t raw24 = read24(reg);
        if (raw24 == 0xFFFFFFFFUL) return 0xFFFFFFFFUL;
        return (raw24 >> 4) & 0x000FFFFF;
    }
};