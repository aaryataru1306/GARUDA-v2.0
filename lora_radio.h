#pragma once
#include <Arduino.h>
#include <RadioLib.h>

// ===============================
// PINS (ESP32) - LoRa bit-bang SPI
// Adjust pin numbers to match your ESP32 board
// ===============================
const int PIN_MOSI = 23;
const int PIN_MISO = 19;
const int PIN_SCK  = 18;
const int PIN_CS   = 5;
const int PIN_BUSY = 27;
const int PIN_RST  = 14;
const int PIN_DIO1 = 26;

// ===============================
// CUSTOM BIT-BANG HAL FOR RADIOLIB (ESP32)
// ===============================
class BitBangHal : public RadioLibHal {
  public:
    BitBangHal() : RadioLibHal(INPUT, OUTPUT, LOW, HIGH, RISING, FALLING) {}

    void init() override { spiBegin(); }
    void term() override { spiEnd(); }
    void pinMode(uint32_t pin, uint32_t mode) override { ::pinMode(pin, mode); }
    void digitalWrite(uint32_t pin, uint32_t value) override { ::digitalWrite(pin, value); }
    uint32_t digitalRead(uint32_t pin) override { return ::digitalRead(pin); }
    void attachInterrupt(uint32_t interruptNum, void (*interruptCb)(void), uint32_t mode) override { ::attachInterrupt(interruptNum, interruptCb, mode); }
    void detachInterrupt(uint32_t interruptNum) override { ::detachInterrupt(interruptNum); }
    void delay(unsigned long ms) override { ::delay(ms); }
    void delayMicroseconds(unsigned long us) override { ::delayMicroseconds(us); }
    unsigned long millis() override { return ::millis(); }
    unsigned long micros() override { return ::micros(); }
    void tone(uint32_t pin, unsigned int frequency, unsigned long duration = 0) override { ::tone(pin, frequency, duration); }
    void noTone(uint32_t pin) override { ::noTone(pin); }
    void yield() override { ::yield(); }

    long pulseIn(uint32_t pin, uint32_t state, uint32_t timeout) override {
        return ::pulseIn(pin, state, timeout);
    }

    // --- Soft SPI Implementation ---
    void spiBegin() override {
        ::pinMode(PIN_MOSI, OUTPUT);
        ::pinMode(PIN_SCK, OUTPUT);
        ::pinMode(PIN_MISO, INPUT_PULLUP);
        ::digitalWrite(PIN_SCK, LOW);
    }

    void spiBeginTransaction() override {}
    void spiEndTransaction() override {}
    void spiEnd() override {}

    void spiTransfer(uint8_t* out, size_t len, uint8_t* in) override {
        for (size_t i = 0; i < len; i++) {
            uint8_t receivedByte = transferByte(out ? out[i] : 0x00);
            if (in) {
                in[i] = receivedByte;
            }
        }
    }

  private:
    uint8_t transferByte(uint8_t data) {
        uint8_t readByte = 0;
        for (int i = 7; i >= 0; i--) {
            ::digitalWrite(PIN_MOSI, (data >> i) & 0x01);
            ::delayMicroseconds(2);
            ::digitalWrite(PIN_SCK, HIGH);
            ::delayMicroseconds(2);
            readByte |= (::digitalRead(PIN_MISO) << i);
            ::digitalWrite(PIN_SCK, LOW);
            ::delayMicroseconds(2);
        }
        return readByte;
    }
};