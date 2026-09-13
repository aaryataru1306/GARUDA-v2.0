#pragma once
#include <Arduino.h>
#include <RadioLib.h> // No SPI.h needed here - this HAL bit-bangs its own SPI

// ===============================
// PINS (Teensy 4.1) - LoRa bit-bang SPI
// ===============================
const int PIN_MOSI = 26;
const int PIN_MISO = 39;
const int PIN_SCK  = 27;
const int PIN_CS   = 8;
const int PIN_BUSY = 7;
const int PIN_RST  = 5;
const int PIN_DIO1 = 6;

// ===============================
// CUSTOM BIT-BANG HAL FOR RADIOLIB
// (verbatim from your pasted sketch - not restructured)
// ===============================
class BitBangHal : public RadioLibHal {
  public:
    BitBangHal() : RadioLibHal(INPUT, OUTPUT, LOW, HIGH, RISING, FALLING) {}

    void init() override { spiBegin(); }
    void term() override { spiEnd(); }
    void pinMode(uint32_t pin, uint32_t mode) override { ::pinMode(pin, mode); }
    void digitalWrite(uint32_t pin, uint32_t value) override { ::digitalWriteFast(pin, value); }
    uint32_t digitalRead(uint32_t pin) override { return ::digitalReadFast(pin); }
    void attachInterrupt(uint32_t interruptNum, void (*interruptCb)(void), uint32_t mode) override { ::attachInterrupt(interruptNum, interruptCb, mode); }
    void detachInterrupt(uint32_t interruptNum) override { ::detachInterrupt(interruptNum); }
    void delay(unsigned long ms) override { ::delay(ms); }
    void delayMicroseconds(unsigned long us) override { ::delayMicroseconds(us); }
    unsigned long millis() override { return ::millis(); }
    unsigned long micros() override { return ::micros(); }
    void tone(uint32_t pin, unsigned int frequency, unsigned long duration = 0) override { ::tone(pin, frequency, duration); }
    void noTone(uint32_t pin) override { ::noTone(pin); }
    void yield() override { ::yield(); }

    // pulseIn is required by newer RadioLib versions
    long pulseIn(uint32_t pin, uint32_t state, uint32_t timeout) override {
        return ::pulseIn(pin, state, timeout);
    }

    // --- Soft SPI Implementation ---
    void spiBegin() override {
        ::pinMode(PIN_MOSI, OUTPUT);
        ::pinMode(PIN_SCK, OUTPUT);
        ::pinMode(PIN_MISO, INPUT_PULLUP);
        ::digitalWriteFast(PIN_SCK, LOW);
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
            ::digitalWriteFast(PIN_MOSI, (data >> i) & 0x01);
            ::delayMicroseconds(2);
            ::digitalWriteFast(PIN_SCK, HIGH);
            ::delayMicroseconds(2);
            readByte |= (::digitalReadFast(PIN_MISO) << i);
            ::digitalWriteFast(PIN_SCK, LOW);
            ::delayMicroseconds(2);
        }
        return readByte;
    }
};