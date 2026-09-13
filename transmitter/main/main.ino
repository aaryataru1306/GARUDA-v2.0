#include <Arduino.h>
#include <RadioLib.h> // No SPI.h!

// ===============================
// PINS (Teensy 4.1)
// ===============================
const int PIN_MOSI = 26; 
const int PIN_MISO = 1;  
const int PIN_SCK  = 27; 
const int PIN_CS   = 0; 
const int PIN_BUSY = 3; 
const int PIN_RST  = 2; 
const int PIN_DIO1 = 4; 

// ===============================
// CUSTOM BIT-BANG HAL FOR RADIOLIB
// ===============================
class BitBangHal : public RadioLibHal {
  public:
    BitBangHal() : RadioLibHal(INPUT, OUTPUT, LOW, HIGH, RISING, FALLING) {}

    // Map Arduino core functions required by RadioLib
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

    // --- NEW: pulseIn is required by newer RadioLib versions ---
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

    // --- FIXED: Updated spiTransfer signature for newer RadioLib versions ---
    void spiTransfer(uint8_t* out, size_t len, uint8_t* in) override {
        for (size_t i = 0; i < len; i++) {
            // Send the byte if 'out' exists, otherwise send dummy byte (0x00)
            uint8_t receivedByte = transferByte(out ? out[i] : 0x00);
            
            // Save the received byte if 'in' buffer was provided
            if (in) {
                in[i] = receivedByte;
            }
        }
    }

  private:
    // Your original bit-banging SPI loop for Teensy!
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

// ===============================
// GLOBALS & RADIOLIB INSTANCE
// ===============================
BitBangHal customHal; // Instantiate our custom Software SPI HAL
SX1262 radio = new Module(&customHal, PIN_CS, PIN_DIO1, PIN_RST, PIN_BUSY);

volatile bool dio1_triggered = false;
uint32_t lastTxTime = 0;

void dio1_isr() {
    dio1_triggered = true;
}

// ===============================
// SETUP & LOOP
// ===============================
void setup() {
    Serial.begin(115200);
    while(!Serial && millis() < 3000);
    
    Serial.println("Initializing SX1262 with Custom Soft-SPI HAL (Teensy)...");

    // Initialize the radio with our custom bit-bang HAL
    // Freq: 867.5 MHz, BW: 125.0 kHz, SF: 7, CR: 4/5, SyncWord: 0x12, Pwr: 22 dBm
    int state = radio.begin(867.5, 125.0, 7, 5, 0x12, 22, 8, 3.3);
    
    if (state != RADIOLIB_ERR_NONE) {
        Serial.print("Failed! Error code: ");
        Serial.println(state);
        while (true); 
    }

    radio.setDio2AsRfSwitch(true);
    radio.setPacketSentAction(dio1_isr);

    Serial.println("SX1262 Initialization Complete.");
    
    String msg = "hello dude";
    Serial.println("Transmitting: " + msg);
    radio.startTransmit(msg);
    lastTxTime = millis();
}

void loop() {
    if (dio1_triggered) {
        dio1_triggered = false; 
        radio.finishTransmit(); // Clears IRQs and returns to standby
        Serial.println("--> TX Complete! (DIO1 Interrupt Cleared)");
    }

    if (millis() - lastTxTime > 200) {
        String msg = "hello dude";
        Serial.println("Transmitting: " + msg);
        radio.startTransmit(msg);
        lastTxTime = millis();
    }
}