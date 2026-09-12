#include <Arduino.h>
#include <RadioLib.h> // No SPI.h!

// ===============================
// SX1262 PINS (ESP32)
// ===============================
#define LORA_SCK   18
#define LORA_MISO  19
#define LORA_MOSI  23
#define LORA_NSS   5
#define LORA_RST   14
#define LORA_BUSY  27
#define LORA_DIO1  26

// ===============================
// CUSTOM BIT-BANG HAL FOR RADIOLIB
// ===============================
class BitBangHal : public RadioLibHal {
  public:
    BitBangHal() : RadioLibHal(INPUT, OUTPUT, LOW, HIGH, RISING, FALLING) {}

    // Map ESP32 core functions required by RadioLib
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
    void tone(uint32_t pin, unsigned int frequency, unsigned long duration = 0) override {} 
    void noTone(uint32_t pin) override {}
    void yield() override { ::yield(); }

    // --- NEW: pulseIn is required by newer RadioLib versions ---
    long pulseIn(uint32_t pin, uint32_t state, uint32_t timeout) override {
        return ::pulseIn(pin, state, timeout);
    }

    // --- Soft SPI Implementation ---
    void spiBegin() override {
        ::pinMode(LORA_MOSI, OUTPUT);
        ::pinMode(LORA_SCK, OUTPUT);
        ::pinMode(LORA_MISO, INPUT);
        ::digitalWrite(LORA_SCK, LOW);
        ::digitalWrite(LORA_MOSI, LOW);
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
    // Your exact original ESP32 bit-banging SPI loop
    uint8_t transferByte(uint8_t data) {
        uint8_t received = 0;
        for (int i = 7; i >= 0; i--) {
            ::digitalWrite(LORA_MOSI, (data >> i) & 0x01);
            ::digitalWrite(LORA_SCK, HIGH);
            
            received <<= 1;
            if (::digitalRead(LORA_MISO)) {
                received |= 1;
            }
            
            ::digitalWrite(LORA_SCK, LOW);
        }
        return received;
    }
};

// ===============================
// GLOBALS & RADIOLIB INSTANCE
// ===============================
BitBangHal customHal; 
SX1262 radio = new Module(&customHal, LORA_NSS, LORA_DIO1, LORA_RST, LORA_BUSY);

volatile bool receivedFlag = false;

// Interrupt Service Routine for DIO1 (Rx Done)
void dio1_isr() {
    receivedFlag = true;
}

// ===============================
// SETUP & LOOP
// ===============================
void setup() {
    Serial.begin(115200);
    delay(1000);
    
    Serial.println("\n ESP32 SX1262 RX TEST (RadioLib + Soft SPI)");

    // Initialize the radio with our custom bit-bang HAL
    // Freq: 867.5 MHz, BW: 125.0 kHz, SF: 7, CR: 4/5, SyncWord: 0x12 (private), Pwr: 22 dBm, Preamble: 8, TCXO: 3.3V
    int state = radio.begin(867.5, 125.0, 7, 5, 0x12, 22, 8, 3.3);
    
    if (state == RADIOLIB_ERR_NONE) {
        Serial.println("SX1262 initialization OK");
    } else {
        Serial.print("SX1262 initialization FAILED, code: ");
        Serial.println(state);
        while (true); 
    }

    // Enable DIO2 as internal RF Switch
    radio.setDio2AsRfSwitch(true);

    // Attach hardware interrupt for reception
    radio.setPacketReceivedAction(dio1_isr);

    // Start receiving
    state = radio.startReceive();
    if (state == RADIOLIB_ERR_NONE) {
        Serial.println("Entering RX Mode. Listening for packets...");
    }
}

void loop() {
    // Check if the DIO1 interrupt fired indicating a received packet
    if (receivedFlag) {
        receivedFlag = false; 
        
        String str;
        // readData() automatically checks buffer length, fetches payload, and clears IRQ flags
        int state = radio.readData(str);

        if (state == RADIOLIB_ERR_NONE) {
            Serial.print("Packet Received! [");
            Serial.print(str.length());
            Serial.print(" bytes]: ");
            Serial.println(str);
        } else {
            Serial.print("Receive failed, code: ");
            Serial.println(state);
        }

        // Put the radio back into continuous receive mode
        radio.startReceive();
    }
}