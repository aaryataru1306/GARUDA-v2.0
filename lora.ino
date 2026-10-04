#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>

#include "mpu.h"
#include "lora_radio.h"

// =====================================================
// MPU9250
// =====================================================

MPU9250 mpu(0x68);

// =====================================================
// SX1262
// =====================================================

BitBangHal customHal;

SX1262 radio = new Module(
  &customHal,
  PIN_CS,
  PIN_DIO1,
  PIN_RST,
  PIN_BUSY
);

// =====================================================
// SETTINGS
// =====================================================

#define LORA_FREQUENCY 867.5
#define LORA_BANDWIDTH 125.0
#define LORA_SF        7
#define LORA_CR        5
#define LORA_POWER     22

#define TELEMETRY_INTERVAL 500

// =====================================================
// STATE
// =====================================================

bool telemetryEnabled = false;

unsigned long lastTelemetryTime = 0;

uint32_t packetCount = 0;

// =====================================================
// SETUP
// =====================================================

void setup()
{
  Serial.begin(115200);

  while (!Serial && millis() < 3000)
  {
    delay(10);
  }

  Serial.println();
  Serial.println("========================================");
  Serial.println("       CANSAT ESP32 TRANSMITTER");
  Serial.println("========================================");

  // ===================================================
  // I2C
  // ===================================================

  Serial.println("Initializing I2C...");

  Wire.begin();

  Wire.setClock(400000);

  // ===================================================
  // MPU9250
  // ===================================================

  Serial.println("Initializing MPU9250...");

  if (!mpu.begin())
  {
    Serial.println("MPU9250 I2C INIT FAILED!");

    while (true)
    {
      delay(1000);
    }
  }

  Serial.println("MPU9250 detected.");

  Serial.println("Calibrating MPU9250...");
  Serial.println("Keep the CanSat stationary.");

  mpu.calibrate();

  Serial.println("MPU9250 calibration complete.");

  // ===================================================
  // SX1262
  // ===================================================

  Serial.println();
  Serial.println("Initializing SX1262...");

  int state = radio.begin(
    LORA_FREQUENCY,
    LORA_BANDWIDTH,
    LORA_SF,
    LORA_CR,

    // SAME SYNC WORD AS RECEIVER
    0x12,

    LORA_POWER,

    8,

    3.3
  );

  if (state != RADIOLIB_ERR_NONE)
  {
    Serial.println("SX1262 INIT FAILED!");

    Serial.print("Error code: ");
    Serial.println(state);

    while (true)
    {
      delay(1000);
    }
  }

  // ===================================================
  // DIO2 RF SWITCH
  // ===================================================

  radio.setDio2AsRfSwitch(true);

  Serial.println("SX1262 initialized successfully.");

  // ===================================================
  // START IN RECEIVE MODE
  // ===================================================

  state = radio.startReceive();

  if (state != RADIOLIB_ERR_NONE)
  {
    Serial.print("RX START FAILED: ");
    Serial.println(state);

    while (true)
    {
      delay(1000);
    }
  }

  Serial.println();
  Serial.println("========================================");
  Serial.println("TRANSMITTER READY");
  Serial.println("Waiting for START command...");
  Serial.println("========================================");

  telemetryEnabled = false;
}

// =====================================================
// PROCESS COMMAND
// =====================================================

void processCommand(String command)
{
  command.trim();
  command.toUpperCase();

  Serial.println();
  Serial.println("----------------------------------------");
  Serial.print("COMMAND RECEIVED: ");
  Serial.println(command);
  Serial.println("----------------------------------------");

  // ===================================================
  // START
  // ===================================================

  if (command == "CMD:START")
  {
    telemetryEnabled = true;

    packetCount = 0;

    lastTelemetryTime = millis();

    Serial.println(">>> TELEMETRY STARTED <<<");

    // -----------------------------------------------
    // SEND ACK
    // -----------------------------------------------

    int state = radio.transmit("ACK:START");

    if (state == RADIOLIB_ERR_NONE)
    {
      Serial.println("ACK:START sent.");
    }
    else
    {
      Serial.print("ACK TX ERROR: ");
      Serial.println(state);
    }

    // -----------------------------------------------
    // VERY IMPORTANT
    // RETURN TO RX
    // -----------------------------------------------

    radio.startReceive();
  }

  // ===================================================
  // STOP
  // ===================================================

  else if (command == "CMD:STOP")
  {
    telemetryEnabled = false;

    Serial.println(">>> TELEMETRY STOPPED <<<");

    // -----------------------------------------------
    // SEND ACK
    // -----------------------------------------------

    int state = radio.transmit("ACK:STOP");

    if (state == RADIOLIB_ERR_NONE)
    {
      Serial.println("ACK:STOP sent.");
    }
    else
    {
      Serial.print("ACK TX ERROR: ");
      Serial.println(state);
    }

    // -----------------------------------------------
    // RETURN TO RX
    // -----------------------------------------------

    radio.startReceive();
  }

  // ===================================================
  // UNKNOWN COMMAND
  // ===================================================

  else
  {
    Serial.println("Unknown command.");

    radio.startReceive();
  }
}

// =====================================================
// CHECK FOR COMMAND
// =====================================================

void checkForCommand()
{
  String command;

  // Wait for a command for 30 ms
  int state = radio.receive(command, 30);

  if (state == RADIOLIB_ERR_NONE)
  {
    processCommand(command);
  }
}

// =====================================================
// SEND TELEMETRY
// =====================================================

void sendTelemetry()
{
  float ax;
  float ay;
  float az;

  float gx;
  float gy;
  float gz;

  // ===================================================
  // READ MPU
  // ===================================================

  mpu.getMotion6(
    ax,
    ay,
    az,
    gx,
    gy,
    gz
  );

  packetCount++;

  // ===================================================
  // CREATE PACKET
  // ===================================================

  char telemetryBuf[160];

  snprintf(
    telemetryBuf,
    sizeof(telemetryBuf),

    "TEL,PKT=%lu,AX=%.3f,AY=%.3f,AZ=%.3f,GX=%.3f,GY=%.3f,GZ=%.3f",

    (unsigned long)packetCount,

    ax,
    ay,
    az,

    gx,
    gy,
    gz
  );

  // ===================================================
  // SERIAL DEBUG
  // ===================================================

  Serial.print("TX: ");
  Serial.println(telemetryBuf);

  // ===================================================
  // TRANSMIT
  // ===================================================

  int state = radio.transmit(telemetryBuf);

  if (state == RADIOLIB_ERR_NONE)
  {
    Serial.println("Telemetry sent.");
  }
  else
  {
    Serial.print("Telemetry TX ERROR: ");
    Serial.println(state);
  }

  // ===================================================
  // VERY IMPORTANT
  // GO BACK TO RX
  // ===================================================

  radio.startReceive();
}

// =====================================================
// LOOP
// =====================================================

void loop()
{
  // ===================================================
  // IF TELEMETRY IS OFF
  //
  // Stay in RX and wait for START
  // ===================================================

  if (!telemetryEnabled)
  {
    checkForCommand();

    delay(5);

    return;
  }

  // ===================================================
  // TELEMETRY IS ON
  //
  // First give the receiver a chance to send STOP.
  // ===================================================

  checkForCommand();

  // ===================================================
  // CHECK WHETHER STOP WAS RECEIVED
  // ===================================================

  if (!telemetryEnabled)
  {
    return;
  }

  // ===================================================
  // TELEMETRY TIMER
  // ===================================================

  if (millis() - lastTelemetryTime >= TELEMETRY_INTERVAL)
  {
    lastTelemetryTime = millis();

    sendTelemetry();
  }

  delay(2);
}