#include <Arduino.h>
#include <CubemarsAK.h>
#include <SPI.h>
#include <mcp2515.h>

// CONFIGURATION
#define CS_PIN 14   // Chip Select (GPIO 14)
#define MOTOR_ID 1  // Target Motor ID
#define SERIAL_BAUD 115200

// OBJECTS
CubemarsAK ak(CS_PIN);

unsigned long lastMoveTime = 0;
bool togglePosition = false;

void setup() {
    Serial.begin(SERIAL_BAUD);
    while (!Serial);

    Serial.println(F("\n[SYSTEM] Starting AK-Series Polymorphic Control..."));

    SPI.begin();

    // 1. Initialize hardware
    ak.initializeCAN();

    // 2. Attach motor with specific model configuration
    // This tells the library that ID 1 is an AK40-10
    ak.attachMotor(MOTOR_ID, MotorPresets::AK40_10);

    delay(1000);
}

void loop() {
    // --- 1. MOVEMENT LOGIC (Every 3 seconds) ---
    if (millis() - lastMoveTime >= 3000) {
        lastMoveTime = millis();
        togglePosition = !togglePosition;

        float targetPos = togglePosition ? 180.0f : 0.0f;
        Serial.print(F(">>> COMMAND [ID "));
        Serial.print(MOTOR_ID);
        Serial.print(F("]: Moving to "));
        Serial.print(targetPos);
        Serial.println(F(" deg"));

        // The library now uses the config registered for MOTOR_ID
        ak.set_pos_spd(MOTOR_ID, targetPos, 200, 100);
    }

    // --- 2. FEEDBACK PROCESSING ---
    ak.updateFeedback();

    static unsigned long lastPrint = 0;
    if (millis() - lastPrint > 100) {
        lastPrint = millis();

        // Data is automatically scaled based on the motor's gear ratio and poles
        float p = ak.getPosition(MOTOR_ID);
        float v = ak.getSpeed(MOTOR_ID);
        float i = ak.getCurrent(MOTOR_ID);

        Serial.print(F("Motor ["));
        Serial.print(MOTOR_ID);
        Serial.print(F("] -> "));
        Serial.print(F("Pos: "));
        Serial.print(p, 1);
        Serial.print(F(" deg | "));
        Serial.print(F("Spd: "));
        Serial.print(v, 0);
        Serial.print(F(" RPM | "));
        Serial.print(F("Cur: "));
        Serial.print(i, 2);
        Serial.println(F(" A"));
    }
}
