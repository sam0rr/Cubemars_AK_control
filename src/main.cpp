#include <Arduino.h>
#include <CubemarsAK.h>
#include <SPI.h>
#include <mcp2515.h>

// ================= CONFIGURATION =================
#define CS_PIN 14   // Chip Select (GPIO 14)
#define MOTOR_ID 1  // Target Motor ID
#define SERIAL_BAUD 115200

// ================= OBJECTS =================
CubemarsAK ak(CS_PIN);

unsigned long lastMoveTime = 0;
bool togglePosition = false;

void setup() {
    Serial.begin(SERIAL_BAUD);
    while (!Serial);

    Serial.println("\n[SYSTEM] Starting AK40-10 Professional Control...");

    SPI.begin();
    ak.initializeCAN();

    delay(1000);
}

void loop() {
    // --- 1. MOVEMENT LOGIC (Every 3 seconds) ---
    if (millis() - lastMoveTime >= 3000) {
        lastMoveTime = millis();
        togglePosition = !togglePosition;

        float targetPos = togglePosition ? 180.0f : 0.0f;
        Serial.print(">>> COMMAND: Moving to ");
        Serial.print(targetPos);
        Serial.println(" deg");

        // ID, Pos, Speed (Mechanical RPM), Accel (Mechanical RPM/s)
        ak.set_pos_spd(MOTOR_ID, targetPos, 200, 100);
    }

    // --- 2. FEEDBACK PROCESSING ---
    // Update the internal state of all motors connected to the bus
    ak.updateFeedback();

    // Print feedback at 10Hz to avoid flooding serial
    static unsigned long lastPrint = 0;
    if (millis() - lastPrint > 100) {
        lastPrint = millis();

        float p = ak.getPosition(MOTOR_ID);
        float v = ak.getSpeed(MOTOR_ID);
        float i = ak.getCurrent(MOTOR_ID);

        Serial.print("Motor [");
        Serial.print(MOTOR_ID);
        Serial.print("] -> ");
        Serial.print("Pos: ");
        Serial.print(p, 1);
        Serial.print(" deg | ");
        Serial.print("Spd: ");
        Serial.print(v, 0);
        Serial.print(" RPM | ");
        Serial.print("Cur: ");
        Serial.print(i, 2);
        Serial.println(" A");
    }
}
