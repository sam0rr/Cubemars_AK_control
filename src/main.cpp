#include <Arduino.h>
#include <mcp2515.h>
#include <SPI.h>
#include <CubemarsAK.h>

// ================= CONFIGURATION =================
#define CS_PIN          14      // Chip Select (GPIO 14)
#define MOTOR_ID        1       // Target Motor ID
#define WHEEL_DIAMETER  0.1     // 10cm Wheel
#define SERIAL_BAUD     115200

// ================= OBJECTS =================
CubemarsAK ak(CS_PIN);

// Variables for movement logic
unsigned long lastMoveTime = 0;
bool togglePosition = false;

// ================= SETUP =================
void setup() {
    Serial.begin(SERIAL_BAUD);
    while (!Serial) {}; 
    
    Serial.println("\n[SYSTEM] Starting Motion Test...");

    SPI.begin();

    // 1. Retry connecting to MCP2515 until success
    Serial.print("[INIT] Connecting to MCP2515...");
    while (ak.mcp2515.reset() != MCP2515::ERROR_OK) {
        Serial.print(".");
        delay(500); 
    }
    Serial.println(" SUCCESS!");

    // 2. Init CAN
    ak.initializeCAN();
    Serial.println("[INIT] Ready to move!");
    
    // Safety delay
    delay(1000);
}

// ================= LOOP =================
void loop() {
    // --- 1. MOVEMENT LOGIC (Every 3 seconds) ---
    if (millis() - lastMoveTime >= 3000) {
        lastMoveTime = millis();
        togglePosition = !togglePosition;

        if (togglePosition) {
            Serial.println(">>> MOVING TO: 180 Degrees");
            // Syntax: ID, Position (deg), Speed (rpm), Acceleration (rpm/s)
            ak.set_pos_spd(MOTOR_ID, 180, 1000, 500);
        } else {
            Serial.println(">>> MOVING TO: 0 Degrees");
            ak.set_pos_spd(MOTOR_ID, 0, 1000, 500);
        }
    }

    // --- 2. READING FEEDBACK ---
    if (ak.mcp2515.readMessage(&ak.canMsg2) == MCP2515::ERROR_OK) {
        
        // Decoding (Standard Servo Mode Data)
        int16_t raw_pos = (ak.canMsg2.data[0] << 8) | ak.canMsg2.data[1];
        int16_t raw_current = (ak.canMsg2.data[4] << 8) | ak.canMsg2.data[5];

        // Convert raw pos (0.1 deg/bit) to actual Degrees
        float current_deg = raw_pos * 0.1; 
        float current_amps = raw_current * 0.01;

        // Print only if ID matches (ignoring the extended bits logic for simplicity)
        // We check the lowest byte to match ID 1
        if ((ak.canMsg2.can_id & 0xFF) == MOTOR_ID) {
            Serial.print("Feedback -> Pos: ");
            Serial.print(current_deg, 1);
            Serial.print(" deg | Current: ");
            Serial.print(current_amps, 2);
            Serial.println(" A");
        }
    }
}