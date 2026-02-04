#include <Arduino.h>
#include <CubemarsAK.h>
#include <SPI.h>
#include <mcp2515.h>

// --- CONFIGURATION ---
#define CS_PIN 14
#define MOTOR_ID 1
#define SERIAL_BAUD 115200

#define NEXTION_RX 16
#define NEXTION_TX 17

#define BTN_START_ID 1
#define BTN_STOP_ID 2

// --- SAFETY & CONTROL ---
#define MAX_RPM 2500        // 100% on Progress Bar
#define RAMP_STEP 20.0f     // RPM change per cycle
#define RAMP_INTERVAL 10    // Interval in ms (Soft Start/Stop)
#define DISPLAY_SMOOTHING 0.1 // 0.1 = Very smooth animation

// --- GLOBALS ---
CubemarsAK ak(CS_PIN);
float targetRPM = 0.0f;
float currentCmdRPM = 0.0f;
float displayedRPM = 0.0f;  // For visual smoothing
unsigned long lastRampTime = 0;
unsigned long lastGuiUpdate = 0;

// --- NEXTION HELPERS ---

void sendCmd(const String& cmd) {
    Serial2.print(cmd);
    Serial2.write(0xFF);
    Serial2.write(0xFF);
    Serial2.write(0xFF);
}

int32_t getVal(const String& var) {
    while (Serial2.available()) Serial2.read(); // Clear buffer
    sendCmd("get " + var);

    uint32_t start = millis();
    while (Serial2.available() < 8) {
        if (millis() - start > 50) return -1; // Fast timeout
    }

    if (Serial2.read() == 0x71) {
        int32_t val = 0;
        val |= Serial2.read();
        val |= (Serial2.read() << 8);
        val |= (Serial2.read() << 16);
        val |= (Serial2.read() << 24);
        Serial2.read(); Serial2.read(); Serial2.read(); // Consume terminators
        return val;
    }
    return -1;
}

void updateProgressBar(float realRPM) {
    // Exponential Moving Average (EMA) for smooth bar movement
    displayedRPM = (displayedRPM * (1.0 - DISPLAY_SMOOTHING)) + (abs(realRPM) * DISPLAY_SMOOTHING);

    // Map RPM (0 to MAX) to Percentage (0 to 100)
    int percent = map((long)displayedRPM, 0, MAX_RPM, 0, 100);
    
    // Clamp to 100% to avoid overflow
    if (percent > 100) percent = 100;
    
    // Send to Progress Bar Component (j0)
    sendCmd("j0.val=" + String(percent));
}

// --- SETUP ---

void setup() {
    Serial.begin(SERIAL_BAUD);
    Serial2.begin(9600, SERIAL_8N1, NEXTION_RX, NEXTION_TX);

    SPI.begin();
    ak.initializeCAN();
    ak.attachMotor(MOTOR_ID, MotorPresets::AK40_10);

    // Initial State
    ak.set_spd(MOTOR_ID, 0.0f);
    sendCmd("t0.txt=\"Ready\"");
    sendCmd("j0.val=0");      // Reset Progress Bar
    sendCmd("n0.val=1000");   // Default speed input

    Serial.println(F("[SYSTEM] Control Ready."));
}

// --- MAIN LOOP ---

void loop() {
    // 1. HANDLE NEXTION INPUT
    if (Serial2.available()) {
        if (Serial2.read() == 0x65) { // Touch Event
            delay(5); // Wait for packet
            Serial2.read();           // Skip Page
            byte id = Serial2.read(); // Component ID
            while (Serial2.available()) Serial2.read(); // Flush

            if (id == BTN_START_ID) {
                int32_t input = getVal("n0.val");
                if (input <= 0) input = 500; // Fallback
                
                // Clamp input
                if (input > MAX_RPM) {
                    input = MAX_RPM;
                    sendCmd("n0.val=" + String(MAX_RPM)); // Feedback correction
                }
                
                targetRPM = (float)input;
                sendCmd("t0.txt=\"Accel...\"");
            } 
            else if (id == BTN_STOP_ID) {
                targetRPM = 0.0f;
                sendCmd("t0.txt=\"Stopping...\"");
            }
        }
    }

    // 2. SOFT START/STOP RAMP LOGIC
    if (millis() - lastRampTime >= RAMP_INTERVAL) {
        lastRampTime = millis();
        
        if (abs(currentCmdRPM - targetRPM) > RAMP_STEP) {
            if (currentCmdRPM < targetRPM) currentCmdRPM += RAMP_STEP;
            else currentCmdRPM -= RAMP_STEP;
        } else {
            currentCmdRPM = targetRPM; // Snap to target when close
            if (targetRPM == 0 && currentCmdRPM == 0) sendCmd("t0.txt=\"Stopped\"");
            else if (targetRPM > 0 && currentCmdRPM == targetRPM) sendCmd("t0.txt=\"Running\"");
        }

        // Send velocity command (Velocity Mode)
        ak.set_spd(MOTOR_ID, currentCmdRPM);
    }

    // 3. TELEMETRY & DISPLAY
    ak.updateFeedback();

    if (millis() - lastGuiUpdate > 100) { // 10Hz update rate
        lastGuiUpdate = millis();
        updateProgressBar(ak.getSpeed(MOTOR_ID));
    }
}
