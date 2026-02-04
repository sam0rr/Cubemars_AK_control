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
#define MAX_RPM 2500        // Absolute safety limit
#define GAUGE_SCALE 2500    // RPM value at 360 degrees
#define RAMP_STEP 20.0f     // RPM change per cycle
#define RAMP_INTERVAL 10    // Interval in ms (Soft Start/Stop)
#define GAUGE_SMOOTHING 0.1 // 0.1 = Very smooth, 1.0 = Instant

// --- GLOBALS ---
CubemarsAK ak(CS_PIN);
float targetRPM = 0.0f;
float currentCmdRPM = 0.0f;
float displayedRPM = 0.0f;  // For smoothing
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

void updateGauge(float realRPM) {
    // Exponential Moving Average (EMA) for smoothness
    displayedRPM = (displayedRPM * (1.0 - GAUGE_SMOOTHING)) + (abs(realRPM) * GAUGE_SMOOTHING);

    int angle = map((long)displayedRPM, 0, GAUGE_SCALE, 0, 360);
    if (angle > 360) angle = 360;
    
    sendCmd("z0.val=" + String(angle));
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
    sendCmd("z0.val=0");
    sendCmd("n0.val=1000");

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
                sendCmd("t0.txt=\"Ramping...\"");
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
        updateGauge(ak.getSpeed(MOTOR_ID));
    }
}
