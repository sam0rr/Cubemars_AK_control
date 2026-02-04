#include <Arduino.h>
#include <mcp2515.h>
#include <SPI.h>
#include <CubemarsAK.h>

#define SDA_PIN 21
#define SCL_PIN 22

// Single motor configuration (Factory Default ID = 1)
#define MOTOR_ID 1

#define WHEEL_DIAMETER 0.1

struct MotorData {
    float position;
};

std::map<canid_t, MotorData> motorReadings;
CubemarsAK ak(5); // MCP2515 CS on GPIO 5

void power_on(uint16_t motor_id) {
    struct can_frame canMsg;
    canMsg.can_id = motor_id;
    canMsg.can_dlc = 8;
    for(int i=0; i<7; i++) canMsg.data[i] = 0xFF;
    canMsg.data[7] = 0xFC;
    ak.mcp2515.sendMessage(&canMsg);
}

void setup() {
    Serial.begin(115200);
    while (!Serial) {};
    
    // Initialize SPI and I2C (even if I2C is unused now, kept for pin safety)
    SPI.begin();
    
    // Initialize CAN
    ak.initializeCAN();

    Serial.println("System Ready. Powering on motor 1...");
    power_on(MOTOR_ID);
    ak.set_origin(MOTOR_ID, 1);
}

void loop() {
    // Read messages from CAN bus
    while (ak.mcp2515.readMessage(&ak.canMsg2) == MCP2515::ERROR_OK) {
        canid_t received_id = ak.canMsg2.can_id;
        
        // Debug: See what ID the motor is actually using
        Serial.print("Received CAN ID: ");
        Serial.print(received_id);
        Serial.print(" (Hex: 0x");
        Serial.print(received_id, HEX);
        Serial.println(")");

        MotorData data;
        // Position calculation
        data.position = (((ak.canMsg2.data[0] << 8) | ak.canMsg2.data[1]) * 0.1 * PI * WHEEL_DIAMETER) / 360;
        motorReadings[received_id] = data;
    }

    // Print the last received position for ID 1 (or whatever ID was detected)
    // We use a simple way to find the first available data in the map
    if (!motorReadings.empty()) {
        auto it = motorReadings.begin();
        Serial.print("SEND ");
        Serial.println(it->second.position, 4);
    } else {
        Serial.println("SEND 0.0000 (Waiting for motor...)");
    }

    // Check for serial commands to move the motor
    if (Serial.available()) {
        String input = Serial.readStringUntil('\n');
        if (input.startsWith("MOVE,")) {
            float target_pos = input.substring(5).toFloat();
            Serial.print("Moving to: ");
            Serial.println(target_pos);
            ak.set_pos_spd(MOTOR_ID, target_pos, 1000, 1000);
        }
    }

    delay(100); 
}
