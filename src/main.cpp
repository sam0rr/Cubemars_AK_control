#include <Arduino.h>
#include <mcp2515.h>
#include <SPI.h>
#include <CubemarsAK.h>

// Motor configuration
#define MOTOR_ID 1
#define WHEEL_DIAMETER 0.1

struct MotorData {
    float position;
    float speed;
    float current;
};

std::map<canid_t, MotorData> motorReadings;
CubemarsAK ak(14); // CS set to 14

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
    
    SPI.begin();
    ak.initializeCAN();

    // Initial commands to activate the motor
    power_on(MOTOR_ID);
    ak.set_origin(MOTOR_ID, 1);
    
    Serial.println("Motor initialized and ready on CS 16.");
}

void loop() {
    // Read incoming CAN messages
    while (ak.mcp2515.readMessage(&ak.canMsg2) == MCP2515::ERROR_OK) {
        canid_t id = ak.canMsg2.can_id;
        MotorData data;
        
        // Position: 0.1 deg/LSB converted to meters
        data.position = (((int16_t)(ak.canMsg2.data[0] << 8) | ak.canMsg2.data[1]) * 0.1 * PI * WHEEL_DIAMETER) / 360;
        // Speed: 10 RPM/LSB
        data.speed = ((int16_t)(ak.canMsg2.data[2] << 8) | ak.canMsg2.data[3]) * 10.0;
        // Current: 0.01 A/LSB
        data.current = ((int16_t)(ak.canMsg2.data[4] << 8) | ak.canMsg2.data[5]) * 0.01;
        
        motorReadings[id] = data;
    }

    // Format output for python_serial.py (expects exactly 9 values)
    if (!motorReadings.empty()) {
        MotorData d = motorReadings.begin()->second;
        Serial.print("SEND ");
        Serial.print(d.position, 4); Serial.print(", ");
        Serial.print(d.speed, 2);    Serial.print(", ");
        Serial.print(d.current, 2);  Serial.print(", ");
        // Fill the 6 other required values with 0.0
        Serial.println("0.0, 0.0, 0.0, 0.0, 0.0, 0.0");
    }

    // Manual command handling
    if (Serial.available()) {
        String input = Serial.readStringUntil('\n');
        if (input.startsWith("MOVE,")) {
            float target = input.substring(5).toFloat();
            ak.set_pos_spd(MOTOR_ID, target, 2000, 2000);
            Serial.print("Moving to: ");
            Serial.println(target);
        }
    }

    delay(20); 
}
