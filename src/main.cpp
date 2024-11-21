#include <Arduino.h>
#include <mcp2515.h>
#include <SPI.h>
#include <CubemarsAK.h>
#include <JrkG2.h>
#include <Wire.h>
#include <VL53L0X.h>

#define SDA_PIN 21
#define SCL_PIN 22

#define X_CONTROL 104
#define X_DATA 2147494248
#define Y_CONTROL 105
#define Y_DATA 2147494249
#define Z_CONTROL 106
#define Z_DATA 2147494250

#define POLE_PAIRS 21.0
#define CHASSIS_REDUCTION_RATIO 9.0
#define LIFT_REDUCTION_RATIO 64.0
#define WHEEL_DIAMETER 0.1
#define Kt_CHASSIS 0.105
#define Kt_LIFT 0.136


struct MotorData {
    float position;
    float speed;
    float current;
    uint8_t motorTemp;
    uint8_t errorCode;
};

std::map<canid_t, MotorData> motorReadings;

JrkG2I2C jrk;
CubemarsAK chassis(10);
CubemarsAK lift(9);

float pos_x, vel_x, cur_x = 0.0;
float pos_y, vel_y, cur_y = 0.0;
float pos_z, vel_z, cur_z = 0.0;
float cmd_x, cmd_y, cmd_z = 0.0;
bool commandReceived = false;

float getPosition(canid_t canID) {
    if (motorReadings.find(canID) != motorReadings.end()) {
        return motorReadings[canID].position;
    }
    return 0.0;
}

float getSpeed(canid_t canID) {
    if (motorReadings.find(canID) != motorReadings.end()) {
        return motorReadings[canID].speed;
    }
    return 0.0;
}

float getCurrent(canid_t canID) {
    if (motorReadings.find(canID) != motorReadings.end()) {
        return motorReadings[canID].current;
    }
    return 0.0; 
}

int8_t getMotorTemp(canid_t canID) {
    if (motorReadings.find(canID) != motorReadings.end()) {
        return motorReadings[canID].motorTemp;
    }
    return 0.0; 
}

uint8_t getErrorCode(canid_t canID) {
    if (motorReadings.find(canID) != motorReadings.end()) {
        return motorReadings[canID].errorCode;
    }
    return 0.0; 
}

void chassis_y(){
    jrk.setTarget(2870);
}

void chassis_x(){
    jrk.setTarget(410);
}

void chassis_stable(){
    jrk.setTarget(1640);
}


void sendMotorData() {

    pos_x = getPosition(X_DATA);
    vel_x = getSpeed(X_DATA);
    cur_x = getCurrent(X_DATA);

    pos_y = getPosition(Y_DATA);
    vel_y = getSpeed(Y_DATA);
    cur_y = getCurrent(Y_DATA);

    pos_z = getPosition(Z_DATA);
    vel_z = getSpeed(Z_DATA);
    cur_z = getCurrent(Z_DATA);

    // Set desired number of decimal places, e.g., 3 decimal places
    Serial.print("SEND ");
    Serial.print(pos_x, 4);
    Serial.print(", ");
    Serial.print(vel_x, 2);
    Serial.print(", ");
    Serial.print(cur_x, 2);
    Serial.print(", ");
    Serial.print(pos_y, 4);
    Serial.print(", ");
    Serial.print(vel_y, 2);
    Serial.print(", ");
    Serial.print(cur_y, 2);
    Serial.print(", ");
    Serial.print(pos_z, 4);
    Serial.print(", ");
    Serial.print(vel_z, 2);
    Serial.print(", ");
    Serial.print(cur_z, 2);
    Serial.println();

}


void parseCommand(String input) {
  // Check if the message starts with "AK80,"
  if (input.startsWith("AK80,")) {
    commandReceived = true;
    input.remove(0, 5); // Remove "AK80," from the input string

    // Split the string into three parts using commas as separators
    int firstComma = input.indexOf(',');
    int secondComma = input.indexOf(',', firstComma + 1);

    if (firstComma == -1 || secondComma == -1) {
      Serial.println("Invalid command format.");
      return;
    }

    String cmd0 = input.substring(0, firstComma);
    String cmd1 = input.substring(firstComma + 1, secondComma);
    String cmd2 = input.substring(secondComma + 1);

    cmd_x = (cmd0.toFloat() * 360) / (PI * WHEEL_DIAMETER);
    cmd_y = (cmd1.toFloat() * 360) / (PI * WHEEL_DIAMETER);
    cmd_z = (cmd2.toFloat() * 360) / (PI * 0.2);

  } else if (input.startsWith("POLO,")) {
    commandReceived = true;
    input.remove(0, 5);

    int poloCommand = input.toInt();

    switch (poloCommand) {
      case 0:
        // Serial.println("Chassis stable");
        chassis_stable();
        break;

      case 1:
        // Serial.println("Chassis change to X");
        chassis_x();
        break;

      case 2:
        // Serial.println("Chassis change to Y");
        chassis_y();
        break;
    }

  } else {
    Serial.println("Invalid command prefix.");
  }
}

void power_on(uint16_t motor_id) {
    struct can_frame canMsg;
    canMsg.can_id = motor_id;
    canMsg.can_dlc = 8;
    canMsg.data[0] = 0xFF;
    canMsg.data[1] = 0xFF;
    canMsg.data[2] = 0xFF;
    canMsg.data[3] = 0xFF;
    canMsg.data[4] = 0xFF;
    canMsg.data[5] = 0xFF;
    canMsg.data[6] = 0xFF;
    canMsg.data[7] = 0xFC;

    // Send the message over CAN
    if (chassis.mcp2515.sendMessage(&canMsg) != MCP2515::ERROR_OK) {
        Serial.print("Error powering on motor with ID: ");
        Serial.println(motor_id);
    } else {
        Serial.print("Powered on motor with ID: ");
        Serial.println(motor_id);
    }
}


void setup() {
    Serial.begin(19200);
    while (!Serial) {};
    chassis.initializeCAN();
    lift.initializeCAN();

    power_on(X_CONTROL);
    power_on(Y_CONTROL);
    power_on(Z_CONTROL);

    chassis.set_origin(X_CONTROL, 1);
    lift.set_origin(Z_CONTROL, 1);
    chassis.set_origin(Y_CONTROL, 1);
}

unsigned long previousMillis = 0;  // Stores the last time Serial was checked
const unsigned long interval = 10; // Interval in milliseconds

void loop() {
    if (chassis.mcp2515.readMessage(&chassis.canMsg2) == MCP2515::ERROR_OK) {
        canid_t can_id = chassis.canMsg2.can_id;
        MotorData data;
        // data.position = ((chassis.canMsg2.data[0] << 8) | chassis.canMsg2.data[1]) * 0.1;
        data.position = (((chassis.canMsg2.data[0] << 8) | chassis.canMsg2.data[1]) * 0.1 * PI * WHEEL_DIAMETER) / 360;
        data.speed = ((((chassis.canMsg2.data[2] << 8) | chassis.canMsg2.data[3]) * 10) / (POLE_PAIRS * CHASSIS_REDUCTION_RATIO)) * ((2*PI*WHEEL_DIAMETER)/60);
        //ERPM
        // data.current = ((chassis.canMsg2.data[4] << 8) | chassis.canMsg2.data[5]) * 0.01;
        // qaxis current
        data.current = ((chassis.canMsg2.data[4] << 8) | chassis.canMsg2.data[5]) * 0.01 * Kt_CHASSIS * CHASSIS_REDUCTION_RATIO;
        // output torque = output_current * kt* reduction_ratio
        data.motorTemp = chassis.canMsg2.data[6];
        data.errorCode = chassis.canMsg2.data[7];
        motorReadings[can_id] = data;

        
    }


    if (lift.mcp2515.readMessage(&lift.canMsg2) == MCP2515::ERROR_OK) {
        canid_t can_id = lift.canMsg2.can_id;
        MotorData data;
        data.position = (((lift.canMsg2.data[0] << 8) | lift.canMsg2.data[1]) * 0.1 * PI * 0.2)/360;
        data.speed = ((((lift.canMsg2.data[2] << 8) | lift.canMsg2.data[3]) * 10) / (POLE_PAIRS * LIFT_REDUCTION_RATIO)) * ((2*PI*0.2)/60);
        // data.current = ((lift.canMsg2.data[4] << 8) | lift.canMsg2.data[5]) * 0.01;
        data.current = ((chassis.canMsg2.data[4] << 8) | chassis.canMsg2.data[5]) * 0.01 * Kt_LIFT * LIFT_REDUCTION_RATIO;
        data.motorTemp = lift.canMsg2.data[6];
        data.errorCode = lift.canMsg2.data[7];
        motorReadings[can_id] = data;

    }    

    sendMotorData();


    // Check for available serial data
    if (Serial.available()) {
        String input = Serial.readStringUntil('\n');
        // Serial.println(input);

        parseCommand(input);

        if (commandReceived == true) {
            chassis.set_pos_spd(X_CONTROL, cmd_x, 3000, 3000); 
            lift.set_pos_spd(Z_CONTROL, cmd_z, 5000, 5000);
            chassis.set_pos_spd(Y_CONTROL, cmd_y, 3000, 3000);

            commandReceived = false;
        }
    }
}