#include <Arduino.h>
#include <mcp2515.h>
#include <SPI.h>
#include <CubemarsAK.h>

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


void sendMotorData(){
    pos_x = getPosition(X_DATA);
    vel_x = getSpeed(X_DATA);
    cur_x = getCurrent(X_DATA);

    pos_y = getPosition(Y_DATA);
    vel_y = getSpeed(Y_DATA);
    cur_y = getCurrent(Y_DATA);

    pos_z = getPosition(Z_DATA);
    vel_z = getSpeed(Z_DATA);
    cur_z = getCurrent(Z_DATA);

    String dataPacket = "SEND, ";
    dataPacket += String(pos_x, 2) + ", " + String(vel_x, 2) + ", " + String(cur_x, 2) + ", ";
    dataPacket += String(pos_y, 2) + ", " + String(vel_y, 2) + ", " + String(cur_y, 2) + ", ";
    dataPacket += String(pos_z, 2) + ", " + String(vel_z, 2) + ", " + String(cur_z, 2) + "\n";

    Serial.print(dataPacket);
}

void parseCommand(String input) {
  // Check if the message starts with "RECV,"
  if (input.startsWith("RECV,")) {
    commandReceived = true;
    input.remove(0, 5); // Remove "RECV," from the input string

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

    cmd_x = (cmd0.toFloat()*360)/(PI*WHEEL_DIAMETER);
    cmd_y = (cmd1.toFloat()*360)/(PI*WHEEL_DIAMETER);
    cmd_z = cmd2.toFloat();
  } else {
    Serial.println("Invalid command prefix.");
  }
}

void setup() {
    Serial.begin(115200);
    while (!Serial) {};

    chassis.initializeCAN();
    lift.initializeCAN();

}



void loop() {
    if (chassis.mcp2515.readMessage(&chassis.canMsg2) == MCP2515::ERROR_OK) {
        canid_t can_id = chassis.canMsg2.can_id;
        MotorData data;
        data.position = ((chassis.canMsg2.data[0] << 8) | chassis.canMsg2.data[1]) * 0.1;
        // data.position = (((chassis.canMsg2.data[0] << 8) | chassis.canMsg2.data[1]) * 0.1 * PI * WHEEL_DIAMETER) / 360;
        data.speed = (((chassis.canMsg2.data[2] << 8) | chassis.canMsg2.data[3]) * 10) / (POLE_PAIRS * CHASSIS_REDUCTION_RATIO);
        data.current = ((chassis.canMsg2.data[4] << 8) | chassis.canMsg2.data[5]) * 0.01;
        // data.current = ((chassis.canMsg2.data[4] << 8) | chassis.canMsg2.data[5]) * 0.01 * Kt_CHASSIS * CHASSIS_REDUCTION_RATIO;
        // output torque = output_current * kt* reduction_ratio
        data.motorTemp = chassis.canMsg2.data[6];
        data.errorCode = chassis.canMsg2.data[7];
        motorReadings[can_id] = data;
    }

    if (lift.mcp2515.readMessage(&lift.canMsg2) == MCP2515::ERROR_OK) {
        canid_t can_id = lift.canMsg2.can_id;
        MotorData data;
        data.position = ((lift.canMsg2.data[0] << 8) | lift.canMsg2.data[1]) * 0.1;
        data.speed = (((lift.canMsg2.data[2] << 8) | lift.canMsg2.data[3]) * 10) / (POLE_PAIRS * LIFT_REDUCTION_RATIO);
        data.current = ((lift.canMsg2.data[4] << 8) | lift.canMsg2.data[5]) * 0.01;
        // data.current = ((chassis.canMsg2.data[4] << 8) | chassis.canMsg2.data[5]) * 0.01 * Kt_LIFT * LIFT_REDUCTION_RATIO;
        data.motorTemp = lift.canMsg2.data[6];
        data.errorCode = lift.canMsg2.data[7];
        motorReadings[can_id] = data;
    }    
    sendMotorData();

    if(Serial.available() > 0){
        String input = Serial.readStringUntil('\n');
        parseCommand(input); 
        if(commandReceived == true){
            chassis.set_pos_spd(X_CONTROL, cmd_x, 3000, 1000);
            lift.set_pos_spd(Z_CONTROL, cmd_z, 2000, 1000);
            chassis.set_pos_spd(Y_CONTROL, cmd_y, 3000, 1000);
            commandReceived = false;
        }
    }
}