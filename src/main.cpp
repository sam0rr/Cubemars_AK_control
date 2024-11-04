#include <Arduino.h>
#include <mcp2515.h>
#include <SPI.h>
#include <CubemarsAK.h>

#define x_control 104
#define x_data 2147494248
#define y_control 105
#define y_data 2147494249
#define z_control 106
#define z_data 2147494250

CubemarsAK chassis(10);
// CubemarsAK lift(9);

float pos_x, vel_x, cur_x = 0.0;
float pos_y, vel_y, cur_y = 0.0;
// float pos_z, vel_z, cur_z = 0.0;
float cmd_x, cmd_y, cmd_z = 0.0;
bool commandReceived = false;


void sendMotorData(){
    pos_x = chassis.getPosition(x_data);
    vel_x = chassis.getSpeed(x_data);
    cur_x = chassis.getCurrent(x_data);

    pos_y = chassis.getPosition(y_data);
    vel_y = chassis.getSpeed(y_data);
    cur_y = chassis.getCurrent(y_data);

    // pos_z = chassis.getPosition(z_data);
    // vel_z = chassis.getSpeed(z_data);
    // cur_z = chassis.getCurrent(z_data);

    String dataPacket = "SEND ";
    dataPacket += String(pos_x, 2) + ", " + String(vel_x, 2) + ", " + String(cur_x, 2) + ", ";
    dataPacket += String(pos_y, 2) + ", " + String(vel_y, 2) + ", " + String(cur_y, 2) + "\n";
    // dataPacket += String(pos_z, 2) + ", " + String(vel_z, 2) + ", " + String(cur_z, 2) + "\n";

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

    // Extract each command part
    String cmd0 = input.substring(0, firstComma);
    String cmd1 = input.substring(firstComma + 1, secondComma);
    String cmd2 = input.substring(secondComma + 1);

    // Convert to float or integer if needed
    cmd_x = cmd0.toFloat();
    cmd_y = cmd1.toFloat();
    cmd_z = cmd2.toFloat();
  } else {
    Serial.println("Invalid command prefix.");
  }
}

void setup() {
    Serial.begin(115200);
    while (!Serial);

    chassis.initializeCAN();
    // lift.initializeCAN();

}



void loop() {
    chassis.unpackServo();
    // lift.unpackServo();
    
    sendMotorData();

    if(Serial.available() > 0){
        String input = Serial.readStringUntil('\n'); // Read until newline character
        parseCommand(input); // Parse and print the values
        if(commandReceived == true){
            if(!((chassis.getPosition(x_data) == cmd_x) && (chassis.getPosition(y_data) == cmd_x))){
                chassis.comm_can_set_pos_spd(x_control, cmd_x, 2000, 1000);
                chassis.comm_can_set_pos_spd(y_control, cmd_x, 2000, 1000);
                // lift.comm_can_set_pos_spd(z_control, cmd_x, 2000, 1000);
            }
        }
    }

}