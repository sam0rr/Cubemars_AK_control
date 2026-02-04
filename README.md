# CubeMars AK80-9 Motor Control with Arduino (ESP32) and MCP2515 CAN Module

## Overview

Code to control the CubeMars AK80-9 motor using an Arduino (ESP32) and the MCP2515 CAN module. The motor supports various control modes such as position control, velocity control, and position-velocity hybrid control. The code also includes functions to read motor parameters like position, speed, current, temperature, and error codes via CAN communication.

## Equipment

- **CubeMars AK80-9 motor** (30-45V power input)
- **ESP32 WROVER-KIT** (controller)
- **MCP2515 CAN module** (to handle CAN communication)
- **Power supply** (30-45V for motor)
- CAN bus connection cables
- SPI cables for connection between ESP32 and MCP2515 module

## Wiring

### MCP2515 Module to ESP32 WROVER Pin Connections

| MCP2515 Pin   | ESP32 Pin (Default VSPI) | Description                                    |
| :------------ | :----------------------- | :--------------------------------------------- |
| **VCC**       | **5V**                   | Power for the MCP2515 (Must be 5V for TJA1050) |
| **GND**       | **GND**                  | Ground                                         |
| **CS**        | **GPIO 14**              | Chip Select (Configured in main.cpp)           |
| **SO (MISO)** | **GPIO 19**              | Master In Slave Out                            |
| **SI (MOSI)** | **GPIO 23**              | Master Out Slave In                            |
| **SCK**       | **GPIO 18**              | Serial Clock                                   |
| **INT**       | **GPIO 4**               | Interrupt (Optional)                           |

### Motor to Power Supply

- Connect the CubeMars AK80-9 motor to the 30-45V power supply.

### CAN Bus Wiring

- Connect the CAN High (CANH) and CAN Low (CANL) lines between the motor and MCP2515 module.

## Software Setup

### Library Requirements

Install the following libraries in your Arduino IDE:
[mcp2515 library](https://github.com/autowp/arduino-mcp2515) - For controlling the MCP2515 CAN module.

### PlatformIO Configuration (`platformio.ini`)

```ini
[env:esp32-wrover]
platform = espressif32
board = esp-wrover-kit
framework = arduino
monitor_speed = 115200
lib_deps =
    autowp/autowp-mcp2515 @ ^1.2.1
    pololu/VL53L0X @ ^1.3.1
    pololu/JrkG2 @ ^1.1.0
    Wire
```

### Motor Control Functions

The motor supports several control modes via CAN commands. Below are the main functions used for controlling the motor.

#### 1. **Position Control** (`set_pos()`)

This function sets the motor to position control mode. You specify a controller id and target position, and the motor will rotate to that position.

#### 2. **Velocity Control** (`set_spd()`)

This function sets the motor to velocity control mode. You specify a controller id and target speed, and the motor will attempt to maintain that speed.

#### 3. **Position-Velocity Control** (`pos_spd()`)

This function combines position and velocity control, allowing you to set both a target position and a maximum velocity for reaching that position.
