/**
 * @file CubemarsAK.cpp
 * @author Samor / Gemini CLI
 * @brief Professional implementation of the Cubemars AK-series library.
 */

#include "CubemarsAK.h"

/**
 * @brief Constructor.
 */
CubemarsAK::CubemarsAK(uint8_t csPin) : mcp2515(csPin) {}

/**
 * @brief Destructor.
 */
CubemarsAK::~CubemarsAK() {}

/**
 * @brief Initializes the CAN controller with production settings.
 * Bitrate: 1000kbps, Clock: 8MHz.
 */
void CubemarsAK::initializeCAN() noexcept {
    mcp2515.reset();
    mcp2515.setBitrate(CAN_1000KBPS, MCP_8MHZ);
    mcp2515.setNormalMode();
    Serial.println(F("[AK-LIB] CAN BUS 1Mbps / 8MHz Ready"));
}

/**
 * @brief Compiles the 29-bit Extended ID required by the VESC-based firmware.
 */
uint32_t CubemarsAK::buildCanId(uint8_t id, AKMode mode) const noexcept {
    return static_cast<uint32_t>(id | (static_cast<uint8_t>(mode) << 8));
}

/**
 * @brief High-reliability transmission routine.
 */
void CubemarsAK::transmit(uint32_t id, const uint8_t* data, uint8_t len) noexcept {
    canMsg.can_id = id | CAN_EFF_FLAG; // Extended Frame Flag is mandatory
    canMsg.can_dlc = len;
    
    // Memory safe copy to local buffer
    if (data && len <= 8) {
        memcpy(canMsg.data, data, len);
    }

    const MCP2515::ERROR status = mcp2515.sendMessage(MCP2515::TXB1, &canMsg);
    if (status != MCP2515::ERROR_OK) {
        Serial.print(F("[AK-LIB] TX Error: "));
        Serial.println(static_cast<int>(status));
    }
}

/**
 * @brief Convert mechanical shaft RPM to electrical ERPM.
 * Logic: Speed_Mech * Gear_Ratio * Pole_Pairs
 */
int32_t CubemarsAK::mechRpmToErpm(float mechRpm) const noexcept {
    return static_cast<int32_t>(mechRpm * AK40_10_GEAR_RATIO * AK40_10_POLE_PAIRS);
}

// Byte Packing Helpers

/**
 * @brief Big-endian serialization for 32-bit integers.
 */
void CubemarsAK::appendInt32(uint8_t* buffer, int32_t val, int32_t* index) noexcept {
    buffer[(*index)++] = static_cast<uint8_t>(val >> 24);
    buffer[(*index)++] = static_cast<uint8_t>(val >> 16);
    buffer[(*index)++] = static_cast<uint8_t>(val >> 8);
    buffer[(*index)++] = static_cast<uint8_t>(val);
}

/**
 * @brief Big-endian serialization for 16-bit integers.
 */
void CubemarsAK::appendInt16(uint8_t* buffer, int16_t val, int16_t* index) noexcept {
    buffer[(*index)++] = static_cast<uint8_t>(val >> 8);
    buffer[(*index)++] = static_cast<uint8_t>(val);
}

// Motor Control Commands

void CubemarsAK::set_duty(uint8_t id, float duty) noexcept {
    int32_t idx = 0;
    uint8_t buf[4];
    duty = constrain(duty, -1.0f, 1.0f);
    appendInt32(buf, static_cast<int32_t>(duty * 100000.0f), &idx);
    transmit(buildCanId(id, AK_PWM), buf, 4);
}

void CubemarsAK::set_current(uint8_t id, float current) noexcept {
    current = constrain(current, -AK40_10_MAX_CURRENT, AK40_10_MAX_CURRENT);
    int32_t idx = 0;
    uint8_t buf[4];
    appendInt32(buf, static_cast<int32_t>(current * 1000.0f), &idx);
    transmit(buildCanId(id, AK_CURRENT), buf, 4);
}

void CubemarsAK::set_cb(uint8_t id, float current) noexcept {
    current = constrain(current, 0, AK40_10_MAX_CURRENT);
    int32_t idx = 0;
    uint8_t buf[4];
    appendInt32(buf, static_cast<int32_t>(current * 1000.0f), &idx);
    transmit(buildCanId(id, AK_CURRENT_BRAKE), buf, 4);
}

void CubemarsAK::set_spd(uint8_t id, float rpm) noexcept {
    int32_t idx = 0;
    uint8_t buf[4];
    appendInt32(buf, mechRpmToErpm(rpm), &idx);
    transmit(buildCanId(id, AK_VELOCITY), buf, 4);
}

void CubemarsAK::set_pos(uint8_t id, float pos) noexcept {
    int32_t idx = 0;
    uint8_t buf[4];
    appendInt32(buf, static_cast<int32_t>(pos * 10000.0f), &idx);
    transmit(buildCanId(id, AK_POSITION), buf, 4);
}

void CubemarsAK::set_origin(uint8_t id, uint8_t mode) noexcept {
    uint8_t buf[1] = {mode};
    transmit(buildCanId(id, AK_ORIGIN), buf, 1);
}

void CubemarsAK::set_pos_spd(uint8_t id, float pos, int16_t spd, int16_t rpa) noexcept {
    int32_t idx32 = 0;
    int16_t idx16 = 4;
    uint8_t buf[8];
    
    // Pos scale: 0.0001 deg/LSB
    appendInt32(buf, static_cast<int32_t>(pos * 10000.0f), &idx32);
    
    // Spd/Accel scale: 1 ERPM / LSB
    appendInt16(buf, static_cast<int16_t>(mechRpmToErpm(static_cast<float>(spd))), &idx16);
    appendInt16(buf, static_cast<int16_t>(mechRpmToErpm(static_cast<float>(rpa))), &idx16);
    
    transmit(buildCanId(id, AK_POSITION_VELOCITY), buf, 8);
}

// Feedback Processing

/**
 * @brief Batch-processes the CAN RX FIFO.
 * Decodes standard feedback frames (0.1 deg, 10 RPM, 0.01A scales).
 */
void CubemarsAK::updateFeedback() noexcept {
    while (mcp2515.readMessage(&canMsg) == MCP2515::ERROR_OK) {
        // Enforce EFF validation
        if (!(canMsg.can_id & CAN_EFF_FLAG)) {
            continue;
        }

        const uint8_t id = static_cast<uint8_t>(canMsg.can_id & 0xFF);
        MotorData data;

        // Position Decoding (0.1 deg / LSB)
        const int16_t p_raw = static_cast<int16_t>((canMsg.data[0] << 8) | canMsg.data[1]);
        data.position = static_cast<float>(p_raw) * 0.1f;

        // Speed Decoding (10 RPM / LSB Mechanical)
        const int16_t v_raw = static_cast<int16_t>((canMsg.data[2] << 8) | canMsg.data[3]);
        data.speed = static_cast<float>(v_raw) * 10.0f;

        // Current Decoding (0.01 A / LSB)
        const int16_t i_raw = static_cast<int16_t>((canMsg.data[4] << 8) | canMsg.data[5]);
        data.current = static_cast<float>(i_raw) * 0.01f;

        data.motorTemp = static_cast<int8_t>(canMsg.data[6]);
        data.errorCode = canMsg.data[7];

        // Store in database
        _motors[id] = data;
    }
}

// Const Correct Getters

float CubemarsAK::getPosition(uint8_t id) const noexcept {
    auto it = _motors.find(id);
    return (it != _motors.end()) ? it->second.position : 0.0f;
}

float CubemarsAK::getSpeed(uint8_t id) const noexcept {
    auto it = _motors.find(id);
    return (it != _motors.end()) ? it->second.speed : 0.0f;
}

float CubemarsAK::getCurrent(uint8_t id) const noexcept {
    auto it = _motors.find(id);
    return (it != _motors.end()) ? it->second.current : 0.0f;
}

int8_t CubemarsAK::getMotorTemp(uint8_t id) const noexcept {
    auto it = _motors.find(id);
    return (it != _motors.end()) ? it->second.motorTemp : 0;
}

uint8_t CubemarsAK::getErrorCode(uint8_t id) const noexcept {
    auto it = _motors.find(id);
    return (it != _motors.end()) ? it->second.errorCode : 0;
}