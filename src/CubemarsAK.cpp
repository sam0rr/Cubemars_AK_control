/**
 * @file CubemarsAK.cpp
 * @author Samor / Gemini CLI
 * @brief Library for Cubemars AK-series motors (Servo Mode).
 * @details High-performance, memory-safe implementation for Cubemars AK-series
 * motor controllers. Supports polymorphic hardware configurations and multi-motor
 * telemetry processing on a single CAN bus.
 * @version 1.0
 * @date 2026-02-04
 */

#include "CubemarsAK.h"

/**
 * @brief Initialize hardware reference.
 * @param csPin SPI Chip Select for MCP2515.
 */
CubemarsAK::CubemarsAK(uint8_t csPin) : mcp2515(csPin) {}

/**
 * @brief Destructor.
 */
CubemarsAK::~CubemarsAK() {}

/**
 * @brief Configures MCP2515 hardware.
 * @details Establishes 1Mbps communication with 8MHz oscillator.
 * @return void
 */
void CubemarsAK::initializeCAN() noexcept {
    mcp2515.reset();
    mcp2515.setBitrate(CAN_1000KBPS, MCP_8MHZ);
    mcp2515.setNormalMode();
    Serial.println(F("[AK-LIB] CAN BUS 1Mbps / 8MHz Ready"));
}

/**
 * @brief Registers a motor ID with its specific hardware configuration.
 * @param id The CAN ID of the motor.
 * @param config The hardware parameters.
 */
void CubemarsAK::attachMotor(uint8_t id, const MotorConfig& config) noexcept {
    _configs[id] = config;
    Serial.print(F("[AK-LIB] Attached Motor ID "));
    Serial.print(id);
    Serial.print(F(" (Ratio: "));
    Serial.print(config.gearRatio);
    Serial.println(F(")"));
}

/**
 * @brief Compiles 29-bit Extended ID.
 * @param id Controller ID (8-bit).
 * @param mode VESC-style protocol mode.
 * @return uint32_t Formatted CAN ID.
 */
uint32_t CubemarsAK::buildCanId(uint8_t id, AKMode mode) const noexcept {
    return static_cast<uint32_t>(id | (static_cast<uint8_t>(mode) << 8));
}

/**
 * @brief Atomic CAN transmission routine.
 * @param id Formatted Extended ID.
 * @param data Byte array (max 8 bytes).
 * @param len Data length.
 */
void CubemarsAK::transmit(uint32_t id, const uint8_t* data, uint8_t len) noexcept {
    canMsg.can_id = id | CAN_EFF_FLAG;
    canMsg.can_dlc = len;

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
 * @brief Scaler for mechanical to electrical velocity based on motor config.
 * @param id Motor CAN ID.
 * @param mechRpm Mechanical shaft RPM.
 * @return int32_t Electrical RPM (ERPM). Defaults to 1:1 if ID not attached.
 */
int32_t CubemarsAK::mechRpmToErpm(uint8_t id, float mechRpm) const noexcept {
    if (_configs.count(id)) {
        const auto& cfg = _configs.at(id);
        return static_cast<int32_t>(mechRpm * cfg.gearRatio * cfg.polePairs);
    }
    return static_cast<int32_t>(mechRpm);
}

/**
 * @brief Serialize int32 to big-endian buffer.
 * @param buffer Target array.
 * @param val Value to pack.
 * @param index Write pointer.
 */
void CubemarsAK::appendInt32(uint8_t* buffer, int32_t val, int32_t* index) noexcept {
    buffer[(*index)++] = static_cast<uint8_t>(val >> 24);
    buffer[(*index)++] = static_cast<uint8_t>(val >> 16);
    buffer[(*index)++] = static_cast<uint8_t>(val >> 8);
    buffer[(*index)++] = static_cast<uint8_t>(val);
}

/**
 * @brief Serialize int16 to big-endian buffer.
 * @param buffer Target array.
 * @param val Value to pack.
 * @param index Write pointer.
 */
void CubemarsAK::appendInt16(uint8_t* buffer, int16_t val, int16_t* index) noexcept {
    buffer[(*index)++] = static_cast<uint8_t>(val >> 8);
    buffer[(*index)++] = static_cast<uint8_t>(val);
}

/**
 * @brief Dispatch Mode 0: Duty Cycle.
 * @param id Motor ID.
 * @param duty Range [-1.0, 1.0].
 */
void CubemarsAK::set_duty(uint8_t id, float duty) noexcept {
    int32_t idx = 0;
    uint8_t buf[4];
    duty = constrain(duty, -1.0f, 1.0f);
    appendInt32(buf, static_cast<int32_t>(duty * 100000.0f), &idx);
    transmit(buildCanId(id, AK_PWM), buf, 4);
}

/**
 * @brief Dispatch Mode 1: Current Control.
 * @param id Motor ID.
 * @param current Amperes (clamped to safety max).
 */
void CubemarsAK::set_current(uint8_t id, float current) noexcept {
    float limit = _configs.count(id) ? _configs.at(id).maxCurrent : 10.0f;
    current = constrain(current, -limit, limit);
    int32_t idx = 0;
    uint8_t buf[4];
    appendInt32(buf, static_cast<int32_t>(current * 1000.0f), &idx);
    transmit(buildCanId(id, AK_CURRENT), buf, 4);
}

/**
 * @brief Dispatch Mode 2: Regenerative Braking.
 * @param id Motor ID.
 * @param current Brake current in Amperes.
 */
void CubemarsAK::set_cb(uint8_t id, float current) noexcept {
    float limit = _configs.count(id) ? _configs.at(id).maxCurrent : 10.0f;
    current = constrain(current, 0, limit);
    int32_t idx = 0;
    uint8_t buf[4];
    appendInt32(buf, static_cast<int32_t>(current * 1000.0f), &idx);
    transmit(buildCanId(id, AK_CURRENT_BRAKE), buf, 4);
}

/**
 * @brief Dispatch Mode 3: Velocity Control.
 * @param id Motor ID.
 * @param rpm Mechanical RPM.
 */
void CubemarsAK::set_spd(uint8_t id, float rpm) noexcept {
    int32_t idx = 0;
    uint8_t buf[4];
    appendInt32(buf, mechRpmToErpm(id, rpm), &idx);
    transmit(buildCanId(id, AK_VELOCITY), buf, 4);
}

/**
 * @brief Dispatch Mode 4: Position Control.
 * @param id Motor ID.
 * @param pos Absolute Degrees.
 */
void CubemarsAK::set_pos(uint8_t id, float pos) noexcept {
    int32_t idx = 0;
    uint8_t buf[4];
    appendInt32(buf, static_cast<int32_t>(pos * 10000.0f), &idx);
    transmit(buildCanId(id, AK_POSITION), buf, 4);
}

/**
 * @brief Dispatch Mode 5: Origin Calibration.
 * @param id Motor ID.
 * @param mode Calibration opcode.
 */
void CubemarsAK::set_origin(uint8_t id, uint8_t mode) noexcept {
    uint8_t buf[1] = {mode};
    transmit(buildCanId(id, AK_ORIGIN), buf, 1);
}

/**
 * @brief Dispatch Mode 6: Position-Velocity Trajectory.
 * @param id Motor ID.
 * @param pos Final Degrees.
 * @param spd Mechanical RPM limit.
 * @param rpa Mechanical Accel limit.
 */
void CubemarsAK::set_pos_spd(uint8_t id, float pos, int16_t spd, int16_t rpa) noexcept {
    int32_t idx32 = 0;
    int16_t idx16 = 4;
    uint8_t buf[8];
    appendInt32(buf, static_cast<int32_t>(pos * 10000.0f), &idx32);
    appendInt16(buf, static_cast<int16_t>(mechRpmToErpm(id, static_cast<float>(spd))), &idx16);
    appendInt16(buf, static_cast<int16_t>(mechRpmToErpm(id, static_cast<float>(rpa))), &idx16);
    transmit(buildCanId(id, AK_POSITION_VELOCITY), buf, 8);
}

/**
 * @brief Ingest pending CAN frames.
 * @details Filters for EFF and maps data by ID using standardized scaling factors.
 */
void CubemarsAK::updateFeedback() noexcept {
    while (mcp2515.readMessage(&canMsg) == MCP2515::ERROR_OK) {
        if (!(canMsg.can_id & CAN_EFF_FLAG)) {
            continue;
        }

        const uint8_t id = static_cast<uint8_t>(canMsg.can_id & 0xFF);
        MotorData data;

        const int16_t p_raw = static_cast<int16_t>((canMsg.data[0] << 8) | canMsg.data[1]);
        data.position = static_cast<float>(p_raw) * 0.1f;

        const int16_t v_raw = static_cast<int16_t>((canMsg.data[2] << 8) | canMsg.data[3]);
        data.speed = static_cast<float>(v_raw) * 10.0f;

        const int16_t i_raw = static_cast<int16_t>((canMsg.data[4] << 8) | canMsg.data[5]);
        data.current = static_cast<float>(i_raw) * 0.01f;

        data.motorTemp = static_cast<int8_t>(canMsg.data[6]);
        data.errorCode = canMsg.data[7];

        _motors[id] = data;
    }
}

/**
 * @brief Retrieve position for specified ID.
 * @param id Motor ID.
 * @return float Degrees.
 */
float CubemarsAK::getPosition(uint8_t id) const noexcept {
    auto it = _motors.find(id);
    return (it != _motors.end()) ? it->second.position : 0.0f;
}

/**
 * @brief Retrieve speed for specified ID.
 * @param id Motor ID.
 * @return float Mechanical RPM.
 */
float CubemarsAK::getSpeed(uint8_t id) const noexcept {
    auto it = _motors.find(id);
    return (it != _motors.end()) ? it->second.speed : 0.0f;
}

/**
 * @brief Retrieve current for specified ID.
 * @param id Motor ID.
 * @return float Amperes.
 */
float CubemarsAK::getCurrent(uint8_t id) const noexcept {
    auto it = _motors.find(id);
    return (it != _motors.end()) ? it->second.current : 0.0f;
}

/**
 * @brief Retrieve temperature for specified ID.
 * @param id Motor ID.
 * @return int8_t Celsius.
 */
int8_t CubemarsAK::getMotorTemp(uint8_t id) const noexcept {
    auto it = _motors.find(id);
    return (it != _motors.end()) ? it->second.motorTemp : 0;
}

/**
 * @brief Retrieve error code for specified ID.
 * @param id Motor ID.
 * @return uint8_t Error status.
 */
uint8_t CubemarsAK::getErrorCode(uint8_t id) const noexcept {
    auto it = _motors.find(id);
    return (it != _motors.end()) ? it->second.errorCode : 0;
}
