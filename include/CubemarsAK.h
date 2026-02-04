/**
 * @file CubemarsAK.h
 * @author Samor / Gemini CLI
 * @brief Professional-grade library for Cubemars AK-series motors (Servo Mode).
 * @details This library implements the Cubemars Servo Mode CAN protocol, specifically
 * optimized for the AK40-10 KV170 motor. It handles multi-motor telemetry
 * using a map-based internal database and ensures hardware safety through
 * rigorous input clamping.
 * @version 1.1
 * @date 2026-02-04
 */

#ifndef CUBEMARSAK_H
#define CUBEMARSAK_H

#include <Arduino.h>
#include <SPI.h>
#include <mcp2515.h>
#include <map>

/**
 * @brief Structure to store the physical state of a motor.
 * Units are standardized to SI or common engineering units.
 */
struct MotorData {
    float position;    /**< Mechanical position in Degrees [deg] */
    float speed;       /**< Mechanical velocity in Revolutions Per Minute [RPM] */
    float current;     /**< Motor phase current in Amperes [A] */
    int8_t motorTemp;  /**< Controller/Motor temperature in Celsius [°C] */
    uint8_t errorCode; /**< Status code (0x00 = No Error) */
};

/**
 * @brief Extended CAN IDs for Cubemars AK Series (Servo Mode Firmware).
 * The Frame ID is constructed as: [Mode (8-bits) << 8 | Controller ID (8-bits)]
 */
enum AKMode : uint8_t {
    AK_PWM = 0,               /**< Duty Cycle Mode (Command range: -1.0 to 1.0) */
    AK_CURRENT = 1,           /**< Current Loop Control (Command unit: Amperes) */
    AK_CURRENT_BRAKE = 2,     /**< Brake Mode (Command unit: Amperes) */
    AK_VELOCITY = 3,          /**< Velocity Control (Command unit: Mechanical RPM) */
    AK_POSITION = 4,          /**< Position Control (Command unit: Degrees) */
    AK_ORIGIN = 5,            /**< Set Home/Zero Position commands */
    AK_POSITION_VELOCITY = 6, /**< Trajectory Control (Pos + Speed + Accel) */
};

/** 
 * @name AK40-10 Physical Constants
 * @{ 
 */
#define AK40_10_GEAR_RATIO  10.0f  /**< Internal planetary reduction ratio */
#define AK40_10_POLE_PAIRS  7      /**< Magnetic pole pairs (14 poles) */
#define AK40_10_MAX_CURRENT 35.0f  /**< Absolute safety current limit (Amps) */
/** @} */

/**
 * @class CubemarsAK
 * @brief High-level hardware abstraction for Cubemars AK-series motor controllers.
 * 
 * @note This class is non-copyable to prevent SPI bus contention or hardware state conflicts.
 */
class CubemarsAK {
public:
    /**
     * @brief Construct a new Cubemars AK controller instance.
     * @param csPin SPI Chip Select pin connected to the MCP2515.
     */
    explicit CubemarsAK(uint8_t csPin);
    
    /**
     * @brief Standard destructor.
     */
    ~CubemarsAK();

    // Prevent copying of hardware resources
    CubemarsAK(const CubemarsAK&) = delete;
    CubemarsAK& operator=(const CubemarsAK&) = delete;

    /**
     * @brief Initializes the MCP2515 CAN controller.
     * @details Sets bitrate to 1Mbps and crystal frequency to 8MHz.
     */
    void initializeCAN() noexcept;

    // COMMAND INTERFACE

    /**
     * @brief Sets the motor duty cycle (Mode 0).
     * @param id Controller CAN ID.
     * @param duty Normalized value between -1.0 and 1.0.
     */
    void set_duty(uint8_t id, float duty) noexcept;

    /**
     * @brief Sets target phase current (Mode 1).
     * @param id Controller CAN ID.
     * @param current Target current in Amperes (Clamped to +/- AK40_10_MAX_CURRENT).
     */
    void set_current(uint8_t id, float current) noexcept;

    /**
     * @brief Applies a braking current (Mode 2).
     * @param id Controller CAN ID.
     * @param current Brake current in Amperes (Positive only).
     */
    void set_cb(uint8_t id, float current) noexcept;

    /**
     * @brief Sets target mechanical speed (Mode 3).
     * @param id Controller CAN ID.
     * @param rpm Target output shaft velocity in RPM.
     */
    void set_spd(uint8_t id, float rpm) noexcept;

    /**
     * @brief Sets target mechanical position (Mode 4).
     * @param id Controller CAN ID.
     * @param pos Absolute position target in Degrees.
     */
    void set_pos(uint8_t id, float pos) noexcept;

    /**
     * @brief Configures the encoder origin (Mode 5).
     * @param id Controller CAN ID.
     * @param mode 0: Set Temp Origin, 1: Set Permanent Zero, 2: Restore Defaults.
     */
    void set_origin(uint8_t id, uint8_t mode) noexcept;

    /**
     * @brief Executes a trapezoidal trajectory (Mode 6).
     * @param id Controller CAN ID.
     * @param pos Final position in Degrees.
     * @param spd Maximum cruise speed in Mechanical RPM.
     * @param rpa Acceleration rate in Mechanical RPM/s.
     */
    void set_pos_spd(uint8_t id, float pos, int16_t spd, int16_t rpa) noexcept;

    // TELEMETRY INTERFACE

    /**
     * @brief Processes all pending CAN messages.
     * @details Updates the internal telemetry database (_motors map).
     */
    void updateFeedback() noexcept;

    /** @name Data Getters (Const Correct) */
    /** @{ */
    float getPosition(uint8_t id) const noexcept;
    float getSpeed(uint8_t id) const noexcept;
    float getCurrent(uint8_t id) const noexcept;
    int8_t getMotorTemp(uint8_t id) const noexcept;
    uint8_t getErrorCode(uint8_t id) const noexcept;
    /** @} */

    MCP2515 mcp2515;         /**< Low-level SPI-to-CAN controller instance */
    struct can_frame canMsg; /**< Shared memory buffer for CAN frames */

private:
    /** @brief Builds an EFF CAN ID from ID and Mode. */
    uint32_t buildCanId(uint8_t id, AKMode mode) const noexcept;
    
    /** @brief Generic transmission method for EID frames. */
    void transmit(uint32_t id, const uint8_t* data, uint8_t len) noexcept;
    
    /** @brief Converts mechanical shaft speed to electrical ERPM. */
    int32_t mechRpmToErpm(float mechRpm) const noexcept;

    /** @name Serialization Helpers */
    /** @{ */
    void appendInt32(uint8_t* buffer, int32_t val, int32_t* index) noexcept;
    void appendInt16(uint8_t* buffer, int16_t val, int16_t* index) noexcept;
    /** @} */

    std::map<uint8_t, MotorData> _motors; /**< Internal database of motor states */
};

#endif // CUBEMARSAK_H
