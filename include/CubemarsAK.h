#ifndef CUBEMARSAK_H
#define CUBEMARSAK_H

#include <Arduino.h>
#include <SPI.h>
#include <mcp2515.h>
#include <map>

// struct MotorData {
//     float position;
//     float speed;
//     float current;
//     uint8_t motorTemp;
//     uint8_t errorCode;
// };

// Define the AKMode enum
enum AKMode {
    AK_PWM = 0,
    AK_CURRENT = 1,
    AK_CURRENT_BRAKE = 2,
    AK_VELOCITY = 3,
    AK_POSITION = 4,
    AK_ORIGIN = 5,
    AK_POSITION_VELOCITY = 6,
};


class CubemarsAK {
public:
    CubemarsAK(uint8_t csPin);
    ~CubemarsAK();

    // std::map<canid_t, MotorData> motorReadings;
    
    void initializeCAN();

    uint32_t canId(int id, AKMode Mode_set);

    unsigned int float_to_uint(float x, float x_min, float x_max, float bits);
    float uint_to_float(unsigned int x_int, float x_min, float x_max, int bits);
    void pack_cmd();

    void set_duty(uint8_t controller_id, float duty);
    void set_current(uint8_t controller_id, float current);
    void set_cb(uint8_t controller_id, float current);
    void set_spd(uint8_t controller_id, float rpm);
    void set_pos(uint8_t controller_id, float pos);
    void set_origin(uint8_t controller_id, uint8_t set_origin_mode);
    void set_pos_spd(uint8_t controller_id, float pos, int16_t spd, int16_t RPA);

    void comm_can_transmit_eid(uint32_t id, const uint8_t *data, uint8_t len);
    void buffer_append_int32(uint8_t* buffer, int32_t number, int32_t *index);
    void buffer_append_int16(uint8_t* buffer, int16_t number, int16_t *index);

    void unpackServo();
    float getPosition(canid_t can_id);
    float getSpeed(canid_t can_id);
    float getCurrent(canid_t can_id);
    int8_t getMotorTemp(canid_t can_id);
    uint8_t getErrorCode(canid_t can_id);

    struct can_frame canMsg2;
    MCP2515 mcp2515;

private:
    uint32_t controller_id;
    float position;
    float speed;
    float current;
    int8_t motorTemp;
    uint8_t errorCode;
};

#endif // CUBEMARSAK_H
