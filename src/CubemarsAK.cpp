#include "CubemarsAK.h"


CubemarsAK::CubemarsAK(uint8_t csPin) : mcp2515(csPin){}

CubemarsAK::~CubemarsAK()
{}

void CubemarsAK::initializeCAN() {
    mcp2515.reset(); 
    mcp2515.setBitrate(CAN_1000KBPS, MCP_8MHZ); 
    mcp2515.setNormalMode(); 
    Serial.println("Setup success");
}

uint32_t CubemarsAK::canId(int id, AKMode Mode_set) {
    return (uint32_t)(id | Mode_set << 8);
}


void CubemarsAK::comm_can_transmit_eid(uint32_t id, const uint8_t *data, uint8_t len) {
    canMsg2.can_id = id | CAN_EFF_FLAG;
    canMsg2.can_dlc = (len > 8) ? 8 : len;
    for (uint8_t i = 0; i < len; i++) {
        canMsg2.data[i] = data[i];
    }
    MCP2515::ERROR sendStatus = mcp2515.sendMessage(MCP2515::TXB1, &canMsg2);
    if (sendStatus != MCP2515::ERROR_OK) {
        Serial.println("Error sending message...");
    }
}

void CubemarsAK::buffer_append_int32(uint8_t* buffer, int32_t number, int32_t *index) {
    buffer[(*index)++] = number >> 24;
    buffer[(*index)++] = number >> 16;
    buffer[(*index)++] = number >> 8;
    buffer[(*index)++] = number;
}

void CubemarsAK::buffer_append_int16(uint8_t* buffer, int16_t number, int16_t *index) {
    buffer[(*index)++] = number >> 8;
    buffer[(*index)++] = number;
}

// Convert Mechanical RPM (output shaft) to Electrical RPM (ERPM)
int32_t CubemarsAK::mechRpmToErpm(float mechRpm) {
    return (int32_t)(mechRpm * AK40_10_GEAR_RATIO * AK40_10_POLE_PAIRS);
}

// DUTY CYCLE MODE - #0
void CubemarsAK::set_duty(uint8_t controller_id, float duty)
{
    int32_t send_index = 0;
    uint8_t buffer[4];
    buffer_append_int32(buffer, (int32_t)(duty * 100000.0), &send_index);
    comm_can_transmit_eid(canId(controller_id, AKMode::AK_PWM), buffer, send_index);
}

// CURRENT LOOP MODE - #1
// The current value is of int32 type, and the value (-60000, 60000) represents -60-60A.
void CubemarsAK::set_current(uint8_t controller_id, float current)
{
    // Safety: Clamp current to motor limits
    if (current > AK40_10_MAX_CURRENT) current = AK40_10_MAX_CURRENT;
    if (current < -AK40_10_MAX_CURRENT) current = -AK40_10_MAX_CURRENT;

    int32_t send_index = 0;
    uint8_t buffer[4];
    buffer_append_int32(buffer, (int32_t)(current * 1000.0), &send_index);
    comm_can_transmit_eid(canId(controller_id, AKMode::AK_CURRENT), buffer, send_index);
}

// CURRENT BRAKE MODE - #2
// The braking current value is of int32 type, and the value (0, 60000) represents 0-60A.
void CubemarsAK::set_cb(uint8_t controller_id, float current)
{
    // Safety: Clamp current to motor limits
    if (current > AK40_10_MAX_CURRENT) current = AK40_10_MAX_CURRENT;
    if (current < 0) current = 0;

    int32_t send_index = 0;
    uint8_t buffer[4];
    buffer_append_int32(buffer, (int32_t)(current * 1000.0), &send_index);
    comm_can_transmit_eid(canId(controller_id, AKMode::AK_CURRENT_BRAKE), buffer, send_index);
}

// VELOCITY MODE - #3
// Input: mechanical RPM (output shaft). Converted to ERPM for the controller.
void CubemarsAK::set_spd(uint8_t controller_id, float rpm)
{
    int32_t send_index = 0;
    uint8_t buffer[4];
    int32_t erpm = mechRpmToErpm(rpm);
    buffer_append_int32(buffer, erpm, &send_index);
    comm_can_transmit_eid(canId(controller_id, AKMode::AK_VELOCITY), buffer, send_index);
}
// POSITION LOOP MODE - #4
// Position as int32 type，range (-360000000, 360000000) represents position (-36000°,36000°)
void CubemarsAK::set_pos(uint8_t controller_id, float pos)
{
    int32_t send_index = 0;
    uint8_t buffer[4];
    buffer_append_int32(buffer, (int32_t)(pos * 10000.0), &send_index);
    comm_can_transmit_eid(canId(controller_id, AKMode::AK_POSITION), buffer, send_index);
}
// SET ORIGIN MODE - #5
// The setting command is uint8_t type, 0 means setting the temporary origin (power failure elimination)
// 1 means setting the permanent zero point (automatic parameter saving)
// 2 means restoring the default zero point (automatic parameter saving)
void CubemarsAK::set_origin(uint8_t controller_id, uint8_t set_origin_mode)
{
    int32_t send_index = 0;
    uint8_t buffer[1]; // Change buffer size to hold only one byte
    buffer[send_index++] = set_origin_mode; // Set the origin mode in the buffer
    comm_can_transmit_eid(canId(controller_id, AKMode::AK_ORIGIN), buffer, send_index);
}

void CubemarsAK::set_pos_spd(uint8_t controller_id, float pos,int16_t spd, int16_t RPA)
{
    int32_t send_index = 0;
    int16_t send_index1 = 4;
    uint8_t buffer[8];
    buffer_append_int32(buffer, (int32_t)(pos * 10000.0), &send_index);
    buffer_append_int16(buffer, spd, &send_index1);
    buffer_append_int16(buffer, RPA, &send_index1);
    comm_can_transmit_eid(canId(controller_id, AKMode::AK_POSITION_VELOCITY), buffer, send_index1);
}
