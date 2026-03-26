/*
 * scout_protocol.h
 *
 * Created on: Aug 07, 2019 21:49
 * Description:
 *
 * Copyright (c) 2019 Ruixiang Du (rdu)
 */

#ifndef SCOUT_PROTOCOL_H
#define SCOUT_PROTOCOL_H

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdint.h>

#define SCOUT_CMD_BUF_LEN 12
#define SCOUT_STATUS_BUF_LEN 32

/*-------------------- Control/Feedback Messages -----------------------*/

/* No padding in the struct */
// reference: https://stackoverflow.com/questions/3318410/pragma-pack-effect
#pragma pack(push, 1)

    // Motion Control
    typedef struct
    {
        uint8_t SteeringAngle_L;
        uint8_t SteeringAngle_H;
        uint8_t TargetSpeed;
        uint8_t GearShift_Along_Cross;
        uint8_t SpeedGear;
        uint8_t LightSwitch;
        uint8_t Sum;
    } MotionControlMessage;

    typedef struct
    {
        uint8_t velocity;
        uint8_t mode;
        struct
        {
            uint8_t high_byte;
            uint8_t low_byte;
        } steering_angle;
    } MotionStatusMessage;

    typedef struct
    {
        union
        {
            // status messages
            MotionStatusMessage motion_status_msg;
            // control messages
            MotionControlMessage motion_control_msg;
        } body;
    } ScoutMessage;

#pragma pack(pop)

#ifdef __cplusplus
}
#endif

#endif /* SCOUT_PROTOCOL_H */
