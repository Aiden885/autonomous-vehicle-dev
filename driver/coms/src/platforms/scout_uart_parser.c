/*
 * scout_uart_parser.c
 *
 * Created on: Aug 14, 2019 12:02
 * Description:
 *
 * Copyright (c) 2019 Ruixiang Du (rdu)
 */

#include "scout_uart_parser.h"

// #define USE_XOR_CHECKSUM

// #define PRINT_CPP_DEBUG_INFO
// #define PRINT_JLINK_DEBUG_INFO

#ifdef PRINT_CPP_DEBUG_INFO
#undef PRINT_JLINK_DEBUG_INFO
#endif

#ifdef PRINT_CPP_DEBUG_INFO
#define < iostream>
#elif (defined(PRINT_JLINK_DEBUG_INFO))
#include "segger/jlink_rtt.h"
#endif

typedef enum
{
    WAIT_FOR_HDR1 = 0,
    WAIT_FOR_HDR2,
    WAIT_FOR_HDR3,
    WAIT_FOR_FRAME_LEN,
    WAIT_FOR_PAYLOAD,
    WAIT_FOR_CHECKSUM
} ScoutSerialDecodeState;

#define PAYLOAD_BUFFER_SIZE ((uint8_t)5)

#define FRAME_SOF_LEN ((uint8_t)2)
#define FRAME_FIXED_FIELD_LEN ((uint8_t)4)

#define FRAME_HDR1 ((uint8_t)0xff)
#define FRAME_HDR2 ((uint8_t)0xa5)
#define FRAME_HDR3 ((uint8_t)0x5a)
#define FRAME_LEN ((uint8_t)0x05)
#define FRAME_CMD ((uint8_t)0x82)

#define FRAME_TYPE_CONTROL ((uint8_t)0x55)
#define FRAME_TYPE_STATUS ((uint8_t)0xaa)

#define FRAME_NONE_ID ((uint8_t)0x00)

// frame buffer
static struct
{
    uint8_t frame_len;
    uint8_t frame_checksum;
    uint8_t payload_buffer[PAYLOAD_BUFFER_SIZE];
    size_t payload_data_pos;
} uart_parsing_data;

// internal functions
static bool ParseChar(uint8_t c, ScoutMessage *msg);
static bool ConstructStatusMessage(ScoutMessage *msg);

void EncodeScoutMsgToUART(const ScoutMessage *msg, uint8_t *buf, uint8_t *len)
{
    buf[0] = 0xFF;
    buf[1] = 0xA5;
    buf[2] = 0x5A;
    buf[3] = 0x07;
    buf[4] = 0x81;

    buf[5] = msg->body.motion_control_msg.SteeringAngle_L;
    buf[6] = msg->body.motion_control_msg.SteeringAngle_H;
    buf[7] = msg->body.motion_control_msg.TargetSpeed;
    buf[8] = msg->body.motion_control_msg.SpeedGear;
    buf[9] = msg->body.motion_control_msg.GearShift_Along_Cross;
    buf[10] = msg->body.motion_control_msg.LightSwitch;

    buf[11] = msg->body.motion_control_msg.Sum;

    // for (size_t i = 0; i < SCOUT_CMD_BUF_LEN; i++)
    // {
    //     printf("%x  \n", buf[i]);
    // }

    *len = SCOUT_CMD_BUF_LEN;
}

bool DecodeScoutMsgFromUART(uint8_t c, ScoutMessage *msg)
{
    static ScoutMessage decoded_msg;

    bool result = ParseChar(c, &decoded_msg);
    if (result)
        *msg = decoded_msg;
    return result;
}

bool ParseChar(uint8_t c, ScoutMessage *msg)
{
    static ScoutSerialDecodeState decode_state = WAIT_FOR_HDR1;

    bool new_frame_parsed = false;
    switch (decode_state)
    {
    case WAIT_FOR_HDR1:
    {
        if (c == FRAME_HDR1)
        {
            uart_parsing_data.frame_len = 0;
            uart_parsing_data.frame_checksum = 0;
            uart_parsing_data.payload_data_pos = 0;
            memset(uart_parsing_data.payload_buffer, 0, PAYLOAD_BUFFER_SIZE);

            decode_state = WAIT_FOR_HDR2;
#ifdef PRINT_CPP_DEBUG_INFO
            std::cout << "found sof1" << std::endl;
#elif (defined(PRINT_JLINK_DEBUG_INFO))
            JLinkWriteString(0, "found sof1\n");
#endif
        }
        break;
    }
    case WAIT_FOR_HDR2:
    {
        if (c == FRAME_HDR2)
        {
            decode_state = WAIT_FOR_HDR3;
#ifdef PRINT_CPP_DEBUG_INFO
            std::cout << "found sof2" << std::endl;
#elif (defined(PRINT_JLINK_DEBUG_INFO))
            JLinkWriteString(0, "found sof2\n");
#endif
        }
        else
        {
            decode_state = WAIT_FOR_HDR1;
#ifdef PRINT_CPP_DEBUG_INFO
            std::cout << "failed to find sof2" << std::endl;
#elif (defined(PRINT_JLINK_DEBUG_INFO))
            JLinkWriteString(0, "failed to find sof2\n");
#endif
        }
        break;
    }
    case WAIT_FOR_HDR3:
    {
        if (c == FRAME_HDR3)
        {
            decode_state = WAIT_FOR_FRAME_LEN;
#ifdef PRINT_CPP_DEBUG_INFO
            std::cout << "found sof2" << std::endl;
#elif (defined(PRINT_JLINK_DEBUG_INFO))
            JLinkWriteString(0, "found sof2\n");
#endif
        }
        else
        {
            decode_state = WAIT_FOR_HDR1;
#ifdef PRINT_CPP_DEBUG_INFO
            std::cout << "failed to find sof2" << std::endl;
#elif (defined(PRINT_JLINK_DEBUG_INFO))
            JLinkWriteString(0, "failed to find sof2\n");
#endif
        }
        break;
    }
    case WAIT_FOR_FRAME_LEN:
    {
        if (c == FRAME_LEN)
        {
            uart_parsing_data.frame_len = c;
            decode_state = WAIT_FOR_PAYLOAD;
#ifdef PRINT_CPP_DEBUG_INFO
            std::cout << "frame len: " << std::hex << static_cast<int>(frame_len) << std::dec << std::endl;
#elif (defined(PRINT_JLINK_DEBUG_INFO))
            JLinkRTTPrintf(0, "frame len: %d\n", frame_len);
#endif
        }
        else
        {
            decode_state = WAIT_FOR_HDR1;
#ifdef PRINT_CPP_DEBUG_INFO
            std::cout << "failed to find sof2" << std::endl;
#elif (defined(PRINT_JLINK_DEBUG_INFO))
            JLinkWriteString(0, "failed to find sof2\n");
#endif
        }
        break;
    }
    case WAIT_FOR_PAYLOAD:
    {
        uart_parsing_data.payload_buffer[uart_parsing_data.payload_data_pos++] = c;
#ifdef PRINT_CPP_DEBUG_INFO
        std::cout << "1 byte added: " << std::hex << static_cast<int>(c) << std::dec << std::endl;
#elif (defined(PRINT_JLINK_DEBUG_INFO))
        JLinkRTTPrintf(0, "1 byte added: %d\n", c);
#endif
        if (uart_parsing_data.payload_data_pos == uart_parsing_data.frame_len)
            decode_state = WAIT_FOR_CHECKSUM;
        break;
    }
    case WAIT_FOR_CHECKSUM:
    {
        uart_parsing_data.frame_checksum = c;
        new_frame_parsed = true;
        decode_state = WAIT_FOR_HDR1;
#ifdef PRINT_CPP_DEBUG_INFO
        std::cout << "--- frame checksum: " << std::hex << static_cast<int>(frame_checksum) << std::dec << std::endl;
        std::cout << "--- internal frame checksum: " << std::hex << static_cast<int>(internal_checksum) << std::dec << std::endl;
#elif (defined(PRINT_JLINK_DEBUG_INFO))
        JLinkRTTPrintf(0, "--- frame checksum: : %d\n", frame_checksum);
        JLinkRTTPrintf(0, "--- internal frame checksum: : %d\n", internal_checksum);
#endif
        break;
    }
    default:
        break;
    }

    if (new_frame_parsed)
    {
#ifdef PRINT_CPP_DEBUG_INFO
        std::cout << "checksum correct" << std::endl;
#elif (defined(PRINT_JLINK_DEBUG_INFO))
        JLinkWriteString(0, "checksum correct\n");
#endif
        ConstructStatusMessage(msg);
    }

    return new_frame_parsed;
}

bool ConstructStatusMessage(ScoutMessage *msg)
{
    if (msg == NULL)
        return false;

    msg->body.motion_status_msg.steering_angle.low_byte = uart_parsing_data.payload_buffer[1];
    msg->body.motion_status_msg.steering_angle.high_byte = uart_parsing_data.payload_buffer[2];
    msg->body.motion_status_msg.velocity = uart_parsing_data.payload_buffer[3];
    msg->body.motion_status_msg.mode = uart_parsing_data.payload_buffer[4];

    return true;
}
