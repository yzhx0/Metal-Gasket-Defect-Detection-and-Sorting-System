#ifndef INSPECTION_PROTOCOL_H
#define INSPECTION_PROTOCOL_H

#include <stdbool.h>
#include <stdint.h>

#define INSPECTION_FRAME_SIZE 8U
#define INSPECTION_SOF_1      0xAAU
#define INSPECTION_SOF_2      0x55U
#define INSPECTION_VERSION    0x01U

typedef enum
{
    INSPECTION_MSG_TRIGGER   = 0x10U,
    INSPECTION_MSG_RESULT    = 0x20U,
    INSPECTION_MSG_ACK       = 0x30U,
    INSPECTION_MSG_HEARTBEAT = 0x40U
} InspectionMessageType;

typedef enum
{
    INSPECTION_RESULT_NORMAL       = 0x00U,
    INSPECTION_RESULT_NOTCH        = 0x01U,
    INSPECTION_RESULT_DEFORMATION  = 0x02U,
    INSPECTION_RESULT_VISION_ERROR = 0x7EU,
    INSPECTION_RESULT_PROTOCOL_ERR = 0x7FU
} InspectionResultCode;

typedef struct
{
    uint8_t type;
    uint8_t sequence;
    uint8_t code;
    uint8_t auxiliary;
} InspectionFrame;

typedef struct
{
    uint8_t data[INSPECTION_FRAME_SIZE];
    uint8_t length;
    uint16_t checksum_errors;
    uint16_t discarded_bytes;
} InspectionParser;

uint8_t InspectionProtocol_Checksum(const uint8_t *data, uint8_t length);
void InspectionProtocol_Build(uint8_t type,
                              uint8_t sequence,
                              uint8_t code,
                              uint8_t auxiliary,
                              uint8_t output[INSPECTION_FRAME_SIZE]);
void InspectionProtocol_ParserInit(InspectionParser *parser);
bool InspectionProtocol_PushByte(InspectionParser *parser,
                                 uint8_t byte,
                                 InspectionFrame *frame);

#endif
