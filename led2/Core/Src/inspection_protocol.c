#include "inspection_protocol.h"

uint8_t InspectionProtocol_Checksum(const uint8_t *data, uint8_t length)
{
    uint8_t checksum = 0U;
    uint8_t index;

    for (index = 0U; index < length; ++index)
    {
        checksum ^= data[index];
    }
    return checksum;
}

void InspectionProtocol_Build(uint8_t type,
                              uint8_t sequence,
                              uint8_t code,
                              uint8_t auxiliary,
                              uint8_t output[INSPECTION_FRAME_SIZE])
{
    output[0] = INSPECTION_SOF_1;
    output[1] = INSPECTION_SOF_2;
    output[2] = INSPECTION_VERSION;
    output[3] = type;
    output[4] = sequence;
    output[5] = code;
    output[6] = auxiliary;
    output[7] = InspectionProtocol_Checksum(&output[2], 5U);
}

void InspectionProtocol_ParserInit(InspectionParser *parser)
{
    parser->length = 0U;
    parser->checksum_errors = 0U;
    parser->discarded_bytes = 0U;
}

bool InspectionProtocol_PushByte(InspectionParser *parser,
                                 uint8_t byte,
                                 InspectionFrame *frame)
{
    if (parser->length == 0U)
    {
        if (byte != INSPECTION_SOF_1)
        {
            ++parser->discarded_bytes;
            return false;
        }
        parser->data[parser->length++] = byte;
        return false;
    }

    if (parser->length == 1U)
    {
        if (byte == INSPECTION_SOF_2)
        {
            parser->data[parser->length++] = byte;
        }
        else if (byte != INSPECTION_SOF_1)
        {
            parser->length = 0U;
            ++parser->discarded_bytes;
        }
        return false;
    }

    parser->data[parser->length++] = byte;
    if (parser->length < INSPECTION_FRAME_SIZE)
    {
        return false;
    }

    parser->length = 0U;
    if ((parser->data[2] != INSPECTION_VERSION) ||
        (parser->data[7] != InspectionProtocol_Checksum(&parser->data[2], 5U)))
    {
        ++parser->checksum_errors;
        return false;
    }

    frame->type = parser->data[3];
    frame->sequence = parser->data[4];
    frame->code = parser->data[5];
    frame->auxiliary = parser->data[6];
    return true;
}
