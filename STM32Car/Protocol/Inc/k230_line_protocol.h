#ifndef K230_LINE_PROTOCOL_H
#define K230_LINE_PROTOCOL_H

#include <stdint.h>

#define K230_LINE_BUFFER_SIZE 40U

typedef struct {
    uint8_t valid;
    int16_t error_x;
    int16_t angle_d10;
    uint8_t quality;
    uint8_t direction_mask;
} K230LineFrame;

void K230LineProtocol_Init(void);
void K230LineProtocol_Poll(void);
uint8_t K230LineProtocol_Parse(const char *line, K230LineFrame *frame);
uint8_t K230LineProtocol_TakeLatest(K230LineFrame *frame);
uint8_t K230LineProtocol_HasFrame(void);
uint32_t K230LineProtocol_GetAgeMs(uint32_t now_ms);
uint32_t K230LineProtocol_GetValidCount(void);
uint32_t K230LineProtocol_GetInvalidCount(void);

#endif /* K230_LINE_PROTOCOL_H */
