#ifndef BALL_POSITION_PROTOCOL_H
#define BALL_POSITION_PROTOCOL_H

#include <stdint.h>

typedef struct {
    uint8_t valid;
    int32_t position_d10_mm;
    uint8_t quality;
    uint16_t sequence;
} BallPositionSample;

void BallPositionProtocol_Init(void);
void BallPositionProtocol_ResetRx(void);
void BallPositionProtocol_FeedByte(uint8_t byte);
uint8_t BallPositionProtocol_TakeSample(BallPositionSample *sample);

extern volatile uint32_t g_ball_protocol_rx_byte_count;
extern volatile uint32_t g_ball_protocol_valid_frame_count;
extern volatile uint32_t g_ball_protocol_invalid_frame_count;
extern volatile uint32_t g_ball_protocol_line_overflow_count;

#endif /* BALL_POSITION_PROTOCOL_H */
