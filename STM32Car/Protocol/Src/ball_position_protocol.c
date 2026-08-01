#include "ball_position_protocol.h"

#include <string.h>

#define BALL_PROTOCOL_LINE_BUFFER_SIZE 48U /* 钢球 ASCII 消息的最大缓存长度。 */

typedef struct {
    char data[BALL_PROTOCOL_LINE_BUFFER_SIZE];
    uint8_t length;
    uint8_t discarding;
    uint8_t sample_ready;
    BallPositionSample latest_sample;
} BallPositionProtocolState;

static BallPositionProtocolState s_protocol;

volatile uint32_t g_ball_protocol_rx_byte_count;
volatile uint32_t g_ball_protocol_valid_frame_count;
volatile uint32_t g_ball_protocol_invalid_frame_count;
volatile uint32_t g_ball_protocol_line_overflow_count;

static uint8_t BallPositionProtocol_ParseInteger(const char **cursor,
                                                 char delimiter,
                                                 int32_t minimum,
                                                 int32_t maximum,
                                                 int32_t *value)
{
    const char *current;
    uint32_t magnitude = 0U;
    uint32_t magnitude_limit;
    uint8_t negative = 0U;
    uint8_t digit_count = 0U;
    uint8_t digit;
    int32_t parsed;

    if ((cursor == 0) || (*cursor == 0) || (value == 0)) {
        return 0U;
    }

    current = *cursor;
    if (*current == '-') {
        negative = 1U;
        current++;
    } else if (*current == '+') {
        current++;
    }

    magnitude_limit = (negative != 0U) ?
        (uint32_t) (-minimum) : (uint32_t) maximum;
    while ((*current >= '0') && (*current <= '9')) {
        digit = (uint8_t) (*current - '0');
        if ((digit > magnitude_limit) ||
            (magnitude > (magnitude_limit - digit) / 10U)) {
            return 0U;
        }
        magnitude = magnitude * 10U + digit;
        digit_count++;
        current++;
    }

    if (digit_count == 0U) {
        return 0U;
    }
    if (delimiter == '\0') {
        if (*current != '\0') {
            return 0U;
        }
    } else {
        if (*current != delimiter) {
            return 0U;
        }
        current++;
    }

    parsed = (negative != 0U) ? -(int32_t) magnitude : (int32_t) magnitude;
    if ((parsed < minimum) || (parsed > maximum)) {
        return 0U;
    }

    *cursor = current;
    *value = parsed;
    return 1U;
}

static uint8_t BallPositionProtocol_ParseLine(const char *line,
                                              BallPositionSample *sample)
{
    const char *cursor;
    int32_t valid;
    int32_t position;
    int32_t quality;
    int32_t sequence;

    if ((line == 0) || (sample == 0) ||
        (line[0] != 'B') || (line[1] != ',')) {
        return 0U;
    }

    cursor = &line[2];
    if (BallPositionProtocol_ParseInteger(&cursor, ',', 0, 1, &valid) == 0U) {
        return 0U;
    }
    if (BallPositionProtocol_ParseInteger(&cursor, ',', -32768, 32767,
                                          &position) == 0U) {
        return 0U;
    }
    if (BallPositionProtocol_ParseInteger(&cursor, ',', 0, 100,
                                          &quality) == 0U) {
        return 0U;
    }
    if (BallPositionProtocol_ParseInteger(&cursor, '\0', 0, 65535,
                                          &sequence) == 0U) {
        return 0U;
    }

    sample->valid = (uint8_t) valid;
    sample->position_d10_mm = position;
    sample->quality = (uint8_t) quality;
    sample->sequence = (uint16_t) sequence;
    return 1U;
}

void BallPositionProtocol_Init(void)
{
    memset(&s_protocol, 0, sizeof(s_protocol));
    g_ball_protocol_rx_byte_count = 0U;
    g_ball_protocol_valid_frame_count = 0U;
    g_ball_protocol_invalid_frame_count = 0U;
    g_ball_protocol_line_overflow_count = 0U;
}

void BallPositionProtocol_ResetRx(void)
{
    s_protocol.length = 0U;
    s_protocol.discarding = 0U;
}

void BallPositionProtocol_FeedByte(uint8_t byte)
{
    BallPositionSample sample;

    g_ball_protocol_rx_byte_count++;
    if (byte == (uint8_t) '\r') {
        return;
    }
    if (byte == (uint8_t) '\n') {
        if (s_protocol.discarding != 0U) {
            BallPositionProtocol_ResetRx();
            return;
        }
        if (s_protocol.length == 0U) {
            return;
        }

        s_protocol.data[s_protocol.length] = '\0';
        if (BallPositionProtocol_ParseLine(s_protocol.data, &sample) != 0U) {
            s_protocol.latest_sample = sample;
            s_protocol.sample_ready = 1U;
            g_ball_protocol_valid_frame_count++;
        } else {
            g_ball_protocol_invalid_frame_count++;
        }
        s_protocol.length = 0U;
        return;
    }

    if (s_protocol.discarding != 0U) {
        return;
    }
    if (s_protocol.length >= (BALL_PROTOCOL_LINE_BUFFER_SIZE - 1U)) {
        s_protocol.length = 0U;
        s_protocol.discarding = 1U;
        g_ball_protocol_line_overflow_count++;
        return;
    }

    s_protocol.data[s_protocol.length] = (char) byte;
    s_protocol.length++;
}

uint8_t BallPositionProtocol_TakeSample(BallPositionSample *sample)
{
    if ((sample == 0) || (s_protocol.sample_ready == 0U)) {
        return 0U;
    }

    *sample = s_protocol.latest_sample;
    s_protocol.sample_ready = 0U;
    return 1U;
}
