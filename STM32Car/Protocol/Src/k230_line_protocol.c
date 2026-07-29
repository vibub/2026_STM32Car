#include "k230_line_protocol.h"
#include "board_time.h"
#include "k230_uart.h"
#include <limits.h>
#include <string.h>

static char s_line[K230_LINE_BUFFER_SIZE];
static uint8_t s_line_length;
static uint8_t s_discarding;
static K230LineFrame s_latest_frame;
static uint8_t s_has_frame;
static uint8_t s_new_frame;
static uint32_t s_last_frame_ms;
static uint32_t s_valid_count;
static uint32_t s_invalid_count;

static uint8_t K230_ParseInteger(const char **cursor, int32_t *value,
                                 uint8_t require_comma)
{
    const char *p = *cursor;
    int32_t result = 0;
    int32_t sign = 1;
    uint8_t digits = 0U;

    if (*p == '-') {
        sign = -1;
        p++;
    }
    while ((*p >= '0') && (*p <= '9')) {
        if (result > (INT32_MAX - 9) / 10) {
            return 0U;
        }
        result = result * 10 + (int32_t) (*p - '0');
        p++;
        digits++;
    }
    if (digits == 0U) {
        return 0U;
    }

    if (require_comma != 0U) {
        if (*p != ',') {
            return 0U;
        }
        p++;
    } else if (*p != '\0') {
        return 0U;
    }

    *value = result * sign;
    *cursor = p;
    return 1U;
}

uint8_t K230LineProtocol_Parse(const char *line, K230LineFrame *frame)
{
    const char *cursor;
    int32_t values[5];
    uint8_t index;

    if ((line == 0) || (frame == 0) || (line[0] != 'L') ||
        (line[1] != ',')) {
        return 0U;
    }

    cursor = line + 2;
    for (index = 0U; index < 5U; index++) {
        if (K230_ParseInteger(&cursor, &values[index],
                              (uint8_t) (index < 4U)) == 0U) {
            return 0U;
        }
    }

    if ((values[0] < 0) || (values[0] > 1) ||
        (values[1] < -160) || (values[1] > 160) ||
        (values[2] < -900) || (values[2] > 900) ||
        (values[3] < 0) || (values[3] > 100) ||
        (values[4] < 0) || (values[4] > 7)) {
        return 0U;
    }
    if ((values[0] == 0) &&
        ((values[1] != 0) || (values[2] != 0) || (values[4] != 0))) {
        return 0U;
    }

    frame->valid = (uint8_t) values[0];
    frame->error_x = (int16_t) values[1];
    frame->angle_d10 = (int16_t) values[2];
    frame->quality = (uint8_t) values[3];
    frame->direction_mask = (uint8_t) values[4];
    return 1U;
}

void K230LineProtocol_Init(void)
{
    memset(&s_latest_frame, 0, sizeof(s_latest_frame));
    s_line_length = 0U;
    s_discarding = 0U;
    s_has_frame = 0U;
    s_new_frame = 0U;
    s_last_frame_ms = 0U;
    s_valid_count = 0U;
    s_invalid_count = 0U;
}

void K230LineProtocol_Poll(void)
{
    uint8_t byte;
    K230LineFrame frame;

    while (K230Uart_ReadByte(&byte) != 0U) {
        if (byte == '\r') {
            continue;
        }
        if (byte == '\n') {
            if ((s_discarding == 0U) && (s_line_length > 0U)) {
                s_line[s_line_length] = '\0';
                if (K230LineProtocol_Parse(s_line, &frame) != 0U) {
                    s_latest_frame = frame;
                    s_has_frame = 1U;
                    s_new_frame = 1U;
                    s_last_frame_ms = Board_TimeMs();
                    s_valid_count++;
                } else {
                    s_invalid_count++;
                }
            } else if (s_discarding != 0U) {
                s_invalid_count++;
            }
            s_line_length = 0U;
            s_discarding = 0U;
            continue;
        }

        if (s_discarding != 0U) {
            continue;
        }
        if (s_line_length >= (K230_LINE_BUFFER_SIZE - 1U)) {
            s_line_length = 0U;
            s_discarding = 1U;
            continue;
        }
        s_line[s_line_length++] = (char) byte;
    }
}

uint8_t K230LineProtocol_TakeLatest(K230LineFrame *frame)
{
    if ((frame == 0) || (s_new_frame == 0U)) {
        return 0U;
    }
    *frame = s_latest_frame;
    s_new_frame = 0U;
    return 1U;
}

uint8_t K230LineProtocol_HasFrame(void)
{
    return s_has_frame;
}

uint32_t K230LineProtocol_GetAgeMs(uint32_t now_ms)
{
    return (s_has_frame != 0U) ?
           (uint32_t) (now_ms - s_last_frame_ms) : UINT32_MAX;
}

uint32_t K230LineProtocol_GetValidCount(void)
{
    return s_valid_count;
}

uint32_t K230LineProtocol_GetInvalidCount(void)
{
    return s_invalid_count;
}
