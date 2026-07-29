#ifndef LINE_FOLLOW_CONTROL_H
#define LINE_FOLLOW_CONTROL_H

#include <stdint.h>
#include "k230_line_protocol.h"

typedef struct {
    uint8_t filter_ready;
    uint8_t has_valid_line;
    uint32_t last_valid_line_ms;
    float filtered_error_px;
    float filtered_angle_deg;
    float last_error_px;
    float last_correction_cm_s;
} LineFollowControl;

typedef struct {
    uint8_t command_valid;
    float left_target_cm_s;
    float right_target_cm_s;
    float filtered_error_px;
    float filtered_angle_deg;
    float correction_cm_s;
} LineFollowOutput;

void LineFollowControl_Init(LineFollowControl *control);
void LineFollowControl_Update(
    LineFollowControl *control,
    uint32_t now_ms,
    const K230LineFrame *frame,
    uint8_t frame_new,
    uint8_t frame_fresh,
    LineFollowOutput *output);

#endif /* LINE_FOLLOW_CONTROL_H */
