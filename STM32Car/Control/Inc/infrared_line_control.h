#ifndef INFRARED_LINE_CONTROL_H
#define INFRARED_LINE_CONTROL_H

#include <stdint.h>

typedef struct {
    uint8_t filter_ready;
    uint8_t has_valid_line;
    uint8_t base_speed_ready;
    int8_t last_line_side;
    uint32_t last_valid_line_ms;
    float filtered_error;
    float last_error;
    float last_correction_cm_s;
    float base_speed_cm_s;
    float applied_base_speed_cm_s;
} InfraredLineControl;

typedef struct {
    uint8_t command_valid;
    uint8_t sensor_mask;
    uint8_t active_count;
    uint8_t line_lost;
    uint8_t wide_line;
    float raw_error;
    float filtered_error;
    float correction_cm_s;
    float left_target_cm_s;
    float right_target_cm_s;
} InfraredLineOutput;

void InfraredLineControl_Init(InfraredLineControl *control);
void InfraredLineControl_SetBaseSpeed(InfraredLineControl *control,
                                      float base_speed_cm_s);
void InfraredLineControl_Update(InfraredLineControl *control,
                                uint32_t now_ms,
                                uint8_t sensor_mask,
                                uint8_t force_wide_slowdown,
                                InfraredLineOutput *output);

#endif /* INFRARED_LINE_CONTROL_H */
