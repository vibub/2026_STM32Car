#ifndef INFRARED_LINE_CONTROL_H
#define INFRARED_LINE_CONTROL_H

#include <stdint.h>

typedef struct {
    uint8_t filter_ready;
    uint8_t has_valid_line;
    uint8_t base_speed_ready;
    uint8_t smooth_response_enabled; /* 第四、第五题启用平滑速度与差速响应。 */
    int8_t last_line_side;
    uint32_t last_valid_line_ms;
    float filtered_error;
    float last_error;
    float last_correction_cm_s;
    float base_speed_cm_s;
    float applied_base_speed_cm_s;  /* 路况缩放后的平滑基础速度，单位厘米每秒。 */
    float applied_base_accel_cm_s2; /* 基础速度轨迹当前加速度，单位厘米每二次方秒。 */
    float maximum_accel_cm_s2;      /* 当前任务允许的最大基础速度加速度。 */
    float maximum_decel_cm_s2;      /* 当前任务允许的最大基础速度减速度。 */
    float maximum_jerk_cm_s3;       /* 当前任务允许的基础速度加加速度。 */
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
void InfraredLineControl_SetSmoothResponse(InfraredLineControl *control,
                                           uint8_t enabled);
void InfraredLineControl_SetMotionLimits(InfraredLineControl *control,
                                         float maximum_accel_cm_s2,
                                         float maximum_decel_cm_s2,
                                         float maximum_jerk_cm_s3);
void InfraredLineControl_SetBaseSpeed(InfraredLineControl *control,
                                      float base_speed_cm_s);
void InfraredLineControl_Update(InfraredLineControl *control,
                                uint32_t now_ms,
                                uint8_t sensor_mask,
                                uint8_t force_wide_slowdown,
                                InfraredLineOutput *output);

#endif /* INFRARED_LINE_CONTROL_H */
