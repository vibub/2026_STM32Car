#include "infrared_line_control.h"
#include "app_config.h"
#include <string.h>

#define IR_LINE_FILTER_ALPHA                 0.45f
#define IR_LINE_KP                           3.00f
#define IR_LINE_KD                           1.80f
#define IR_LINE_CORRECTION_LIMIT_CM_S       10.00f
#define IR_LINE_CORRECTION_SLEW_CM_S         2.50f
#define IR_LINE_MEDIUM_ERROR                 1.00f
#define IR_LINE_LARGE_ERROR                  2.00f
#define IR_LINE_MEDIUM_SPEED_SCALE           0.82f
#define IR_LINE_LARGE_SPEED_SCALE            0.65f
#define IR_LINE_WIDE_SPEED_SCALE             0.65f
#define IR_LINE_RECOVERY_SPEED_SCALE         0.50f
#define IR_LINE_SEARCH_SPEED_SCALE            0.35f
#define IR_LINE_EDGE_POSITION_CM              4.20f
#define IR_LINE_DIRECTION_EPSILON_CM          0.05f
#define IR_LINE_TARGET_MAX_CM_S              60.00f
#define IR_LINE_SENSOR_MASK                  0xFFU

static float InfraredLine_Abs(float value)
{
    return (value < 0.0f) ? -value : value;
}

static float InfraredLine_Limit(float value, float minimum, float maximum)
{
    if (value > maximum) {
        return maximum;
    }
    if (value < minimum) {
        return minimum;
    }
    return value;
}

static float InfraredLine_LimitDelta(float value, float previous, float delta)
{
    if (value > previous + delta) {
        return previous + delta;
    }
    if (value < previous - delta) {
        return previous - delta;
    }
    return value;
}

static uint8_t InfraredLine_CalculateError(uint8_t mask, float *error)
{
    /* 相邻探头中心距 12 mm，bit0 在车辆左侧，单位为厘米。 */
    static const float positions_cm[8] = {
        -4.2f, -3.0f, -1.8f, -0.6f,
        0.6f, 1.8f, 3.0f, 4.2f
    };
    uint8_t index;
    uint8_t active_count = 0U;
    float position_sum = 0.0f;

    for (index = 0U; index < 8U; index++) {
        if ((mask & (uint8_t) (1U << index)) != 0U) {
            position_sum += positions_cm[index];
            active_count++;
        }
    }

    if (active_count != 0U) {
        *error = position_sum / (float) active_count;
    } else {
        *error = 0.0f;
    }
    return active_count;
}

static void InfraredLine_SetTargets(float base_speed,
                                    float correction,
                                    InfraredLineOutput *output)
{
    float left_target = base_speed + correction;
    float right_target = base_speed - correction;
    float maximum;
    float minimum;
    float shift;

    /* 左右目标成对平移，避免独立限幅把已经计算出的差速削平。 */
    maximum = (left_target > right_target) ? left_target : right_target;
    if (maximum > IR_LINE_TARGET_MAX_CM_S) {
        shift = maximum - IR_LINE_TARGET_MAX_CM_S;
        left_target -= shift;
        right_target -= shift;
    }

    minimum = (left_target < right_target) ? left_target : right_target;
    if (minimum < 0.0f) {
        shift = -minimum;
        left_target += shift;
        right_target += shift;
    }

    output->left_target_cm_s = InfraredLine_Limit(
        left_target, 0.0f, IR_LINE_TARGET_MAX_CM_S);
    output->right_target_cm_s = InfraredLine_Limit(
        right_target, 0.0f, IR_LINE_TARGET_MAX_CM_S);
}

void InfraredLineControl_Init(InfraredLineControl *control)
{
    if (control != 0) {
        memset(control, 0, sizeof(*control));
    }
}

void InfraredLineControl_Update(InfraredLineControl *control,
                                uint32_t now_ms,
                                uint8_t sensor_mask,
                                InfraredLineOutput *output)
{
    uint32_t line_age_ms;
    uint8_t active_count;
    float raw_error;
    float reported_error;
    float derivative = 0.0f;
    float target_correction;
    float base_speed = APP_BASE_SPEED_CM_S;
    float abs_error;

    if (output == 0) {
        return;
    }
    memset(output, 0, sizeof(*output));
    if (control == 0) {
        return;
    }

    sensor_mask &= IR_LINE_SENSOR_MASK;
    output->sensor_mask = sensor_mask;
    active_count = InfraredLine_CalculateError(sensor_mask, &raw_error);
    output->active_count = active_count;
    output->raw_error = raw_error;
    reported_error = raw_error;

    if (active_count != 0U) {
        output->wide_line = (uint8_t) (sensor_mask == IR_LINE_SENSOR_MASK);
        if (control->filter_ready == 0U) {
            control->filtered_error = raw_error;
            control->last_error = raw_error;
            control->filter_ready = 1U;
        } else {
            control->filtered_error += IR_LINE_FILTER_ALPHA *
                (raw_error - control->filtered_error);
            derivative = control->filtered_error - control->last_error;
            control->last_error = control->filtered_error;
        }

        if (raw_error < -IR_LINE_DIRECTION_EPSILON_CM) {
            control->last_line_side = -1;
        } else if (raw_error > IR_LINE_DIRECTION_EPSILON_CM) {
            control->last_line_side = 1;
        }

        target_correction = IR_LINE_KP * control->filtered_error +
                            IR_LINE_KD * derivative;
        target_correction = InfraredLine_Limit(
            target_correction,
            -IR_LINE_CORRECTION_LIMIT_CM_S,
            IR_LINE_CORRECTION_LIMIT_CM_S);
        control->last_correction_cm_s = InfraredLine_LimitDelta(
            target_correction,
            control->last_correction_cm_s,
            IR_LINE_CORRECTION_SLEW_CM_S);

        abs_error = InfraredLine_Abs(control->filtered_error);
        if (output->wide_line != 0U) {
            base_speed *= IR_LINE_WIDE_SPEED_SCALE;
        } else if (abs_error >= IR_LINE_LARGE_ERROR) {
            base_speed *= IR_LINE_LARGE_SPEED_SCALE;
        } else if (abs_error >= IR_LINE_MEDIUM_ERROR) {
            base_speed *= IR_LINE_MEDIUM_SPEED_SCALE;
        }

        control->last_valid_line_ms = now_ms;
        control->has_valid_line = 1U;
        reported_error = control->filtered_error;
    } else {
        output->line_lost = 1U;
        if ((control->has_valid_line == 0U) ||
            (control->last_line_side == 0)) {
            return;
        }

        line_age_ms = (uint32_t) (now_ms - control->last_valid_line_ms);
        if (line_age_ms > APP_IR_LINE_LOST_STOP_MS) {
            control->filter_ready = 0U;
            control->has_valid_line = 0U;
            control->last_line_side = 0;
            control->last_correction_cm_s = 0.0f;
            return;
        }

        if (line_age_ms <= APP_IR_LINE_LOST_RECOVERY_MS) {
            reported_error = (float) control->last_line_side *
                             IR_LINE_EDGE_POSITION_CM;
            output->raw_error = reported_error;
            target_correction = IR_LINE_KP * reported_error;
            base_speed *= IR_LINE_RECOVERY_SPEED_SCALE;
        } else {
            reported_error = (float) control->last_line_side *
                             IR_LINE_EDGE_POSITION_CM;
            output->raw_error = reported_error;
            target_correction = (float) control->last_line_side *
                                IR_LINE_CORRECTION_LIMIT_CM_S;
            base_speed *= IR_LINE_SEARCH_SPEED_SCALE;
        }

        target_correction = InfraredLine_Limit(
            target_correction,
            -IR_LINE_CORRECTION_LIMIT_CM_S,
            IR_LINE_CORRECTION_LIMIT_CM_S);
        control->last_correction_cm_s = InfraredLine_LimitDelta(
            target_correction,
            control->last_correction_cm_s,
            IR_LINE_CORRECTION_SLEW_CM_S);
    }

    output->command_valid = 1U;
    output->filtered_error = reported_error;
    output->correction_cm_s = control->last_correction_cm_s;
    InfraredLine_SetTargets(base_speed,
                            control->last_correction_cm_s,
                            output);
}
