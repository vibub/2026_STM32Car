#include "line_follow_control.h"
#include "app_config.h"
#include <string.h>

#define LINE_FILTER_ALPHA                0.35f
#define LINE_ERROR_KP                    0.0010f
#define LINE_ERROR_KD                    0.0100f
#define LINE_ANGLE_KP                    0.0020f
#define LINE_RATIO_TO_CM_S              60.0f
#define LINE_CORRECTION_LIMIT_RATIO      0.120f
#define LINE_CORRECTION_SLEW_RATIO       0.035f
#define LINE_ERROR_DEADBAND_PX           4.0f
#define LINE_ANGLE_DEADBAND_DEG          2.0f
#define LINE_MEDIUM_ERROR_PX            20.0f
#define LINE_LARGE_ERROR_PX             45.0f
#define LINE_MEDIUM_SPEED_SCALE          0.65f
#define LINE_LARGE_SPEED_SCALE           0.39f
#define LINE_MEDIUM_MIN_RATIO            0.025f
#define LINE_LARGE_MIN_RATIO             0.070f
#define LINE_INVALID_SPEED_SCALE         0.45f
#define LINE_INVALID_DECAY_CM_S          0.50f

static float LineFollow_Abs(float value)
{
    return (value < 0.0f) ? -value : value;
}

static float LineFollow_Limit(float value, float limit)
{
    if (value > limit) {
        return limit;
    }
    if (value < -limit) {
        return -limit;
    }
    return value;
}

static float LineFollow_LimitDelta(float value, float previous, float delta)
{
    if (value > previous + delta) {
        return previous + delta;
    }
    if (value < previous - delta) {
        return previous - delta;
    }
    return value;
}

static float LineFollow_MoveTowardZero(float value, float delta)
{
    if (value > delta) {
        return value - delta;
    }
    if (value < -delta) {
        return value + delta;
    }
    return 0.0f;
}

void LineFollowControl_Init(LineFollowControl *control)
{
    if (control != 0) {
        memset(control, 0, sizeof(*control));
    }
}

void LineFollowControl_Update(
    LineFollowControl *control,
    uint32_t now_ms,
    const K230LineFrame *frame,
    uint8_t frame_new,
    uint8_t frame_fresh,
    LineFollowOutput *output)
{
    uint32_t valid_age_ms;
    float abs_error;
    float error_for_control;
    float angle_for_control;
    float correction_ratio;
    float minimum_ratio = 0.0f;
    float base_speed = APP_BASE_SPEED_CM_S;

    if (output == 0) {
        return;
    }
    memset(output, 0, sizeof(*output));
    if ((control == 0) || (frame == 0) || (frame_fresh == 0U)) {
        return;
    }

    if ((frame_new != 0U) && (frame->valid != 0U) &&
        (frame->quality >= APP_K230_MIN_QUALITY)) {
        if (control->filter_ready == 0U) {
            control->filtered_error_px = (float) frame->error_x;
            control->filtered_angle_deg = (float) frame->angle_d10 / 10.0f;
            control->filter_ready = 1U;
        } else {
            control->filtered_error_px += LINE_FILTER_ALPHA *
                ((float) frame->error_x - control->filtered_error_px);
            control->filtered_angle_deg += LINE_FILTER_ALPHA *
                (((float) frame->angle_d10 / 10.0f) -
                 control->filtered_angle_deg);
        }
        control->last_valid_line_ms = now_ms;
        control->has_valid_line = 1U;

        error_for_control = control->filtered_error_px;
        angle_for_control = control->filtered_angle_deg;
        if (LineFollow_Abs(error_for_control) <= LINE_ERROR_DEADBAND_PX) {
            error_for_control = 0.0f;
        }
        if (LineFollow_Abs(angle_for_control) <= LINE_ANGLE_DEADBAND_DEG) {
            angle_for_control = 0.0f;
        }

        correction_ratio = LINE_ERROR_KP * error_for_control +
                           LINE_ERROR_KD *
                               (error_for_control - control->last_error_px) +
                           LINE_ANGLE_KP * angle_for_control;
        control->last_error_px = error_for_control;
        abs_error = LineFollow_Abs(control->filtered_error_px);
        if (abs_error >= LINE_LARGE_ERROR_PX) {
            base_speed *= LINE_LARGE_SPEED_SCALE;
            minimum_ratio = LINE_LARGE_MIN_RATIO;
        } else if (abs_error >= LINE_MEDIUM_ERROR_PX) {
            base_speed *= LINE_MEDIUM_SPEED_SCALE;
            minimum_ratio = LINE_MEDIUM_MIN_RATIO;
        }
        if ((minimum_ratio > 0.0f) &&
            (LineFollow_Abs(correction_ratio) < minimum_ratio) &&
            (correction_ratio != 0.0f)) {
            correction_ratio = (correction_ratio > 0.0f) ?
                               minimum_ratio : -minimum_ratio;
        }
        correction_ratio = LineFollow_Limit(
            correction_ratio, LINE_CORRECTION_LIMIT_RATIO);
        control->last_correction_cm_s = LineFollow_LimitDelta(
            correction_ratio * LINE_RATIO_TO_CM_S,
            control->last_correction_cm_s,
            LINE_CORRECTION_SLEW_RATIO * LINE_RATIO_TO_CM_S);
    }

    if (control->has_valid_line == 0U) {
        return;
    }
    valid_age_ms = (uint32_t) (now_ms - control->last_valid_line_ms);
    if (valid_age_ms > APP_LINE_INVALID_HOLD_MS) {
        return;
    }

    if ((frame->valid == 0U) || (frame->quality < APP_K230_MIN_QUALITY)) {
        base_speed *= LINE_INVALID_SPEED_SCALE;
        control->last_correction_cm_s = LineFollow_MoveTowardZero(
            control->last_correction_cm_s, LINE_INVALID_DECAY_CM_S);
    } else if (frame_new == 0U) {
        abs_error = LineFollow_Abs(control->filtered_error_px);
        if (abs_error >= LINE_LARGE_ERROR_PX) {
            base_speed *= LINE_LARGE_SPEED_SCALE;
        } else if (abs_error >= LINE_MEDIUM_ERROR_PX) {
            base_speed *= LINE_MEDIUM_SPEED_SCALE;
        }
    }

    output->command_valid = 1U;
    output->filtered_error_px = control->filtered_error_px;
    output->filtered_angle_deg = control->filtered_angle_deg;
    output->correction_cm_s = control->last_correction_cm_s;
    output->left_target_cm_s = base_speed + control->last_correction_cm_s;
    output->right_target_cm_s = base_speed - control->last_correction_cm_s;
}
