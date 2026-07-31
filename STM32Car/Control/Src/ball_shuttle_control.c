#include "ball_shuttle_control.h"

#include <string.h>

#define BALL_SHUTTLE_READY_POSITION_MM          5.0f /* 中心准备阶段允许的位置误差。 */
#define BALL_SHUTTLE_READY_SPEED_MM_S            5.0f /* 中心准备阶段允许的钢球速度。 */
#define BALL_SHUTTLE_READY_HYST_POSITION_MM      8.0f /* 已就绪状态的位置撤销阈值。 */
#define BALL_SHUTTLE_READY_HYST_SPEED_MM_S       8.0f /* 已就绪状态的速度撤销阈值。 */
#define BALL_SHUTTLE_READY_TIME_MS              500U  /* 中心条件连续满足的最短时间。 */
#define BALL_SHUTTLE_TURN_ARM_POSITION_MM       30.0f /* 进入正向中后段后武装预测折返。 */
#define BALL_SHUTTLE_TURN_ARM_SPEED_MM_S         5.0f /* 武装预测折返所需的最小正向速度。 */
#define BALL_SHUTTLE_POSITIVE_TARGET_MM         50.0f /* 第三题正向物理峰值目标。 */
#define BALL_SHUTTLE_POSITIVE_TOLERANCE_MM       5.0f /* 正向物理峰值允许误差。 */
#define BALL_SHUTTLE_TURN_LEAD_MIN_MM            2.0f /* 动态折返最小提前量。 */
#define BALL_SHUTTLE_TURN_LEAD_MAX_MM           15.0f /* 动态折返最大提前量。 */
#define BALL_SHUTTLE_TURN_CONFIRM_SAMPLES         2U  /* 预测折返连续新样本确认数。 */
#define BALL_SHUTTLE_POSITIVE_HARD_LIMIT_MM     55.0f /* 正向实测位置的无条件折返上限。 */
#define BALL_SHUTTLE_PEAK_REVERSE_SPEED_MM_S    -5.0f /* 确认物理峰值所需的反向速度。 */
#define BALL_SHUTTLE_PEAK_DROP_MM                1.0f /* 确认物理峰值所需的最小回落量。 */
#define BALL_SHUTTLE_PEAK_CONFIRM_SAMPLES         2U  /* 速度反向连续新样本确认数。 */
#define BALL_SHUTTLE_NEGATIVE_SETTLE_MM        -30.0f /* 进入最终目标前二十毫米时启用端点积分。 */
#define BALL_SHUTTLE_FINAL_TARGET_MM            -50.0f /* 第三题最终稳定目标位置。 */
#define BALL_SHUTTLE_FINAL_TOLERANCE_MM           5.0f /* 最终位置必须满足题目正负五毫米要求。 */
#define BALL_SHUTTLE_FINAL_SPEED_MM_S             5.0f /* 最终稳定允许的最大钢球速度。 */
#define BALL_SHUTTLE_STABLE_TIME_MS             500U  /* 位置和速度连续满足要求的最短时间。 */
#define BALL_SHUTTLE_TIMEOUT_MS                5000U  /* 正式起跑到最终稳定的总时间上限。 */

static float BallShuttleControl_Abs(float value)
{
    return (value < 0.0f) ? -value : value;
}

static float BallShuttleControl_Max(float first, float second)
{
    return (first > second) ? first : second;
}

static float BallShuttleControl_Clamp(float value,
                                      float minimum,
                                      float maximum)
{
    if (value < minimum) {
        return minimum;
    }
    if (value > maximum) {
        return maximum;
    }
    return value;
}

static uint8_t BallShuttleControl_IsHealthy(
    const BallShuttleMeasurement *measurement)
{
    return (uint8_t) ((measurement != 0) &&
                      (measurement->balance_enabled != 0U) &&
                      (measurement->tracking_valid != 0U) &&
                      (measurement->timed_out == 0U) &&
                      (measurement->imu_fault == 0U) &&
                      (measurement->mapping_fault == 0U));
}

static BallShuttleEvent BallShuttleControl_Fail(
    BallShuttleControl *control,
    BallShuttleFailure failure)
{
    control->state = BALL_SHUTTLE_STATE_FAILED;
    control->failure = failure;
    control->start_ready = 0U;
    control->ready_timer_active = 0U;
    control->stable_ready = 0U;
    return BALL_SHUTTLE_EVENT_FAILED;
}

static BallShuttleEvent BallShuttleControl_UpdateReady(
    BallShuttleControl *control,
    uint32_t now_ms,
    const BallShuttleMeasurement *measurement)
{
    uint8_t health_ready;
    uint8_t center_ready;
    uint8_t hysteresis_ready;

    health_ready = BallShuttleControl_IsHealthy(measurement);
    if ((measurement == 0) ||
        (measurement->ready_monitor_enabled == 0U) ||
        (health_ready == 0U)) {
        control->ready_timer_active = 0U;
        if (control->start_ready != 0U) {
            control->start_ready = 0U;
            return BALL_SHUTTLE_EVENT_NOT_READY;
        }
        return BALL_SHUTTLE_EVENT_NONE;
    }

    hysteresis_ready = (uint8_t) (
        (BallShuttleControl_Abs(measurement->filtered_position_mm) <=
         BALL_SHUTTLE_READY_HYST_POSITION_MM) &&
        (BallShuttleControl_Abs(measurement->velocity_mm_s) <=
         BALL_SHUTTLE_READY_HYST_SPEED_MM_S));
    if (control->start_ready != 0U) {
        if (hysteresis_ready != 0U) {
            return BALL_SHUTTLE_EVENT_NONE;
        }
        control->start_ready = 0U;
        control->ready_timer_active = 0U;
        return BALL_SHUTTLE_EVENT_NOT_READY;
    }

    center_ready = (uint8_t) (
        (BallShuttleControl_Abs(measurement->filtered_position_mm) <=
         BALL_SHUTTLE_READY_POSITION_MM) &&
        (BallShuttleControl_Abs(measurement->velocity_mm_s) <=
         BALL_SHUTTLE_READY_SPEED_MM_S));
    if (center_ready == 0U) {
        control->ready_timer_active = 0U;
        return BALL_SHUTTLE_EVENT_NONE;
    }
    if (measurement->measurement_accepted == 0U) {
        return BALL_SHUTTLE_EVENT_NONE;
    }

    if (control->ready_timer_active == 0U) {
        control->ready_timer_active = 1U;
        control->ready_since_ms = now_ms;
    } else if ((uint32_t) (now_ms - control->ready_since_ms) >=
               BALL_SHUTTLE_READY_TIME_MS) {
        control->start_ready = 1U;
        control->ready_timer_active = 0U;
        return BALL_SHUTTLE_EVENT_READY;
    }
    return BALL_SHUTTLE_EVENT_NONE;
}

static void BallShuttleControl_UpdatePeak(
    BallShuttleControl *control,
    const BallShuttleMeasurement *measurement)
{
    uint8_t reverse_sample;

    if (measurement->filtered_position_mm > control->positive_peak_mm) {
        control->positive_peak_mm = measurement->filtered_position_mm;
    }
    if ((control->positive_peak_checked != 0U) ||
        (measurement->measurement_accepted == 0U)) {
        return;
    }

    reverse_sample = (uint8_t) (
        (measurement->velocity_mm_s <=
         BALL_SHUTTLE_PEAK_REVERSE_SPEED_MM_S) &&
        (measurement->filtered_position_mm <=
         (control->positive_peak_mm - BALL_SHUTTLE_PEAK_DROP_MM)));
    if (reverse_sample != 0U) {
        if (control->peak_reverse_confirm_count <
            BALL_SHUTTLE_PEAK_CONFIRM_SAMPLES) {
            control->peak_reverse_confirm_count++;
        }
        if (control->peak_reverse_confirm_count >=
            BALL_SHUTTLE_PEAK_CONFIRM_SAMPLES) {
            control->positive_peak_checked = 1U;
            control->positive_peak_valid = (uint8_t) (
                BallShuttleControl_Abs(
                    control->positive_peak_mm -
                    BALL_SHUTTLE_POSITIVE_TARGET_MM) <=
                BALL_SHUTTLE_POSITIVE_TOLERANCE_MM);
        }
    } else {
        control->peak_reverse_confirm_count = 0U;
    }
}

void BallShuttleControl_Init(BallShuttleControl *control)
{
    if (control == 0) {
        return;
    }

    memset(control, 0, sizeof(*control));
    control->state = BALL_SHUTTLE_STATE_IDLE;
}

uint8_t BallShuttleControl_Start(BallShuttleControl *control,
                                 uint32_t now_ms)
{
    if ((control == 0) ||
        (control->state != BALL_SHUTTLE_STATE_IDLE) ||
        (control->start_ready == 0U)) {
        return 0U;
    }

    control->state = BALL_SHUTTLE_STATE_MOVING_POSITIVE;
    control->failure = BALL_SHUTTLE_FAILURE_NONE;
    control->start_ready = 0U;
    control->ready_timer_active = 0U;
    control->stable_ready = 0U;
    control->turn_armed = 0U;
    control->turn_confirm_count = 0U;
    control->peak_reverse_confirm_count = 0U;
    control->positive_peak_checked = 0U;
    control->positive_peak_valid = 0U;
    control->started_ms = now_ms;
    control->stable_since_ms = 0U;
    control->elapsed_ms = 0U;
    control->predicted_turn_lead_mm = 0.0f;
    control->predicted_peak_mm = 0.0f;
    control->positive_peak_mm = 0.0f;
    return 1U;
}

void BallShuttleControl_Stop(BallShuttleControl *control)
{
    if (control == 0) {
        return;
    }

    BallShuttleControl_Init(control);
}

BallShuttleEvent BallShuttleControl_Update(
    BallShuttleControl *control,
    uint32_t now_ms,
    const BallShuttleMeasurement *measurement)
{
    float prediction_lead_mm;
    float braking_lead_mm;
    float final_error_mm;
    uint8_t final_position_ready;
    uint8_t final_speed_ready;
    uint8_t predicted_turn_ready;

    if (control == 0) {
        return BALL_SHUTTLE_EVENT_NONE;
    }
    if (control->state == BALL_SHUTTLE_STATE_IDLE) {
        return BallShuttleControl_UpdateReady(control, now_ms, measurement);
    }
    if (control->state == BALL_SHUTTLE_STATE_FAILED) {
        return BALL_SHUTTLE_EVENT_NONE;
    }
    if (measurement == 0) {
        return BallShuttleControl_Fail(
            control, BALL_SHUTTLE_FAILURE_TRACKING);
    }

    control->elapsed_ms = (uint32_t) (now_ms - control->started_ms);
    if ((measurement->timed_out != 0U) ||
        (measurement->tracking_valid == 0U)) {
        return BallShuttleControl_Fail(
            control, BALL_SHUTTLE_FAILURE_TRACKING);
    }
    if ((measurement->balance_enabled == 0U) ||
        (measurement->imu_fault != 0U) ||
        (measurement->mapping_fault != 0U)) {
        return BallShuttleControl_Fail(
            control, BALL_SHUTTLE_FAILURE_BALANCE);
    }
    if ((control->state != BALL_SHUTTLE_STATE_HOLDING_NEGATIVE) &&
        (control->elapsed_ms > BALL_SHUTTLE_TIMEOUT_MS)) {
        return BallShuttleControl_Fail(
            control, BALL_SHUTTLE_FAILURE_TIMEOUT);
    }

    switch (control->state) {
        case BALL_SHUTTLE_STATE_MOVING_POSITIVE:
            if (measurement->filtered_position_mm >
                control->positive_peak_mm) {
                control->positive_peak_mm =
                    measurement->filtered_position_mm;
            }
            if (measurement->filtered_position_mm >=
                BALL_SHUTTLE_POSITIVE_HARD_LIMIT_MM) {
                control->state = BALL_SHUTTLE_STATE_MOVING_NEGATIVE;
                control->turn_confirm_count = 0U;
                return BALL_SHUTTLE_EVENT_TURN_NEGATIVE;
            }
            if ((control->turn_armed == 0U) &&
                (measurement->filtered_position_mm >=
                 BALL_SHUTTLE_TURN_ARM_POSITION_MM) &&
                (measurement->velocity_mm_s >=
                 BALL_SHUTTLE_TURN_ARM_SPEED_MM_S)) {
                control->turn_armed = 1U;
            }

            prediction_lead_mm =
                measurement->predicted_position_mm -
                measurement->filtered_position_mm;
            if (prediction_lead_mm < 0.0f) {
                prediction_lead_mm = 0.0f;
            }
            braking_lead_mm = measurement->stop_distance_mm;
            if (braking_lead_mm < 0.0f) {
                braking_lead_mm = 0.0f;
            }
            control->predicted_turn_lead_mm = BallShuttleControl_Clamp(
                BallShuttleControl_Max(prediction_lead_mm,
                                       braking_lead_mm),
                BALL_SHUTTLE_TURN_LEAD_MIN_MM,
                BALL_SHUTTLE_TURN_LEAD_MAX_MM);
            control->predicted_peak_mm =
                measurement->filtered_position_mm +
                control->predicted_turn_lead_mm;
            /* 慢速未武装时，正目标本身仍需连续两个新样本确认。 */
            predicted_turn_ready = (uint8_t) (
                (((control->turn_armed != 0U) &&
                  (control->predicted_peak_mm >=
                   BALL_SHUTTLE_POSITIVE_TARGET_MM))) ||
                (measurement->filtered_position_mm >=
                 BALL_SHUTTLE_POSITIVE_TARGET_MM));

            if (measurement->measurement_accepted != 0U) {
                if (predicted_turn_ready != 0U) {
                    if (control->turn_confirm_count <
                        BALL_SHUTTLE_TURN_CONFIRM_SAMPLES) {
                        control->turn_confirm_count++;
                    }
                } else {
                    control->turn_confirm_count = 0U;
                }
            }
            if (control->turn_confirm_count >=
                BALL_SHUTTLE_TURN_CONFIRM_SAMPLES) {
                control->state = BALL_SHUTTLE_STATE_MOVING_NEGATIVE;
                control->turn_confirm_count = 0U;
                return BALL_SHUTTLE_EVENT_TURN_NEGATIVE;
            }
            break;

        case BALL_SHUTTLE_STATE_MOVING_NEGATIVE:
            BallShuttleControl_UpdatePeak(control, measurement);
            if (measurement->filtered_position_mm <=
                BALL_SHUTTLE_NEGATIVE_SETTLE_MM) {
                control->state = BALL_SHUTTLE_STATE_SETTLING_NEGATIVE;
                control->stable_ready = 0U;
                return BALL_SHUTTLE_EVENT_ENABLE_ENDPOINT_HOLD;
            }
            break;

        case BALL_SHUTTLE_STATE_SETTLING_NEGATIVE:
            BallShuttleControl_UpdatePeak(control, measurement);
            final_error_mm = measurement->filtered_position_mm -
                             BALL_SHUTTLE_FINAL_TARGET_MM;
            final_position_ready = (uint8_t) (
                BallShuttleControl_Abs(final_error_mm) <=
                BALL_SHUTTLE_FINAL_TOLERANCE_MM);
            final_speed_ready = (uint8_t) (
                BallShuttleControl_Abs(measurement->velocity_mm_s) <=
                BALL_SHUTTLE_FINAL_SPEED_MM_S);
            if ((final_position_ready != 0U) &&
                (final_speed_ready != 0U)) {
                if (control->stable_ready == 0U) {
                    control->stable_ready = 1U;
                    control->stable_since_ms = now_ms;
                } else if ((uint32_t) (now_ms -
                                       control->stable_since_ms) >=
                           BALL_SHUTTLE_STABLE_TIME_MS) {
                    if ((control->positive_peak_checked == 0U) ||
                        (control->positive_peak_valid == 0U)) {
                        return BallShuttleControl_Fail(
                            control,
                            BALL_SHUTTLE_FAILURE_POSITIVE_PEAK);
                    }
                    control->state =
                        BALL_SHUTTLE_STATE_HOLDING_NEGATIVE;
                    return BALL_SHUTTLE_EVENT_SUCCEEDED;
                }
            } else {
                control->stable_ready = 0U;
            }
            break;

        case BALL_SHUTTLE_STATE_HOLDING_NEGATIVE:
            return BALL_SHUTTLE_EVENT_NONE;

        default:
            return BallShuttleControl_Fail(
                control, BALL_SHUTTLE_FAILURE_BALANCE);
    }

    return BALL_SHUTTLE_EVENT_NONE;
}

uint8_t BallShuttleControl_IsReady(const BallShuttleControl *control)
{
    return (uint8_t) ((control != 0) &&
                      (control->state == BALL_SHUTTLE_STATE_IDLE) &&
                      (control->start_ready != 0U));
}

uint8_t BallShuttleControl_IsRunning(const BallShuttleControl *control)
{
    if (control == 0) {
        return 0U;
    }

    return (uint8_t) (
        (control->state == BALL_SHUTTLE_STATE_MOVING_POSITIVE) ||
        (control->state == BALL_SHUTTLE_STATE_MOVING_NEGATIVE) ||
        (control->state == BALL_SHUTTLE_STATE_SETTLING_NEGATIVE));
}

uint8_t BallShuttleControl_IsHolding(const BallShuttleControl *control)
{
    return (uint8_t) ((control != 0) &&
                      (control->state ==
                       BALL_SHUTTLE_STATE_HOLDING_NEGATIVE));
}
