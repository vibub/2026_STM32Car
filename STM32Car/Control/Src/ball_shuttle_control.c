#include "ball_shuttle_control.h"

#include <string.h>

#define BALL_SHUTTLE_POSITIVE_TURN_MM       45.0f /* 首次达到该正位置后立即折返，保留视觉误差余量。 */
#define BALL_SHUTTLE_NEGATIVE_SETTLE_MM    -30.0f /* 进入最终目标前二十毫米时启用端点积分。 */
#define BALL_SHUTTLE_FINAL_TARGET_MM        -50.0f /* 第三题最终稳定目标位置。 */
#define BALL_SHUTTLE_FINAL_TOLERANCE_MM       7.0f /* 软件到位范围，叠加视觉误差后仍满足题目要求。 */
#define BALL_SHUTTLE_FINAL_SPEED_MM_S         5.0f /* 最终稳定允许的最大钢球速度。 */
#define BALL_SHUTTLE_STABLE_TIME_MS         500U  /* 位置和速度连续满足要求的最短时间。 */
#define BALL_SHUTTLE_TIMEOUT_MS            5000U  /* O 点到最终稳定状态的总时间上限。 */

static float BallShuttleControl_Abs(float value)
{
    return (value < 0.0f) ? -value : value;
}

static BallShuttleEvent BallShuttleControl_Fail(
    BallShuttleControl *control,
    BallShuttleFailure failure)
{
    control->state = BALL_SHUTTLE_STATE_FAILED;
    control->failure = failure;
    control->stable_ready = 0U;
    return BALL_SHUTTLE_EVENT_FAILED;
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
        (BallShuttleControl_IsRunning(control) != 0U) ||
        (BallShuttleControl_IsHolding(control) != 0U)) {
        return 0U;
    }

    control->state = BALL_SHUTTLE_STATE_MOVING_POSITIVE;
    control->failure = BALL_SHUTTLE_FAILURE_NONE;
    control->stable_ready = 0U;
    control->started_ms = now_ms;
    control->stable_since_ms = 0U;
    control->elapsed_ms = 0U;
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
    float final_error_mm;
    uint8_t final_position_ready;
    uint8_t final_speed_ready;

    if ((control == 0) ||
        (control->state == BALL_SHUTTLE_STATE_IDLE) ||
        (control->state == BALL_SHUTTLE_STATE_FAILED)) {
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
            if (measurement->filtered_position_mm >=
                BALL_SHUTTLE_POSITIVE_TURN_MM) {
                control->state = BALL_SHUTTLE_STATE_MOVING_NEGATIVE;
                return BALL_SHUTTLE_EVENT_TURN_NEGATIVE;
            }
            break;

        case BALL_SHUTTLE_STATE_MOVING_NEGATIVE:
            if (measurement->filtered_position_mm <=
                BALL_SHUTTLE_NEGATIVE_SETTLE_MM) {
                control->state = BALL_SHUTTLE_STATE_SETTLING_NEGATIVE;
                control->stable_ready = 0U;
                return BALL_SHUTTLE_EVENT_ENABLE_ENDPOINT_HOLD;
            }
            break;

        case BALL_SHUTTLE_STATE_SETTLING_NEGATIVE:
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
                    control->state = BALL_SHUTTLE_STATE_HOLDING_NEGATIVE;
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
