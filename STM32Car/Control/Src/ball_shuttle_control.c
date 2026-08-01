#include "ball_shuttle_control.h"

#include <string.h>

static void BallShuttleControl_LoadDefaults(BallShuttleConfig *config)
{
    config->angle_magnitude_deg[0] =
        BALL_SHUTTLE_DEFAULT_PHASE_1_ANGLE_DEG;
    config->angle_magnitude_deg[1] =
        BALL_SHUTTLE_DEFAULT_PHASE_2_ANGLE_DEG;
    config->angle_magnitude_deg[2] =
        BALL_SHUTTLE_DEFAULT_PHASE_3_ANGLE_DEG;
    config->duration_ms[0] = BALL_SHUTTLE_DEFAULT_PHASE_1_TIME_MS;
    config->duration_ms[1] = BALL_SHUTTLE_DEFAULT_PHASE_2_TIME_MS;
    config->duration_ms[2] = BALL_SHUTTLE_DEFAULT_PHASE_3_TIME_MS;
}

static uint8_t BallShuttleControl_ConfigIsValid(
    const BallShuttleConfig *config)
{
    uint8_t index;
    uint32_t total_time_ms = 0U;

    if (config == 0) {
        return 0U;
    }
    for (index = 0U; index < BALL_SHUTTLE_PHASE_COUNT; index++) {
        if (!((config->angle_magnitude_deg[index] >= 0.0f) &&
              (config->angle_magnitude_deg[index] <=
               BALL_SHUTTLE_MAX_ANGLE_DEG))) {
            return 0U;
        }
        if ((config->duration_ms[index] <
             BALL_SHUTTLE_MIN_PHASE_TIME_MS) ||
            (config->duration_ms[index] >
             BALL_SHUTTLE_MAX_PHASE_TIME_MS)) {
            return 0U;
        }
        total_time_ms += config->duration_ms[index];
    }
    return (uint8_t) (total_time_ms <= BALL_SHUTTLE_MAX_TOTAL_TIME_MS);
}

static void BallShuttleControl_ResetRuntime(BallShuttleControl *control)
{
    control->state = BALL_SHUTTLE_STATE_IDLE;
    control->failure = BALL_SHUTTLE_FAILURE_NONE;
    control->started_ms = 0U;
    control->phase_started_ms = 0U;
    control->elapsed_ms = 0U;
    control->target_pipe_pitch_deg = 0.0f;
}

static uint8_t BallShuttleControl_MeasurementIsHealthy(
    const BallShuttleMeasurement *measurement)
{
    return (uint8_t) ((measurement != 0) &&
                      (measurement->pipe_control_enabled != 0U) &&
                      (measurement->manual_pipe_mode_enabled != 0U) &&
                      (measurement->attitude_valid != 0U) &&
                      (measurement->imu_fault == 0U) &&
                      (measurement->mapping_fault == 0U));
}

static BallShuttleEvent BallShuttleControl_Fail(
    BallShuttleControl *control)
{
    control->state = BALL_SHUTTLE_STATE_FAILED;
    control->failure = BALL_SHUTTLE_FAILURE_PIPE_CONTROL;
    control->target_pipe_pitch_deg = 0.0f;
    return BALL_SHUTTLE_EVENT_FAILED;
}

void BallShuttleControl_Init(BallShuttleControl *control)
{
    if (control == 0) {
        return;
    }

    memset(control, 0, sizeof(*control));
    BallShuttleControl_LoadDefaults(&control->config);
    control->state = BALL_SHUTTLE_STATE_IDLE;
}

uint8_t BallShuttleControl_SetConfig(BallShuttleControl *control,
                                     const BallShuttleConfig *config)
{
    if ((control == 0) ||
        (BallShuttleControl_IsRunning(control) != 0U) ||
        (BallShuttleControl_ConfigIsValid(config) == 0U)) {
        return 0U;
    }

    control->config = *config;
    return 1U;
}

void BallShuttleControl_GetConfig(const BallShuttleControl *control,
                                  BallShuttleConfig *config)
{
    if ((control == 0) || (config == 0)) {
        return;
    }

    *config = control->config;
}

uint8_t BallShuttleControl_ResetConfig(BallShuttleControl *control)
{
    BallShuttleConfig config;

    if ((control == 0) ||
        (BallShuttleControl_IsRunning(control) != 0U)) {
        return 0U;
    }
    BallShuttleControl_LoadDefaults(&config);
    control->config = config;
    return 1U;
}

uint8_t BallShuttleControl_Start(BallShuttleControl *control,
                                 uint32_t now_ms)
{
    if ((control == 0) ||
        (BallShuttleControl_IsRunning(control) != 0U) ||
        (BallShuttleControl_ConfigIsValid(&control->config) == 0U)) {
        return 0U;
    }

    control->state = BALL_SHUTTLE_STATE_PHASE_1_NEGATIVE;
    control->failure = BALL_SHUTTLE_FAILURE_NONE;
    control->started_ms = now_ms;
    control->phase_started_ms = now_ms;
    control->elapsed_ms = 0U;
    control->target_pipe_pitch_deg =
        -control->config.angle_magnitude_deg[0];
    return 1U;
}

void BallShuttleControl_HoldLevel(BallShuttleControl *control,
                                  uint32_t now_ms)
{
    if (control == 0) {
        return;
    }

    if (control->started_ms != 0U) {
        control->elapsed_ms = (uint32_t) (now_ms - control->started_ms);
    }
    control->state = BALL_SHUTTLE_STATE_HOLDING_LEVEL;
    control->failure = BALL_SHUTTLE_FAILURE_NONE;
    control->phase_started_ms = now_ms;
    control->target_pipe_pitch_deg = 0.0f;
}

void BallShuttleControl_Stop(BallShuttleControl *control)
{
    if (control == 0) {
        return;
    }

    BallShuttleControl_ResetRuntime(control);
}

BallShuttleEvent BallShuttleControl_Update(
    BallShuttleControl *control,
    uint32_t now_ms,
    const BallShuttleMeasurement *measurement)
{
    uint32_t phase_elapsed_ms;

    if (control == 0) {
        return BALL_SHUTTLE_EVENT_NONE;
    }
    if ((control->state == BALL_SHUTTLE_STATE_IDLE) ||
        (control->state == BALL_SHUTTLE_STATE_FAILED)) {
        return BALL_SHUTTLE_EVENT_NONE;
    }
    if (BallShuttleControl_MeasurementIsHealthy(measurement) == 0U) {
        return BallShuttleControl_Fail(control);
    }

    control->elapsed_ms = (uint32_t) (now_ms - control->started_ms);
    phase_elapsed_ms = (uint32_t) (now_ms - control->phase_started_ms);

    switch (control->state) {
        case BALL_SHUTTLE_STATE_PHASE_1_NEGATIVE:
            if (phase_elapsed_ms >= control->config.duration_ms[0]) {
                control->state = BALL_SHUTTLE_STATE_PHASE_2_POSITIVE;
                control->phase_started_ms = now_ms;
                control->target_pipe_pitch_deg =
                    control->config.angle_magnitude_deg[1];
                return BALL_SHUTTLE_EVENT_PHASE_2;
            }
            break;

        case BALL_SHUTTLE_STATE_PHASE_2_POSITIVE:
            if (phase_elapsed_ms >= control->config.duration_ms[1]) {
                control->state = BALL_SHUTTLE_STATE_PHASE_3_NEGATIVE;
                control->phase_started_ms = now_ms;
                control->target_pipe_pitch_deg =
                    -control->config.angle_magnitude_deg[2];
                return BALL_SHUTTLE_EVENT_PHASE_3;
            }
            break;

        case BALL_SHUTTLE_STATE_PHASE_3_NEGATIVE:
            if (phase_elapsed_ms >= control->config.duration_ms[2]) {
                control->state = BALL_SHUTTLE_STATE_HOLDING_LEVEL;
                control->phase_started_ms = now_ms;
                control->target_pipe_pitch_deg = 0.0f;
                return BALL_SHUTTLE_EVENT_SUCCEEDED;
            }
            break;

        case BALL_SHUTTLE_STATE_HOLDING_LEVEL:
            break;

        default:
            return BallShuttleControl_Fail(control);
    }

    return BALL_SHUTTLE_EVENT_NONE;
}

float BallShuttleControl_GetTargetPipePitchDeg(
    const BallShuttleControl *control)
{
    return (control != 0) ? control->target_pipe_pitch_deg : 0.0f;
}

uint8_t BallShuttleControl_IsRunning(const BallShuttleControl *control)
{
    if (control == 0) {
        return 0U;
    }

    return (uint8_t) (
        (control->state == BALL_SHUTTLE_STATE_PHASE_1_NEGATIVE) ||
        (control->state == BALL_SHUTTLE_STATE_PHASE_2_POSITIVE) ||
        (control->state == BALL_SHUTTLE_STATE_PHASE_3_NEGATIVE));
}

uint8_t BallShuttleControl_IsHolding(const BallShuttleControl *control)
{
    return (uint8_t) ((control != 0) &&
                      (control->state ==
                       BALL_SHUTTLE_STATE_HOLDING_LEVEL));
}
