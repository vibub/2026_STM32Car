#include "chassis_motion_profile.h"

#include <string.h>

#define CHASSIS_MOTION_DT_MAX_S                 0.100f /* 单次更新允许使用的最大时间，避免任务阻塞后速度命令跳变。 */
#define CHASSIS_MOTION_SPEED_RESPONSE_TIME_S    0.250f /* 速度误差换算为期望加速度时使用的响应时间。 */
#define CHASSIS_MOTION_SPEED_TOLERANCE_CM_S     0.50f  /* 判定进入巡航状态的速度误差范围。 */
#define CHASSIS_MOTION_ACCEL_TOLERANCE_CM_S2    1.00f  /* 判定进入巡航状态的命令加速度范围。 */
#define CHASSIS_MOTION_STOP_ACCEL_EPSILON_CM_S2 0.01f  /* 停稳前让减速度按 jerk 自然回零，避免末端突变。 */

static float ChassisMotionProfile_Abs(float value)
{
    return (value < 0.0f) ? -value : value;
}

static float ChassisMotionProfile_Limit(float value,
                                        float minimum,
                                        float maximum)
{
    if (value > maximum) {
        return maximum;
    }
    if (value < minimum) {
        return minimum;
    }
    return value;
}

static void ChassisMotionProfile_FillOutput(
    const ChassisMotionProfile *profile,
    ChassisMotionOutput *output)
{
    if (output == 0) {
        return;
    }

    memset(output, 0, sizeof(*output));
    if (profile == 0) {
        return;
    }

    output->state = profile->state;
    output->active = profile->active;
    output->distance_reached = profile->distance_reached;
    output->stop_requested = profile->stop_requested;
    output->stopped = (uint8_t) (
        profile->state == CHASSIS_MOTION_STATE_STOPPED);
    output->target_speed_cm_s = profile->config.target_speed_cm_s;
    output->command_speed_cm_s = profile->command_speed_cm_s;
    output->command_accel_cm_s2 = profile->command_accel_cm_s2;
    output->travelled_distance_cm = profile->travelled_distance_cm;
}

void ChassisMotionProfile_Init(ChassisMotionProfile *profile)
{
    if (profile != 0) {
        memset(profile, 0, sizeof(*profile));
        profile->state = CHASSIS_MOTION_STATE_IDLE;
    }
}

uint8_t ChassisMotionProfile_Start(ChassisMotionProfile *profile,
                                   uint32_t now_ms,
                                   float initial_speed_cm_s,
                                   const ChassisMotionConfig *config)
{
    if ((profile == 0) || (config == 0) ||
        (config->target_speed_cm_s < 0.0f) ||
        (config->maximum_accel_cm_s2 <= 0.0f) ||
        (config->maximum_decel_cm_s2 <= 0.0f) ||
        (config->maximum_jerk_cm_s3 <= 0.0f) ||
        (config->completion_distance_cm < 0.0f)) {
        return 0U;
    }

    ChassisMotionProfile_Init(profile);
    profile->config = *config;
    profile->active = 1U;
    profile->time_ready = 1U;
    profile->last_update_ms = now_ms;
    profile->command_speed_cm_s =
        (initial_speed_cm_s > 0.0f) ? initial_speed_cm_s : 0.0f;
    profile->state =
        (ChassisMotionProfile_Abs(config->target_speed_cm_s -
                                  profile->command_speed_cm_s) <=
         CHASSIS_MOTION_SPEED_TOLERANCE_CM_S) ?
        CHASSIS_MOTION_STATE_CRUISING : CHASSIS_MOTION_STATE_RAMPING;
    return 1U;
}

void ChassisMotionProfile_Stop(ChassisMotionProfile *profile)
{
    ChassisMotionProfile_Init(profile);
}

void ChassisMotionProfile_SetTargetSpeed(ChassisMotionProfile *profile,
                                         float target_speed_cm_s)
{
    if ((profile == 0) || (profile->active == 0U)) {
        return;
    }

    profile->config.target_speed_cm_s =
        (target_speed_cm_s > 0.0f) ? target_speed_cm_s : 0.0f;
    if (profile->stop_requested == 0U) {
        profile->state = CHASSIS_MOTION_STATE_RAMPING;
    }
}

void ChassisMotionProfile_RequestStop(ChassisMotionProfile *profile)
{
    if ((profile == 0) || (profile->active == 0U)) {
        return;
    }

    profile->stop_requested = 1U;
    profile->config.target_speed_cm_s = 0.0f;
    profile->state = CHASSIS_MOTION_STATE_BRAKING;
}

void ChassisMotionProfile_Update(ChassisMotionProfile *profile,
                                 uint32_t now_ms,
                                 float distance_delta_cm,
                                 ChassisMotionOutput *output)
{
    uint32_t elapsed_ms;
    float dt_s;
    float speed_error;
    float desired_accel;
    float maximum_accel_delta;
    float next_speed;

    if ((profile == 0) || (profile->active == 0U)) {
        ChassisMotionProfile_FillOutput(profile, output);
        return;
    }

    if (distance_delta_cm > 0.0f) {
        profile->travelled_distance_cm += distance_delta_cm;
    }
    if ((profile->distance_reached == 0U) &&
        (profile->config.completion_distance_cm > 0.0f) &&
        (profile->travelled_distance_cm >=
         profile->config.completion_distance_cm)) {
        profile->distance_reached = 1U;
        profile->state = CHASSIS_MOTION_STATE_PASSED;
    }

    if (profile->time_ready == 0U) {
        profile->time_ready = 1U;
        profile->last_update_ms = now_ms;
        ChassisMotionProfile_FillOutput(profile, output);
        return;
    }

    elapsed_ms = (uint32_t) (now_ms - profile->last_update_ms);
    profile->last_update_ms = now_ms;
    dt_s = ChassisMotionProfile_Limit((float) elapsed_ms * 0.001f,
                                      0.0f,
                                      CHASSIS_MOTION_DT_MAX_S);
    if (dt_s <= 0.0f) {
        ChassisMotionProfile_FillOutput(profile, output);
        return;
    }

    speed_error = profile->config.target_speed_cm_s -
                  profile->command_speed_cm_s;
    desired_accel = speed_error / CHASSIS_MOTION_SPEED_RESPONSE_TIME_S;
    desired_accel = ChassisMotionProfile_Limit(
        desired_accel,
        -profile->config.maximum_decel_cm_s2,
        profile->config.maximum_accel_cm_s2);

    maximum_accel_delta = profile->config.maximum_jerk_cm_s3 * dt_s;
    profile->command_accel_cm_s2 += ChassisMotionProfile_Limit(
        desired_accel - profile->command_accel_cm_s2,
        -maximum_accel_delta,
        maximum_accel_delta);
    profile->command_accel_cm_s2 = ChassisMotionProfile_Limit(
        profile->command_accel_cm_s2,
        -profile->config.maximum_decel_cm_s2,
        profile->config.maximum_accel_cm_s2);

    next_speed = profile->command_speed_cm_s +
                 profile->command_accel_cm_s2 * dt_s;
    if (((speed_error > 0.0f) &&
         (next_speed > profile->config.target_speed_cm_s)) ||
        ((speed_error < 0.0f) &&
         (next_speed < profile->config.target_speed_cm_s))) {
        next_speed = profile->config.target_speed_cm_s;
        if (profile->stop_requested == 0U) {
            profile->command_accel_cm_s2 = 0.0f;
        }
    }
    profile->command_speed_cm_s =
        (next_speed > 0.0f) ? next_speed : 0.0f;

    if (profile->stop_requested != 0U) {
        if ((profile->command_speed_cm_s <=
             CHASSIS_MOTION_SPEED_TOLERANCE_CM_S) &&
            (ChassisMotionProfile_Abs(profile->command_accel_cm_s2) <=
             CHASSIS_MOTION_STOP_ACCEL_EPSILON_CM_S2)) {
            profile->command_speed_cm_s = 0.0f;
            profile->command_accel_cm_s2 = 0.0f;
            profile->state = CHASSIS_MOTION_STATE_STOPPED;
        } else {
            profile->state = CHASSIS_MOTION_STATE_BRAKING;
        }
    } else if (profile->distance_reached == 0U) {
        speed_error = profile->config.target_speed_cm_s -
                      profile->command_speed_cm_s;
        if ((ChassisMotionProfile_Abs(speed_error) <=
             CHASSIS_MOTION_SPEED_TOLERANCE_CM_S) &&
            (ChassisMotionProfile_Abs(profile->command_accel_cm_s2) <=
             CHASSIS_MOTION_ACCEL_TOLERANCE_CM_S2)) {
            profile->state = CHASSIS_MOTION_STATE_CRUISING;
        } else {
            profile->state = CHASSIS_MOTION_STATE_RAMPING;
        }
    }

    ChassisMotionProfile_FillOutput(profile, output);
}
