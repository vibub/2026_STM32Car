#include "wheel_speed_pid.h"
#include <string.h>

#define SPEED_TARGET_LIMIT_CM_S          60.0f
#define SPEED_PID_KP                      0.0050f
#define SPEED_PID_KI                      0.0010f
#define SPEED_PID_KD                      0.0005f
#define SPEED_PID_INTEGRAL_LIMIT        500.0f
#define SPEED_PID_INTEGRAL_ERROR_LIMIT   50.0f
#define SPEED_PID_OUTPUT_LIMIT            0.50f
#define SPEED_STARTUP_KICK_MIN_CM_S        5.0f
#define SPEED_STARTUP_KICK_PWM             0.20f
#define SPEED_STARTUP_KICK_WINDOWS         5U
#define SPEED_STALL_MIN_CM_S               5.0f
#define SPEED_STALL_MIN_PWM                0.18f
#define SPEED_STALL_FEEDBACK_CM_S          0.30f
#define SPEED_STALL_WINDOWS               50U

static float SpeedPid_Abs(float value)
{
    return (value < 0.0f) ? -value : value;
}

static float SpeedPid_Limit(float value, float minimum, float maximum)
{
    if (value > maximum) {
        return maximum;
    }
    if (value < minimum) {
        return minimum;
    }
    return value;
}

void WheelSpeedPid_Init(WheelSpeedPid *pid)
{
    WheelSpeedPid_Reset(pid);
}

void WheelSpeedPid_Reset(WheelSpeedPid *pid)
{
    if (pid != 0) {
        memset(pid, 0, sizeof(*pid));
    }
}

void WheelSpeedPid_SetTarget(WheelSpeedPid *pid, float target_cm_s)
{
    float previous;

    if (pid == 0) {
        return;
    }

    previous = pid->target_cm_s;
    target_cm_s = SpeedPid_Limit(target_cm_s,
                                 -SPEED_TARGET_LIMIT_CM_S,
                                 SPEED_TARGET_LIMIT_CM_S);
    if (SpeedPid_Abs(target_cm_s) < 0.1f) {
        target_cm_s = 0.0f;
    }

    if ((target_cm_s == 0.0f) || ((target_cm_s * previous) < 0.0f)) {
        pid->integral = 0.0f;
        pid->last_error = 0.0f;
    }
    if ((SpeedPid_Abs(target_cm_s) >= SPEED_STARTUP_KICK_MIN_CM_S) &&
        ((SpeedPid_Abs(previous) < 0.1f) || ((target_cm_s * previous) < 0.0f))) {
        pid->startup_kick_windows = SPEED_STARTUP_KICK_WINDOWS;
    } else if (target_cm_s == 0.0f) {
        pid->startup_kick_windows = 0U;
    }

    pid->target_cm_s = target_cm_s;
}

float WheelSpeedPid_Step(WheelSpeedPid *pid, float measured_cm_s)
{
    float error;
    float candidate_integral;
    float candidate_output;

    if (pid == 0) {
        return 0.0f;
    }

    if (SpeedPid_Abs(pid->target_cm_s) < 0.1f) {
        pid->integral = 0.0f;
        pid->last_error = 0.0f;
        pid->command = 0.0f;
        pid->p_term = 0.0f;
        pid->i_term = 0.0f;
        pid->d_term = 0.0f;
        pid->zero_feedback_windows = 0U;
        pid->fault = 0U;
        return 0.0f;
    }

    error = pid->target_cm_s - measured_cm_s;
    pid->p_term = SPEED_PID_KP * error;
    pid->d_term = SPEED_PID_KD * (error - pid->last_error);
    pid->last_error = error;

    candidate_integral = SpeedPid_Limit(pid->integral + error,
                                        -SPEED_PID_INTEGRAL_LIMIT,
                                        SPEED_PID_INTEGRAL_LIMIT);
    candidate_output = pid->p_term +
                       SPEED_PID_KI * candidate_integral +
                       pid->d_term;
    if ((SpeedPid_Abs(error) < SPEED_PID_INTEGRAL_ERROR_LIMIT) &&
        ((SpeedPid_Abs(candidate_output) < SPEED_PID_OUTPUT_LIMIT) ||
         ((candidate_output > SPEED_PID_OUTPUT_LIMIT) && (error < 0.0f)) ||
         ((candidate_output < -SPEED_PID_OUTPUT_LIMIT) && (error > 0.0f)))) {
        pid->integral = candidate_integral;
    }

    pid->i_term = SPEED_PID_KI * pid->integral;
    pid->command = SpeedPid_Limit(pid->p_term + pid->i_term + pid->d_term,
                                  -SPEED_PID_OUTPUT_LIMIT,
                                  SPEED_PID_OUTPUT_LIMIT);

    if ((pid->startup_kick_windows > 0U) &&
        (SpeedPid_Abs(measured_cm_s) < SPEED_STALL_FEEDBACK_CM_S)) {
        pid->command = (pid->target_cm_s > 0.0f) ?
                       SPEED_STARTUP_KICK_PWM : -SPEED_STARTUP_KICK_PWM;
        pid->startup_kick_windows--;
    } else {
        pid->startup_kick_windows = 0U;
    }

    if ((SpeedPid_Abs(pid->target_cm_s) >= SPEED_STALL_MIN_CM_S) &&
        (SpeedPid_Abs(pid->command) >= SPEED_STALL_MIN_PWM) &&
        (SpeedPid_Abs(measured_cm_s) < SPEED_STALL_FEEDBACK_CM_S)) {
        if (pid->zero_feedback_windows < 0xFFFFU) {
            pid->zero_feedback_windows++;
        }
        if (pid->zero_feedback_windows >= SPEED_STALL_WINDOWS) {
            pid->fault = 1U;
        }
    } else {
        pid->zero_feedback_windows = 0U;
        pid->fault = 0U;
    }

    return pid->command;
}

uint8_t WheelSpeedPid_HasFault(const WheelSpeedPid *pid)
{
    return (pid != 0) ? pid->fault : 1U;
}
