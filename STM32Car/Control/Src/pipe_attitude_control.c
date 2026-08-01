#include "pipe_attitude_control.h"

#include <math.h>
#include <string.h>

#define PIPE_ATTITUDE_RAD_TO_DEG                  57.2957795f /* 弧度转换为角度的比例。 */
#define PIPE_ATTITUDE_FIXED_IMU_PITCH_SIGN              (-1) /* PWM增大时原始IMU角减小，归一化后定义为正角。 */
#define PIPE_ATTITUDE_MIN_PIPE_PITCH_DEG          (-4.7423f) /* 500 us相对970 us水平点的水管角。 */
#define PIPE_ATTITUDE_MAX_PIPE_PITCH_DEG            6.3567f  /* 1600 us相对970 us水平点的水管角。 */
#define PIPE_ATTITUDE_LEVEL_SETTLE_MS                 500U   /* 到达人工确认零位PWM后的静置时间。 */
#define PIPE_ATTITUDE_LEVEL_SAMPLE_COUNT              180U   /* 水平零偏采样数量，约为3秒。 */
#define PIPE_ATTITUDE_LEVEL_TIMEOUT_MS               5000U   /* 单点水平采样最长等待时间。 */
#define PIPE_ATTITUDE_MAX_REJECTED_SAMPLES             90U   /* 标定期间允许拒绝的运动样本数。 */
#define PIPE_ATTITUDE_FRESH_MS                        100U   /* 姿态数据允许的最大陈旧时间。 */
#define PIPE_ATTITUDE_FILTER_TAU_S                    0.50f  /* 互补滤波低频校正时间常数。 */
#define PIPE_ATTITUDE_DT_MIN_S                       0.002f  /* 姿态滤波最小时间步长。 */
#define PIPE_ATTITUDE_DT_MAX_S                       0.100f  /* 姿态滤波和标定限速的最大时间步长。 */
#define PIPE_ATTITUDE_ACCEL_NORM_MIN_SQ               0.81f  /* Accel可信时最小模长平方。 */
#define PIPE_ATTITUDE_ACCEL_NORM_MAX_SQ               1.21f  /* Accel可信时最大模长平方。 */
#define PIPE_ATTITUDE_LEVEL_NORM_MIN_SQ              0.7225f /* 静态标定允许的最小模长平方。 */
#define PIPE_ATTITUDE_LEVEL_NORM_MAX_SQ              1.3225f /* 静态标定允许的最大模长平方。 */
#define PIPE_ATTITUDE_CHASSIS_ACCEL_GATE_CM_S2       50.0f   /* 禁用Accel校正的底盘加速度门槛。 */
#define PIPE_ATTITUDE_LEVEL_GYRO_LIMIT_DPS            3.0f   /* 水平零偏标定允许的最大角速度。 */
#define PIPE_ATTITUDE_CALIBRATION_SLEW_DPS            8.0f   /* 标定期间回水平的最大水管角速度。 */
#define PIPE_ATTITUDE_TARGET_EPSILON_DEG              0.03f  /* 判定水管角命令到位的软件误差。 */

static float PipeAttitudeControl_Abs(float value)
{
    return (value < 0.0f) ? -value : value;
}

static float PipeAttitudeControl_Limit(float value,
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

static float PipeAttitudeControl_Approach(float value,
                                          float target,
                                          float maximum_delta)
{
    if (value < target - maximum_delta) {
        return value + maximum_delta;
    }
    if (value > target + maximum_delta) {
        return value - maximum_delta;
    }
    return target;
}

static float PipeAttitudeControl_AccelNormSquared(
    const PipeAttitudeMeasurement *measurement)
{
    return measurement->accel_x_g * measurement->accel_x_g +
           measurement->accel_y_g * measurement->accel_y_g +
           measurement->accel_z_g * measurement->accel_z_g;
}

static float PipeAttitudeControl_AccelPitchDeg(
    const PipeAttitudeMeasurement *measurement)
{
    /* +Y朝钢球、+Z向上；固定安装方向由-1符号归一化为PWM增大时角度为正。 */
    return atan2f(measurement->accel_y_g,
                  measurement->accel_z_g) * PIPE_ATTITUDE_RAD_TO_DEG;
}

static uint8_t PipeAttitudeControl_IsActiveState(PipeCalibrationState state)
{
    return (uint8_t) ((state == PIPE_CALIBRATION_SETTLE_LEVEL) ||
                      (state == PIPE_CALIBRATION_SAMPLE_LEVEL) ||
                      (state == PIPE_CALIBRATION_RETURN_ABORTED) ||
                      (state == PIPE_CALIBRATION_RETURN_FAILED));
}

static void PipeAttitudeControl_SetEvent(PipeAttitudeControl *control,
                                         PipeCalibrationEvent event)
{
    control->pending_event = event;
    control->event_pending = (event != PIPE_CALIBRATION_EVENT_NONE) ? 1U : 0U;
}

static void PipeAttitudeControl_SetState(PipeAttitudeControl *control,
                                         PipeCalibrationState state,
                                         uint32_t now_ms,
                                         PipeCalibrationEvent event)
{
    control->calibration_state = state;
    control->state_started_ms = now_ms;
    control->settle_started_ms = 0U;
    control->settle_ready = 0U;
    control->sample_count = 0U;
    control->rejected_sample_count = 0U;
    control->sample_sum = 0.0f;
    if (state == PIPE_CALIBRATION_SAMPLE_LEVEL) {
        control->gyro_bias_sum = 0.0f;
        control->accel_pitch_sum = 0.0f;
    }
    PipeAttitudeControl_SetEvent(control, event);
}

static void PipeAttitudeControl_FillFixedMapping(PipeAttitudeControl *control)
{
    float span = PIPE_ATTITUDE_MAX_PIPE_PITCH_DEG -
                 PIPE_ATTITUDE_MIN_PIPE_PITCH_DEG;
    uint8_t index;

    for (index = 0U;
         index < LINKAGE_KINEMATICS_CALIBRATION_POINT_COUNT;
         index++) {
        control->mapping_pitch_deg[index] =
            PIPE_ATTITUDE_MIN_PIPE_PITCH_DEG +
            span * (float) index /
            (float) (LINKAGE_KINEMATICS_CALIBRATION_POINT_COUNT - 1U);
    }
    control->negative_pitch_deg = PIPE_ATTITUDE_MIN_PIPE_PITCH_DEG;
    control->center_pitch_deg = 0.0f;
    control->positive_pitch_deg = PIPE_ATTITUDE_MAX_PIPE_PITCH_DEG;
    control->return_pitch_deg = 0.0f;
    control->model_scale = 1.0f;
    control->model_offset_deg = 0.0f;
}

static uint8_t PipeAttitudeControl_LevelSampleIsStatic(
    const PipeAttitudeMeasurement *measurement)
{
    float norm_squared;

    if ((measurement == 0) || (measurement->valid == 0U)) {
        return 0U;
    }

    norm_squared = PipeAttitudeControl_AccelNormSquared(measurement);
    if ((norm_squared < PIPE_ATTITUDE_LEVEL_NORM_MIN_SQ) ||
        (norm_squared > PIPE_ATTITUDE_LEVEL_NORM_MAX_SQ)) {
        return 0U;
    }
    if ((PipeAttitudeControl_Abs(measurement->gyro_x_dps) >
         PIPE_ATTITUDE_LEVEL_GYRO_LIMIT_DPS) ||
        (PipeAttitudeControl_Abs(measurement->gyro_y_dps) >
         PIPE_ATTITUDE_LEVEL_GYRO_LIMIT_DPS) ||
        (PipeAttitudeControl_Abs(measurement->gyro_z_dps) >
         PIPE_ATTITUDE_LEVEL_GYRO_LIMIT_DPS)) {
        return 0U;
    }
    return 1U;
}

static void PipeAttitudeControl_UpdateFusion(
    PipeAttitudeControl *control,
    const PipeAttitudeMeasurement *measurement,
    float chassis_accel_cm_s2)
{
    uint32_t elapsed_ms;
    float dt_s;
    float norm_squared;
    float predicted_pitch;
    float alpha;

    if ((control->level_calibrated == 0U) ||
        (measurement == 0) || (measurement->valid == 0U)) {
        return;
    }

    control->pitch_rate_dps = measurement->gyro_x_dps -
                              control->gyro_x_bias_dps;
    control->accel_pitch_deg = control->raw_accel_pitch_deg -
                               control->accel_pitch_zero_deg;
    norm_squared = PipeAttitudeControl_AccelNormSquared(measurement);
    control->accel_trusted =
        (uint8_t) ((norm_squared >= PIPE_ATTITUDE_ACCEL_NORM_MIN_SQ) &&
                   (norm_squared <= PIPE_ATTITUDE_ACCEL_NORM_MAX_SQ) &&
                   (PipeAttitudeControl_Abs(chassis_accel_cm_s2) <=
                    PIPE_ATTITUDE_CHASSIS_ACCEL_GATE_CM_S2));

    if (control->sample_ready == 0U) {
        control->pitch_deg = (control->accel_trusted != 0U) ?
                             control->accel_pitch_deg : 0.0f;
        control->sample_ready = 1U;
        control->filter_ready = 1U;
        control->last_sample_ms = measurement->timestamp_ms;
        return;
    }

    elapsed_ms = (uint32_t) (measurement->timestamp_ms -
                             control->last_sample_ms);
    dt_s = (float) elapsed_ms * 0.001f;
    if ((dt_s < PIPE_ATTITUDE_DT_MIN_S) ||
        (dt_s > PIPE_ATTITUDE_DT_MAX_S)) {
        if (control->accel_trusted != 0U) {
            control->pitch_deg = control->accel_pitch_deg;
        }
        control->last_sample_ms = measurement->timestamp_ms;
        return;
    }

    predicted_pitch = control->pitch_deg + control->pitch_rate_dps * dt_s;
    if (control->accel_trusted != 0U) {
        alpha = PIPE_ATTITUDE_FILTER_TAU_S /
                (PIPE_ATTITUDE_FILTER_TAU_S + dt_s);
        control->pitch_deg = alpha * predicted_pitch +
                             (1.0f - alpha) * control->accel_pitch_deg;
    } else {
        control->pitch_deg = predicted_pitch;
    }
    control->last_sample_ms = measurement->timestamp_ms;
}

static void PipeAttitudeControl_UpdateServoOverride(
    PipeAttitudeControl *control,
    uint32_t now_ms)
{
    uint32_t elapsed_ms;
    float dt_s;
    float maximum_delta;

    if (control->task_time_ready != 0U) {
        elapsed_ms = (uint32_t) (now_ms - control->last_task_ms);
        dt_s = PipeAttitudeControl_Limit((float) elapsed_ms * 0.001f,
                                        0.0f,
                                        PIPE_ATTITUDE_DT_MAX_S);
        maximum_delta = PIPE_ATTITUDE_CALIBRATION_SLEW_DPS * dt_s;
        if (maximum_delta > 0.0f) {
            control->servo_override_deg = PipeAttitudeControl_Approach(
                control->servo_override_deg,
                control->servo_target_deg,
                maximum_delta);
        }
    } else {
        control->task_time_ready = 1U;
    }
    control->last_task_ms = now_ms;
}

static uint8_t PipeAttitudeControl_TargetReached(
    const PipeAttitudeControl *control)
{
    return (uint8_t) (PipeAttitudeControl_Abs(
        control->servo_override_deg - control->servo_target_deg) <=
        PIPE_ATTITUDE_TARGET_EPSILON_DEG);
}

static uint8_t PipeAttitudeControl_SettleElapsed(
    PipeAttitudeControl *control,
    uint32_t now_ms)
{
    if (PipeAttitudeControl_TargetReached(control) == 0U) {
        control->settle_ready = 0U;
        return 0U;
    }
    if (control->settle_ready == 0U) {
        control->settle_ready = 1U;
        control->settle_started_ms = now_ms;
        return 0U;
    }
    return (uint8_t) ((uint32_t) (now_ms - control->settle_started_ms) >=
                      PIPE_ATTITUDE_LEVEL_SETTLE_MS);
}

static void PipeAttitudeControl_CollectLevelSample(
    PipeAttitudeControl *control,
    uint32_t now_ms,
    const PipeAttitudeMeasurement *measurement)
{
    if (measurement == 0) {
        return;
    }
    if (PipeAttitudeControl_LevelSampleIsStatic(measurement) == 0U) {
        if (control->rejected_sample_count < 0xFFFFU) {
            control->rejected_sample_count++;
        }
        if (control->rejected_sample_count >
            PIPE_ATTITUDE_MAX_REJECTED_SAMPLES) {
            control->calibration_valid = 0U;
            control->servo_target_deg = 0.0f;
            PipeAttitudeControl_SetState(
                control,
                PIPE_CALIBRATION_RETURN_FAILED,
                now_ms,
                PIPE_CALIBRATION_EVENT_FAILED_MOTION);
        }
        return;
    }

    control->gyro_bias_sum += measurement->gyro_x_dps;
    control->accel_pitch_sum += control->raw_accel_pitch_deg;
    control->sample_count++;
    if (control->sample_count < PIPE_ATTITUDE_LEVEL_SAMPLE_COUNT) {
        return;
    }

    control->gyro_x_bias_dps = control->gyro_bias_sum /
                               (float) control->sample_count;
    control->accel_pitch_zero_deg = control->accel_pitch_sum /
                                    (float) control->sample_count;
    control->accel_pitch_deg = 0.0f;
    control->pitch_deg = 0.0f;
    control->pitch_rate_dps = 0.0f;
    control->imu_pitch_sign = PIPE_ATTITUDE_FIXED_IMU_PITCH_SIGN;
    control->level_calibrated = 1U;
    control->calibration_valid = 1U;
    control->filter_ready = 1U;
    control->sample_ready = 1U;
    control->last_sample_ms = measurement->timestamp_ms;
    PipeAttitudeControl_FillFixedMapping(control);
    PipeAttitudeControl_SetState(control,
                                 PIPE_CALIBRATION_SUCCESS,
                                 now_ms,
                                 PIPE_CALIBRATION_EVENT_SUCCESS);
}

static void PipeAttitudeControl_FillOutput(PipeAttitudeControl *control,
                                           uint32_t now_ms,
                                           PipeAttitudeOutput *output)
{
    float sign = 1.0f;

    if (output == 0) {
        return;
    }

    memset(output, 0, sizeof(*output));
    if (control == 0) {
        return;
    }

    if ((control->calibration_valid != 0U) &&
        (control->imu_pitch_sign != 0)) {
        sign = (float) control->imu_pitch_sign;
    }
    output->attitude_valid =
        (uint8_t) ((control->calibration_valid != 0U) &&
                   (control->sample_ready != 0U) &&
                   ((uint32_t) (now_ms - control->last_sample_ms) <=
                    PIPE_ATTITUDE_FRESH_MS));
    output->accel_trusted = control->accel_trusted;
    output->level_calibrated = control->level_calibrated;
    output->calibration_active =
        PipeAttitudeControl_IsActiveState(control->calibration_state);
    output->calibration_valid = control->calibration_valid;
    output->servo_override_active = output->calibration_active;
    output->status_changed = control->event_pending;
    output->imu_pitch_sign = control->imu_pitch_sign;
    output->calibration_state = control->calibration_state;
    output->calibration_event = control->pending_event;
    output->calibration_point_index = 0U;
    output->calibration_sample_count = control->sample_count;
    /* 兼容字段现在保存相对水管角，App通过线性公式换算为PWM。 */
    output->servo_override_deg = control->servo_override_deg;
    output->servo_target_deg = control->servo_target_deg;
    output->calibration_target_pitch_deg = 0.0f;
    output->gyro_x_bias_dps = control->gyro_x_bias_dps;
    output->accel_pitch_zero_deg = control->accel_pitch_zero_deg;
    output->raw_accel_pitch_deg = control->raw_accel_pitch_deg;
    output->raw_pitch_deg = control->pitch_deg;
    output->raw_pitch_rate_dps = control->pitch_rate_dps;
    output->accel_pitch_deg = sign * control->accel_pitch_deg;
    output->pitch_deg = sign * control->pitch_deg;
    output->pitch_rate_dps = sign * control->pitch_rate_dps;
    output->negative_pitch_deg = control->negative_pitch_deg;
    output->center_pitch_deg = control->center_pitch_deg;
    output->positive_pitch_deg = control->positive_pitch_deg;
    output->return_pitch_deg = control->return_pitch_deg;
    output->model_scale = control->model_scale;
    output->model_offset_deg = control->model_offset_deg;
    memcpy(output->mapping_pitch_deg,
           control->mapping_pitch_deg,
           sizeof(output->mapping_pitch_deg));

    control->event_pending = 0U;
    control->pending_event = PIPE_CALIBRATION_EVENT_NONE;
}

void PipeAttitudeControl_Init(PipeAttitudeControl *control)
{
    if (control == 0) {
        return;
    }

    memset(control, 0, sizeof(*control));
    control->imu_pitch_sign = PIPE_ATTITUDE_FIXED_IMU_PITCH_SIGN;
    control->servo_target_deg = 0.0f;
    control->servo_override_deg = 0.0f;
    PipeAttitudeControl_FillFixedMapping(control);
}

uint8_t PipeAttitudeControl_StartCalibration(PipeAttitudeControl *control,
                                             uint32_t now_ms,
                                             float current_pipe_pitch_deg)
{
    if ((control == 0) ||
        (PipeAttitudeControl_IsActiveState(control->calibration_state) != 0U)) {
        return 0U;
    }

    control->calibration_valid = 0U;
    control->level_calibrated = 0U;
    control->filter_ready = 0U;
    control->sample_ready = 0U;
    control->accel_trusted = 0U;
    control->imu_pitch_sign = PIPE_ATTITUDE_FIXED_IMU_PITCH_SIGN;
    control->servo_override_deg = PipeAttitudeControl_Limit(
        current_pipe_pitch_deg,
        PIPE_ATTITUDE_MIN_PIPE_PITCH_DEG,
        PIPE_ATTITUDE_MAX_PIPE_PITCH_DEG);
    control->servo_target_deg = 0.0f;
    control->calibration_target_pitch_deg = 0.0f;
    control->task_time_ready = 0U;
    PipeAttitudeControl_SetState(control,
                                 PIPE_CALIBRATION_SETTLE_LEVEL,
                                 now_ms,
                                 PIPE_CALIBRATION_EVENT_READY);
    return 1U;
}

void PipeAttitudeControl_AbortCalibration(PipeAttitudeControl *control,
                                          uint32_t now_ms)
{
    if ((control == 0) ||
        (PipeAttitudeControl_IsActiveState(control->calibration_state) == 0U)) {
        return;
    }

    control->calibration_valid = 0U;
    control->level_calibrated = 0U;
    control->servo_target_deg = 0.0f;
    PipeAttitudeControl_SetState(control,
                                 PIPE_CALIBRATION_RETURN_ABORTED,
                                 now_ms,
                                 PIPE_CALIBRATION_EVENT_ABORTED);
}

uint8_t PipeAttitudeControl_IsCalibrationActive(
    const PipeAttitudeControl *control)
{
    return (control != 0) ?
           PipeAttitudeControl_IsActiveState(control->calibration_state) : 0U;
}

uint8_t PipeAttitudeControl_IsCalibrationValid(
    const PipeAttitudeControl *control)
{
    return (control != 0) ? control->calibration_valid : 0U;
}

void PipeAttitudeControl_Update(PipeAttitudeControl *control,
                                uint32_t now_ms,
                                const PipeAttitudeMeasurement *measurement,
                                float chassis_accel_cm_s2,
                                PipeAttitudeOutput *output)
{
    if (control == 0) {
        PipeAttitudeControl_FillOutput(0, now_ms, output);
        return;
    }

    if ((measurement != 0) && (measurement->valid != 0U)) {
        control->raw_accel_pitch_deg =
            PipeAttitudeControl_AccelPitchDeg(measurement);
        if (control->calibration_valid != 0U) {
            PipeAttitudeControl_UpdateFusion(control,
                                             measurement,
                                             chassis_accel_cm_s2);
        }
    }

    if (PipeAttitudeControl_IsActiveState(control->calibration_state) != 0U) {
        PipeAttitudeControl_UpdateServoOverride(control, now_ms);
    }

    switch (control->calibration_state) {
        case PIPE_CALIBRATION_SETTLE_LEVEL:
            if ((uint32_t) (now_ms - control->state_started_ms) >
                PIPE_ATTITUDE_LEVEL_TIMEOUT_MS) {
                control->calibration_valid = 0U;
                control->servo_target_deg = 0.0f;
                PipeAttitudeControl_SetState(
                    control,
                    PIPE_CALIBRATION_RETURN_FAILED,
                    now_ms,
                    PIPE_CALIBRATION_EVENT_FAILED_IMU);
            } else if (PipeAttitudeControl_SettleElapsed(control,
                                                         now_ms) != 0U) {
                PipeAttitudeControl_SetState(control,
                                             PIPE_CALIBRATION_SAMPLE_LEVEL,
                                             now_ms,
                                             PIPE_CALIBRATION_EVENT_LEVEL);
            }
            break;

        case PIPE_CALIBRATION_SAMPLE_LEVEL:
            if ((uint32_t) (now_ms - control->state_started_ms) >
                PIPE_ATTITUDE_LEVEL_TIMEOUT_MS) {
                control->calibration_valid = 0U;
                control->servo_target_deg = 0.0f;
                PipeAttitudeControl_SetState(
                    control,
                    PIPE_CALIBRATION_RETURN_FAILED,
                    now_ms,
                    PIPE_CALIBRATION_EVENT_FAILED_IMU);
            } else {
                PipeAttitudeControl_CollectLevelSample(control,
                                                       now_ms,
                                                       measurement);
            }
            break;

        case PIPE_CALIBRATION_RETURN_ABORTED:
            if (PipeAttitudeControl_TargetReached(control) != 0U) {
                PipeAttitudeControl_SetState(control,
                                             PIPE_CALIBRATION_IDLE,
                                             now_ms,
                                             PIPE_CALIBRATION_EVENT_NONE);
            }
            break;

        case PIPE_CALIBRATION_RETURN_FAILED:
            if (PipeAttitudeControl_TargetReached(control) != 0U) {
                PipeAttitudeControl_SetState(control,
                                             PIPE_CALIBRATION_FAILED,
                                             now_ms,
                                             PIPE_CALIBRATION_EVENT_NONE);
            }
            break;

        default:
            break;
    }

    PipeAttitudeControl_FillOutput(control, now_ms, output);
}
