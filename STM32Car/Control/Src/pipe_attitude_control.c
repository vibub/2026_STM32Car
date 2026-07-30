#include "pipe_attitude_control.h"

#include <math.h>
#include <string.h>

#define PIPE_ATTITUDE_RAD_TO_DEG                 57.2957795f /* 弧度转换为角度的比例。 */
#define PIPE_ATTITUDE_SERVO_NEUTRAL_DEG          90.0f       /* 三点标定使用的舵机中位角。 */
#define PIPE_ATTITUDE_SERVO_NEGATIVE_DEG         88.0f       /* 三点标定负向舵机角。 */
#define PIPE_ATTITUDE_SERVO_POSITIVE_DEG         92.0f       /* 三点标定正向舵机角。 */
#define PIPE_ATTITUDE_LEVEL_SETTLE_MS           500U         /* 中位零偏采样前静置时间。 */
#define PIPE_ATTITUDE_POINT_SETTLE_MS           800U         /* 每个舵机标定点动作后的静置时间。 */
#define PIPE_ATTITUDE_LEVEL_SAMPLE_COUNT        180U         /* 水平零偏标定采样数量，约为 3 秒。 */
#define PIPE_ATTITUDE_POINT_SAMPLE_COUNT         30U         /* 单个齿轮标定点的平均采样数量。 */
#define PIPE_ATTITUDE_LEVEL_TIMEOUT_MS          5000U        /* 水平标定采样超时时间。 */
#define PIPE_ATTITUDE_POINT_TIMEOUT_MS          2500U        /* 单个齿轮标定点采样超时时间。 */
#define PIPE_ATTITUDE_MAX_REJECTED_SAMPLES       90U         /* 标定期间允许拒绝的最大运动样本数。 */
#define PIPE_ATTITUDE_FRESH_MS                  100U         /* 姿态数据允许的最大陈旧时间。 */
#define PIPE_ATTITUDE_FILTER_TAU_S                0.50f       /* 互补滤波低频校正时间常数。 */
#define PIPE_ATTITUDE_DT_MIN_S                    0.002f      /* 姿态滤波最小时间步长。 */
#define PIPE_ATTITUDE_DT_MAX_S                    0.100f      /* 姿态滤波最大有效时间步长。 */
#define PIPE_ATTITUDE_ACCEL_NORM_MIN_SQ            0.81f       /* Accel 可信时最小模长平方。 */
#define PIPE_ATTITUDE_ACCEL_NORM_MAX_SQ            1.21f       /* Accel 可信时最大模长平方。 */
#define PIPE_ATTITUDE_LEVEL_NORM_MIN_SQ            0.7225f     /* 静态标定允许的最小模长平方。 */
#define PIPE_ATTITUDE_LEVEL_NORM_MAX_SQ            1.3225f     /* 静态标定允许的最大模长平方。 */
#define PIPE_ATTITUDE_CHASSIS_ACCEL_GATE_CM_S2     50.0f       /* 禁用 Accel 校正的底盘加速度门槛。 */
#define PIPE_ATTITUDE_LEVEL_GYRO_LIMIT_DPS           3.0f      /* 水平零偏标定允许的最大 X 轴角速度。 */
#define PIPE_ATTITUDE_POINT_GYRO_LIMIT_DPS           2.0f      /* 三点角度平均允许的最大俯仰角速度。 */
#define PIPE_ATTITUDE_MIN_CALIBRATION_SPAN_DEG       0.40f     /* 88°与 92°标定点的最小俯仰跨度。 */
#define PIPE_ATTITUDE_MAX_RETURN_ERROR_DEG            0.60f     /* 回到 90°后允许的最大回差。 */
#define PIPE_ATTITUDE_MIN_SERVO_PER_PIPE             0.20f     /* 舵机/水管角比例绝对值下限。 */
#define PIPE_ATTITUDE_MAX_SERVO_PER_PIPE            10.0f      /* 舵机/水管角比例绝对值上限。 */

static float PipeAttitudeControl_Abs(float value)
{
    return (value < 0.0f) ? -value : value;
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
    return atan2f(measurement->accel_y_g,
                  measurement->accel_z_g) * PIPE_ATTITUDE_RAD_TO_DEG;
}

static uint8_t PipeAttitudeControl_IsActiveState(PipeCalibrationState state)
{
    return (uint8_t) ((state >= PIPE_CALIBRATION_SETTLE_LEVEL) &&
                      (state <= PIPE_CALIBRATION_SAMPLE_RETURN));
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
    control->sample_count = 0U;
    control->rejected_sample_count = 0U;
    control->sample_sum = 0.0f;
    if (state == PIPE_CALIBRATION_SAMPLE_LEVEL) {
        control->gyro_bias_sum = 0.0f;
        control->accel_pitch_sum = 0.0f;
    }
    PipeAttitudeControl_SetEvent(control, event);
}

static void PipeAttitudeControl_Fail(PipeAttitudeControl *control,
                                     uint32_t now_ms,
                                     PipeCalibrationEvent event)
{
    control->calibration_valid = 0U;
    PipeAttitudeControl_SetState(control,
                                 PIPE_CALIBRATION_FAILED,
                                 now_ms,
                                 event);
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
    control->accel_pitch_deg = PipeAttitudeControl_AccelPitchDeg(measurement) -
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
        return;
    }

    control->gyro_bias_sum += measurement->gyro_x_dps;
    control->accel_pitch_sum +=
        PipeAttitudeControl_AccelPitchDeg(measurement);
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
    control->level_calibrated = 1U;
    control->filter_ready = 1U;
    control->sample_ready = 1U;
    control->last_sample_ms = measurement->timestamp_ms;
    PipeAttitudeControl_SetState(control,
                                 PIPE_CALIBRATION_SETTLE_NEGATIVE,
                                 now_ms,
                                 PIPE_CALIBRATION_EVENT_NEGATIVE);
}

static void PipeAttitudeControl_CollectPointSample(
    PipeAttitudeControl *control,
    uint32_t now_ms)
{
    float average;

    if ((control->sample_ready == 0U) ||
        (control->accel_trusted == 0U) ||
        (PipeAttitudeControl_Abs(control->pitch_rate_dps) >
         PIPE_ATTITUDE_POINT_GYRO_LIMIT_DPS)) {
        if (control->rejected_sample_count < 0xFFFFU) {
            control->rejected_sample_count++;
        }
        return;
    }

    control->sample_sum += control->pitch_deg;
    control->sample_count++;
    if (control->sample_count < PIPE_ATTITUDE_POINT_SAMPLE_COUNT) {
        return;
    }

    average = control->sample_sum / (float) control->sample_count;
    switch (control->calibration_state) {
        case PIPE_CALIBRATION_SAMPLE_NEGATIVE:
            control->negative_pitch_deg = average;
            PipeAttitudeControl_SetState(control,
                                         PIPE_CALIBRATION_SETTLE_CENTER,
                                         now_ms,
                                         PIPE_CALIBRATION_EVENT_CENTER);
            break;

        case PIPE_CALIBRATION_SAMPLE_CENTER:
            control->center_pitch_deg = average;
            PipeAttitudeControl_SetState(control,
                                         PIPE_CALIBRATION_SETTLE_POSITIVE,
                                         now_ms,
                                         PIPE_CALIBRATION_EVENT_POSITIVE);
            break;

        case PIPE_CALIBRATION_SAMPLE_POSITIVE:
            control->positive_pitch_deg = average;
            PipeAttitudeControl_SetState(control,
                                         PIPE_CALIBRATION_SETTLE_RETURN,
                                         now_ms,
                                         PIPE_CALIBRATION_EVENT_RETURN);
            break;

        case PIPE_CALIBRATION_SAMPLE_RETURN:
            control->return_pitch_deg = average;
            break;

        default:
            break;
    }
}

static void PipeAttitudeControl_FinishMapping(PipeAttitudeControl *control,
                                              uint32_t now_ms)
{
    float span;
    float negative_relative;
    float positive_relative;
    float ratio;

    span = control->positive_pitch_deg - control->negative_pitch_deg;
    negative_relative = control->negative_pitch_deg -
                        control->center_pitch_deg;
    positive_relative = control->positive_pitch_deg -
                        control->center_pitch_deg;
    if ((PipeAttitudeControl_Abs(span) <
         PIPE_ATTITUDE_MIN_CALIBRATION_SPAN_DEG) ||
        ((negative_relative * positive_relative) >= 0.0f) ||
        (PipeAttitudeControl_Abs(control->return_pitch_deg -
                                 control->center_pitch_deg) >
         PIPE_ATTITUDE_MAX_RETURN_ERROR_DEG)) {
        PipeAttitudeControl_Fail(control,
                                 now_ms,
                                 PIPE_CALIBRATION_EVENT_FAILED_RANGE);
        return;
    }

    ratio = (PIPE_ATTITUDE_SERVO_POSITIVE_DEG -
             PIPE_ATTITUDE_SERVO_NEGATIVE_DEG) / span;
    if ((PipeAttitudeControl_Abs(ratio) <
         PIPE_ATTITUDE_MIN_SERVO_PER_PIPE) ||
        (PipeAttitudeControl_Abs(ratio) >
         PIPE_ATTITUDE_MAX_SERVO_PER_PIPE)) {
        PipeAttitudeControl_Fail(control,
                                 now_ms,
                                 PIPE_CALIBRATION_EVENT_FAILED_RANGE);
        return;
    }

    control->servo_deg_per_pipe_deg = ratio;
    control->calibration_valid = 1U;
    PipeAttitudeControl_SetState(control,
                                 PIPE_CALIBRATION_SUCCESS,
                                 now_ms,
                                 PIPE_CALIBRATION_EVENT_SUCCESS);
}

static float PipeAttitudeControl_GetCalibrationServoAngle(
    PipeCalibrationState state)
{
    if ((state == PIPE_CALIBRATION_SETTLE_NEGATIVE) ||
        (state == PIPE_CALIBRATION_SAMPLE_NEGATIVE)) {
        return PIPE_ATTITUDE_SERVO_NEGATIVE_DEG;
    }
    if ((state == PIPE_CALIBRATION_SETTLE_POSITIVE) ||
        (state == PIPE_CALIBRATION_SAMPLE_POSITIVE)) {
        return PIPE_ATTITUDE_SERVO_POSITIVE_DEG;
    }
    return PIPE_ATTITUDE_SERVO_NEUTRAL_DEG;
}

static void PipeAttitudeControl_FillOutput(PipeAttitudeControl *control,
                                           uint32_t now_ms,
                                           PipeAttitudeOutput *output)
{
    if (output == 0) {
        return;
    }

    memset(output, 0, sizeof(*output));
    if (control == 0) {
        return;
    }

    output->attitude_valid =
        (uint8_t) ((control->level_calibrated != 0U) &&
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
    output->calibration_state = control->calibration_state;
    output->calibration_event = control->pending_event;
    output->calibration_sample_count = control->sample_count;
    output->servo_override_deg = PipeAttitudeControl_GetCalibrationServoAngle(
        control->calibration_state);
    output->gyro_x_bias_dps = control->gyro_x_bias_dps;
    output->accel_pitch_zero_deg = control->accel_pitch_zero_deg;
    output->accel_pitch_deg = control->accel_pitch_deg;
    output->pitch_deg = control->pitch_deg;
    output->pitch_rate_dps = control->pitch_rate_dps;
    output->negative_pitch_deg = control->negative_pitch_deg;
    output->center_pitch_deg = control->center_pitch_deg;
    output->positive_pitch_deg = control->positive_pitch_deg;
    output->return_pitch_deg = control->return_pitch_deg;
    output->servo_deg_per_pipe_deg = control->servo_deg_per_pipe_deg;

    control->event_pending = 0U;
    control->pending_event = PIPE_CALIBRATION_EVENT_NONE;
}

void PipeAttitudeControl_Init(PipeAttitudeControl *control)
{
    if (control == 0) {
        return;
    }

    memset(control, 0, sizeof(*control));
    control->calibration_state = PIPE_CALIBRATION_IDLE;
}

uint8_t PipeAttitudeControl_StartCalibration(PipeAttitudeControl *control,
                                             uint32_t now_ms)
{
    if ((control == 0) ||
        (PipeAttitudeControl_IsActiveState(control->calibration_state) != 0U)) {
        return 0U;
    }

    PipeAttitudeControl_Init(control);
    PipeAttitudeControl_SetState(control,
                                 PIPE_CALIBRATION_SETTLE_LEVEL,
                                 now_ms,
                                 PIPE_CALIBRATION_EVENT_READY);
    return 1U;
}

void PipeAttitudeControl_AbortCalibration(PipeAttitudeControl *control)
{
    if ((control == 0) ||
        (PipeAttitudeControl_IsActiveState(control->calibration_state) == 0U)) {
        return;
    }

    control->calibration_valid = 0U;
    control->calibration_state = PIPE_CALIBRATION_IDLE;
    control->sample_count = 0U;
    PipeAttitudeControl_SetEvent(control, PIPE_CALIBRATION_EVENT_ABORTED);
}

uint8_t PipeAttitudeControl_IsCalibrationActive(
    const PipeAttitudeControl *control)
{
    if (control == 0) {
        return 0U;
    }
    return PipeAttitudeControl_IsActiveState(control->calibration_state);
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
    uint32_t state_elapsed_ms;

    if (control == 0) {
        PipeAttitudeControl_FillOutput(0, now_ms, output);
        return;
    }

    if ((measurement != 0) && (measurement->valid != 0U)) {
        if (control->calibration_state ==
            PIPE_CALIBRATION_SAMPLE_LEVEL) {
            PipeAttitudeControl_CollectLevelSample(control,
                                                   now_ms,
                                                   measurement);
        } else {
            PipeAttitudeControl_UpdateFusion(control,
                                             measurement,
                                             chassis_accel_cm_s2);
            if ((control->calibration_state ==
                 PIPE_CALIBRATION_SAMPLE_NEGATIVE) ||
                (control->calibration_state ==
                 PIPE_CALIBRATION_SAMPLE_CENTER) ||
                (control->calibration_state ==
                 PIPE_CALIBRATION_SAMPLE_POSITIVE) ||
                (control->calibration_state ==
                 PIPE_CALIBRATION_SAMPLE_RETURN)) {
                PipeAttitudeControl_CollectPointSample(control, now_ms);
            }
        }
    }

    state_elapsed_ms = (uint32_t) (now_ms - control->state_started_ms);
    switch (control->calibration_state) {
        case PIPE_CALIBRATION_SETTLE_LEVEL:
            if (state_elapsed_ms >= PIPE_ATTITUDE_LEVEL_SETTLE_MS) {
                PipeAttitudeControl_SetState(control,
                                             PIPE_CALIBRATION_SAMPLE_LEVEL,
                                             now_ms,
                                             PIPE_CALIBRATION_EVENT_LEVEL);
            }
            break;

        case PIPE_CALIBRATION_SAMPLE_LEVEL:
            if (state_elapsed_ms > PIPE_ATTITUDE_LEVEL_TIMEOUT_MS) {
                PipeAttitudeControl_Fail(
                    control,
                    now_ms,
                    (control->rejected_sample_count >
                     PIPE_ATTITUDE_MAX_REJECTED_SAMPLES) ?
                    PIPE_CALIBRATION_EVENT_FAILED_MOTION :
                    PIPE_CALIBRATION_EVENT_FAILED_IMU);
            }
            break;

        case PIPE_CALIBRATION_SETTLE_NEGATIVE:
            if (state_elapsed_ms >= PIPE_ATTITUDE_POINT_SETTLE_MS) {
                PipeAttitudeControl_SetState(
                    control,
                    PIPE_CALIBRATION_SAMPLE_NEGATIVE,
                    now_ms,
                    PIPE_CALIBRATION_EVENT_NONE);
            }
            break;

        case PIPE_CALIBRATION_SAMPLE_NEGATIVE:
        case PIPE_CALIBRATION_SAMPLE_CENTER:
        case PIPE_CALIBRATION_SAMPLE_POSITIVE:
            if (state_elapsed_ms > PIPE_ATTITUDE_POINT_TIMEOUT_MS) {
                PipeAttitudeControl_Fail(
                    control,
                    now_ms,
                    (control->rejected_sample_count >
                     PIPE_ATTITUDE_MAX_REJECTED_SAMPLES) ?
                    PIPE_CALIBRATION_EVENT_FAILED_MOTION :
                    PIPE_CALIBRATION_EVENT_FAILED_IMU);
            }
            break;

        case PIPE_CALIBRATION_SETTLE_CENTER:
            if (state_elapsed_ms >= PIPE_ATTITUDE_POINT_SETTLE_MS) {
                PipeAttitudeControl_SetState(
                    control,
                    PIPE_CALIBRATION_SAMPLE_CENTER,
                    now_ms,
                    PIPE_CALIBRATION_EVENT_NONE);
            }
            break;

        case PIPE_CALIBRATION_SETTLE_POSITIVE:
            if (state_elapsed_ms >= PIPE_ATTITUDE_POINT_SETTLE_MS) {
                PipeAttitudeControl_SetState(
                    control,
                    PIPE_CALIBRATION_SAMPLE_POSITIVE,
                    now_ms,
                    PIPE_CALIBRATION_EVENT_NONE);
            }
            break;

        case PIPE_CALIBRATION_SETTLE_RETURN:
            if (state_elapsed_ms >= PIPE_ATTITUDE_POINT_SETTLE_MS) {
                PipeAttitudeControl_SetState(
                    control,
                    PIPE_CALIBRATION_SAMPLE_RETURN,
                    now_ms,
                    PIPE_CALIBRATION_EVENT_NONE);
            }
            break;

        case PIPE_CALIBRATION_SAMPLE_RETURN:
            if (control->sample_count >=
                PIPE_ATTITUDE_POINT_SAMPLE_COUNT) {
                PipeAttitudeControl_FinishMapping(control, now_ms);
            } else if (state_elapsed_ms >
                       PIPE_ATTITUDE_POINT_TIMEOUT_MS) {
                PipeAttitudeControl_Fail(
                    control,
                    now_ms,
                    (control->rejected_sample_count >
                     PIPE_ATTITUDE_MAX_REJECTED_SAMPLES) ?
                    PIPE_CALIBRATION_EVENT_FAILED_MOTION :
                    PIPE_CALIBRATION_EVENT_FAILED_IMU);
            }
            break;

        default:
            break;
    }

    PipeAttitudeControl_FillOutput(control, now_ms, output);
}
