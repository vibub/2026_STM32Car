#include "pipe_attitude_control.h"

#include "linkage_kinematics.h"

#include <math.h>
#include <string.h>

#define PIPE_ATTITUDE_RAD_TO_DEG                  57.2957795f /* 弧度转换为角度的比例。 */
#define PIPE_ATTITUDE_CALIBRATION_NEGATIVE_DEG   -10.0f      /* 与负向几何死点保留余量的标定角。 */
#define PIPE_ATTITUDE_CALIBRATION_CENTER_DEG       0.0f      /* 三点标定使用的水平模型目标角。 */
#define PIPE_ATTITUDE_CALIBRATION_POSITIVE_DEG     6.8f      /* 留出几何死点余量的正向模型标定角。 */
#define PIPE_ATTITUDE_LEVEL_SETTLE_MS            500U        /* 水平零偏采样前的静置时间。 */
#define PIPE_ATTITUDE_POINT_SETTLE_MS            800U        /* 到达模型标定点后的静置时间。 */
#define PIPE_ATTITUDE_LEVEL_SAMPLE_COUNT         180U        /* 水平零偏标定采样数量，约为 3 秒。 */
#define PIPE_ATTITUDE_POINT_SAMPLE_COUNT          30U        /* 单个模型标定点的平均采样数量。 */
#define PIPE_ATTITUDE_LEVEL_TIMEOUT_MS           5000U       /* 水平标定采样超时时间。 */
#define PIPE_ATTITUDE_POINT_TIMEOUT_MS           2500U       /* 单个模型标定点采样超时时间。 */
#define PIPE_ATTITUDE_MAX_REJECTED_SAMPLES        90U        /* 标定期间允许拒绝的最大运动样本数。 */
#define PIPE_ATTITUDE_FRESH_MS                   100U        /* 姿态数据允许的最大陈旧时间。 */
#define PIPE_ATTITUDE_FILTER_TAU_S                 0.50f     /* 互补滤波低频校正时间常数。 */
#define PIPE_ATTITUDE_DT_MIN_S                     0.002f    /* 姿态滤波最小时间步长。 */
#define PIPE_ATTITUDE_DT_MAX_S                     0.100f    /* 姿态滤波和标定限速的最大时间步长。 */
#define PIPE_ATTITUDE_ACCEL_NORM_MIN_SQ             0.81f     /* Accel 可信时最小模长平方。 */
#define PIPE_ATTITUDE_ACCEL_NORM_MAX_SQ             1.21f     /* Accel 可信时最大模长平方。 */
#define PIPE_ATTITUDE_LEVEL_NORM_MIN_SQ             0.7225f   /* 静态标定允许的最小模长平方。 */
#define PIPE_ATTITUDE_LEVEL_NORM_MAX_SQ             1.3225f   /* 静态标定允许的最大模长平方。 */
#define PIPE_ATTITUDE_CHASSIS_ACCEL_GATE_CM_S2      50.0f     /* 禁用 Accel 校正的底盘加速度门槛。 */
#define PIPE_ATTITUDE_LEVEL_GYRO_LIMIT_DPS            3.0f    /* 水平零偏标定允许的最大角速度。 */
#define PIPE_ATTITUDE_POINT_GYRO_LIMIT_DPS            2.0f    /* 标定点平均允许的最大俯仰角速度。 */
#define PIPE_ATTITUDE_MODEL_SCALE_MIN                  0.35f   /* 柔性水管模型校正比例允许的最小值。 */
#define PIPE_ATTITUDE_MODEL_SCALE_MAX                  1.25f   /* 模型校正比例允许的最大值。 */
#define PIPE_ATTITUDE_MODEL_OFFSET_MAX_DEG             0.75f   /* 模型校正零偏允许的最大绝对值。 */
#define PIPE_ATTITUDE_MAX_RETURN_ERROR_DEG             0.60f   /* 回到模型水平点后允许的最大回差。 */
#define PIPE_ATTITUDE_MAPPING_MIN_STEP_DEG              0.05f   /* 相邻实测点允许的最小单调角度差。 */
#define PIPE_ATTITUDE_CALIBRATION_SERVO_SLEW_DEG_S    30.0f   /* 标定期间舵机命令最大变化率。 */
#define PIPE_ATTITUDE_SERVO_TARGET_EPSILON_DEG          0.01f  /* 判定软件舵机命令到位的角度误差。 */
#define PIPE_ATTITUDE_SERVO_SAFE_MIN_DEG                0.0f   /* 重新装配后允许的最小舵机内部角。 */
#define PIPE_ATTITUDE_SERVO_SAFE_MAX_DEG              180.0f   /* 重新装配后允许的最大舵机内部角。 */

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
    /* 远端安装仍保持 +Y 朝钢球、+Z 向上；水平转向造成的符号反转由三点标定自动识别。 */
    return atan2f(measurement->accel_y_g,
                  measurement->accel_z_g) * PIPE_ATTITUDE_RAD_TO_DEG;
}

static uint8_t PipeAttitudeControl_IsActiveState(PipeCalibrationState state)
{
    return (uint8_t) ((state >= PIPE_CALIBRATION_SETTLE_LEVEL) &&
                      (state <= PIPE_CALIBRATION_RETURN_FAILED));
}

static uint8_t PipeAttitudeControl_IsPointSampleState(
    PipeCalibrationState state)
{
    return (uint8_t) ((state == PIPE_CALIBRATION_SAMPLE_MAPPING_POINT) ||
                      (state == PIPE_CALIBRATION_SAMPLE_RETURN));
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

static uint8_t PipeAttitudeControl_SetServoTarget(
    PipeAttitudeControl *control,
    float target_pipe_pitch_deg)
{
    float servo_angle_deg;

    if (LinkageKinematics_PipePitchToServoAngle(target_pipe_pitch_deg,
                                                 &servo_angle_deg) == 0U) {
        return 0U;
    }
    if ((servo_angle_deg < PIPE_ATTITUDE_SERVO_SAFE_MIN_DEG) ||
        (servo_angle_deg > PIPE_ATTITUDE_SERVO_SAFE_MAX_DEG)) {
        return 0U;
    }

    control->calibration_target_pitch_deg = target_pipe_pitch_deg;
    control->servo_target_deg = servo_angle_deg;
    return 1U;
}

static uint8_t PipeAttitudeControl_SetPointState(
    PipeAttitudeControl *control,
    PipeCalibrationState state,
    float target_pipe_pitch_deg,
    uint32_t now_ms,
    PipeCalibrationEvent event)
{
    if (PipeAttitudeControl_SetServoTarget(control,
                                           target_pipe_pitch_deg) == 0U) {
        return 0U;
    }
    PipeAttitudeControl_SetState(control, state, now_ms, event);
    return 1U;
}

static uint8_t PipeAttitudeControl_SetMappingPointState(
    PipeAttitudeControl *control,
    uint8_t point_index,
    uint32_t now_ms)
{
    float model_pitch_deg;
    float servo_angle_deg;

    if ((LinkageKinematics_GetCalibrationPoint(point_index,
                                               &model_pitch_deg,
                                               &servo_angle_deg) == 0U) ||
        (servo_angle_deg < PIPE_ATTITUDE_SERVO_SAFE_MIN_DEG) ||
        (servo_angle_deg > PIPE_ATTITUDE_SERVO_SAFE_MAX_DEG)) {
        return 0U;
    }

    control->calibration_point_index = point_index;
    control->calibration_target_pitch_deg = model_pitch_deg;
    control->servo_target_deg = servo_angle_deg;
    PipeAttitudeControl_SetState(control,
                                 PIPE_CALIBRATION_SETTLE_MAPPING_POINT,
                                 now_ms,
                                 PIPE_CALIBRATION_EVENT_MAPPING_POINT);
    return 1U;
}

static void PipeAttitudeControl_Fail(PipeAttitudeControl *control,
                                     uint32_t now_ms,
                                     PipeCalibrationEvent event)
{
    control->calibration_valid = 0U;
    if (PipeAttitudeControl_SetServoTarget(
            control, PIPE_ATTITUDE_CALIBRATION_CENTER_DEG) == 0U) {
        control->servo_target_deg =
            LinkageKinematics_GetNominalServoCenterDeg();
    }
    PipeAttitudeControl_SetState(control,
                                 PIPE_CALIBRATION_RETURN_FAILED,
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
    control->level_calibrated = 1U;
    control->filter_ready = 1U;
    control->sample_ready = 1U;
    control->last_sample_ms = measurement->timestamp_ms;
    if (PipeAttitudeControl_SetMappingPointState(control,
                                                 0U,
                                                 now_ms) == 0U) {
        PipeAttitudeControl_Fail(control,
                                 now_ms,
                                 PIPE_CALIBRATION_EVENT_FAILED_RANGE);
    }
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

    /* 标定点已经静止，直接采集重力角，避免互补滤波滞后低估实际行程。 */
    control->sample_sum += control->accel_pitch_deg;
    control->sample_count++;
    if (control->sample_count < PIPE_ATTITUDE_POINT_SAMPLE_COUNT) {
        return;
    }

    average = control->sample_sum / (float) control->sample_count;
    switch (control->calibration_state) {
        case PIPE_CALIBRATION_SAMPLE_MAPPING_POINT:
            if (control->calibration_point_index >=
                LINKAGE_KINEMATICS_CALIBRATION_POINT_COUNT) {
                PipeAttitudeControl_Fail(
                    control, now_ms, PIPE_CALIBRATION_EVENT_FAILED_RANGE);
                break;
            }
            control->mapping_pitch_deg[
                control->calibration_point_index] = average;
            if ((uint8_t) (control->calibration_point_index + 1U) <
                LINKAGE_KINEMATICS_CALIBRATION_POINT_COUNT) {
                if (PipeAttitudeControl_SetMappingPointState(
                        control,
                        (uint8_t) (control->calibration_point_index + 1U),
                        now_ms) == 0U) {
                    PipeAttitudeControl_Fail(
                        control, now_ms,
                        PIPE_CALIBRATION_EVENT_FAILED_RANGE);
                }
            } else {
                control->negative_pitch_deg = control->mapping_pitch_deg[0];
                control->center_pitch_deg = control->mapping_pitch_deg[
                    LINKAGE_KINEMATICS_LEVEL_POINT_INDEX];
                control->positive_pitch_deg = control->mapping_pitch_deg[
                    LINKAGE_KINEMATICS_CALIBRATION_POINT_COUNT - 1U];
                if (PipeAttitudeControl_SetPointState(
                        control,
                        PIPE_CALIBRATION_SETTLE_RETURN,
                        PIPE_ATTITUDE_CALIBRATION_CENTER_DEG,
                        now_ms,
                        PIPE_CALIBRATION_EVENT_RETURN) == 0U) {
                    PipeAttitudeControl_Fail(
                        control, now_ms,
                        PIPE_CALIBRATION_EVENT_FAILED_RANGE);
                }
            }
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
    float raw_span;
    float normalized_span;
    float return_error;
    float first_model_pitch_deg;
    float last_model_pitch_deg;
    float servo_angle_deg;
    uint8_t index;

    raw_span = control->mapping_pitch_deg[
        LINKAGE_KINEMATICS_CALIBRATION_POINT_COUNT - 1U] -
        control->mapping_pitch_deg[0];
    if (PipeAttitudeControl_Abs(raw_span) < 0.01f) {
        PipeAttitudeControl_Fail(control,
                                 now_ms,
                                 PIPE_CALIBRATION_EVENT_FAILED_RANGE);
        return;
    }

    control->imu_pitch_sign = (raw_span > 0.0f) ? 1 : -1;
    for (index = 0U;
         index < LINKAGE_KINEMATICS_CALIBRATION_POINT_COUNT;
         index++) {
        control->mapping_pitch_deg[index] *=
            (float) control->imu_pitch_sign;
    }
    control->return_pitch_deg *= (float) control->imu_pitch_sign;
    control->negative_pitch_deg = control->mapping_pitch_deg[0];
    control->center_pitch_deg = control->mapping_pitch_deg[
        LINKAGE_KINEMATICS_LEVEL_POINT_INDEX];
    control->positive_pitch_deg = control->mapping_pitch_deg[
        LINKAGE_KINEMATICS_CALIBRATION_POINT_COUNT - 1U];

    if ((LinkageKinematics_GetCalibrationPoint(
             0U, &first_model_pitch_deg, &servo_angle_deg) == 0U) ||
        (LinkageKinematics_GetCalibrationPoint(
             LINKAGE_KINEMATICS_CALIBRATION_POINT_COUNT - 1U,
             &last_model_pitch_deg,
             &servo_angle_deg) == 0U)) {
        PipeAttitudeControl_Fail(control,
                                 now_ms,
                                 PIPE_CALIBRATION_EVENT_FAILED_RANGE);
        return;
    }

    normalized_span = control->positive_pitch_deg -
                      control->negative_pitch_deg;
    control->model_scale = normalized_span /
        (last_model_pitch_deg - first_model_pitch_deg);
    for (index = 0U;
         index < LINKAGE_KINEMATICS_CALIBRATION_POINT_COUNT;
         index++) {
        if ((index > 0U) &&
            ((control->mapping_pitch_deg[index] -
              control->mapping_pitch_deg[index - 1U]) <
             PIPE_ATTITUDE_MAPPING_MIN_STEP_DEG)) {
            PipeAttitudeControl_Fail(
                control, now_ms, PIPE_CALIBRATION_EVENT_FAILED_RANGE);
            return;
        }
    }
    /* 九点曲线直接参与控制，O仅表示122°水平采样点相对初始零位的偏差。 */
    control->model_offset_deg = control->center_pitch_deg;
    return_error = PipeAttitudeControl_Abs(control->return_pitch_deg -
                                           control->center_pitch_deg);

    if ((control->negative_pitch_deg >= control->center_pitch_deg) ||
        (control->center_pitch_deg >= control->positive_pitch_deg) ||
        (control->model_scale < PIPE_ATTITUDE_MODEL_SCALE_MIN) ||
        (control->model_scale > PIPE_ATTITUDE_MODEL_SCALE_MAX) ||
        (PipeAttitudeControl_Abs(control->model_offset_deg) >
         PIPE_ATTITUDE_MODEL_OFFSET_MAX_DEG) ||
        (return_error > PIPE_ATTITUDE_MAX_RETURN_ERROR_DEG)) {
        PipeAttitudeControl_Fail(control,
                                 now_ms,
                                 PIPE_CALIBRATION_EVENT_FAILED_RANGE);
        return;
    }

    control->calibration_valid = 1U;
    PipeAttitudeControl_SetState(control,
                                 PIPE_CALIBRATION_SUCCESS,
                                 now_ms,
                                 PIPE_CALIBRATION_EVENT_SUCCESS);
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
        maximum_delta = PIPE_ATTITUDE_CALIBRATION_SERVO_SLEW_DEG_S * dt_s;
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

static uint8_t PipeAttitudeControl_ServoTargetReached(
    const PipeAttitudeControl *control)
{
    return (uint8_t) (PipeAttitudeControl_Abs(
        control->servo_override_deg - control->servo_target_deg) <=
        PIPE_ATTITUDE_SERVO_TARGET_EPSILON_DEG);
}

static uint8_t PipeAttitudeControl_SettleElapsed(
    PipeAttitudeControl *control,
    uint32_t now_ms,
    uint32_t settle_ms)
{
    if (PipeAttitudeControl_ServoTargetReached(control) == 0U) {
        control->settle_ready = 0U;
        return 0U;
    }
    if (control->settle_ready == 0U) {
        control->settle_ready = 1U;
        control->settle_started_ms = now_ms;
        return 0U;
    }
    return (uint8_t) ((uint32_t) (now_ms - control->settle_started_ms) >=
                      settle_ms);
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
    output->calibration_point_index = control->calibration_point_index;
    output->calibration_sample_count = control->sample_count;
    output->servo_override_deg = control->servo_override_deg;
    output->servo_target_deg = control->servo_target_deg;
    output->calibration_target_pitch_deg =
        control->calibration_target_pitch_deg;
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
    float neutral_servo_deg;

    if (control == 0) {
        return;
    }

    memset(control, 0, sizeof(*control));
    neutral_servo_deg = LinkageKinematics_GetNominalServoCenterDeg();
    control->calibration_state = PIPE_CALIBRATION_IDLE;
    control->calibration_target_pitch_deg =
        PIPE_ATTITUDE_CALIBRATION_CENTER_DEG;
    control->servo_target_deg = neutral_servo_deg;
    control->servo_override_deg = neutral_servo_deg;
}

uint8_t PipeAttitudeControl_StartCalibration(PipeAttitudeControl *control,
                                             uint32_t now_ms,
                                             float current_servo_angle_deg)
{
    if ((control == 0) ||
        (PipeAttitudeControl_IsActiveState(control->calibration_state) != 0U)) {
        return 0U;
    }

    PipeAttitudeControl_Init(control);
    control->servo_override_deg = PipeAttitudeControl_Limit(
        current_servo_angle_deg,
        PIPE_ATTITUDE_SERVO_SAFE_MIN_DEG,
        PIPE_ATTITUDE_SERVO_SAFE_MAX_DEG);
    control->last_task_ms = now_ms;
    control->task_time_ready = 1U;
    if (PipeAttitudeControl_SetPointState(
            control,
            PIPE_CALIBRATION_SETTLE_LEVEL,
            PIPE_ATTITUDE_CALIBRATION_CENTER_DEG,
            now_ms,
            PIPE_CALIBRATION_EVENT_READY) == 0U) {
        PipeAttitudeControl_Init(control);
        return 0U;
    }
    return 1U;
}

void PipeAttitudeControl_AbortCalibration(PipeAttitudeControl *control,
                                          uint32_t now_ms)
{
    if ((control == 0) ||
        (PipeAttitudeControl_IsActiveState(control->calibration_state) == 0U) ||
        (control->calibration_state == PIPE_CALIBRATION_RETURN_ABORTED)) {
        return;
    }

    control->calibration_valid = 0U;
    if (PipeAttitudeControl_SetServoTarget(
            control, PIPE_ATTITUDE_CALIBRATION_CENTER_DEG) == 0U) {
        control->servo_target_deg =
            LinkageKinematics_GetNominalServoCenterDeg();
    }
    PipeAttitudeControl_SetState(control,
                                 PIPE_CALIBRATION_RETURN_ABORTED,
                                 now_ms,
                                 PIPE_CALIBRATION_EVENT_ABORTED);
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
        control->raw_accel_pitch_deg =
            PipeAttitudeControl_AccelPitchDeg(measurement);
        if (control->calibration_state == PIPE_CALIBRATION_SAMPLE_LEVEL) {
            PipeAttitudeControl_CollectLevelSample(control,
                                                   now_ms,
                                                   measurement);
        } else {
            PipeAttitudeControl_UpdateFusion(control,
                                             measurement,
                                             chassis_accel_cm_s2);
            if (PipeAttitudeControl_IsPointSampleState(
                    control->calibration_state) != 0U) {
                PipeAttitudeControl_CollectPointSample(control, now_ms);
            }
        }
    }

    if (PipeAttitudeControl_IsActiveState(control->calibration_state) != 0U) {
        PipeAttitudeControl_UpdateServoOverride(control, now_ms);
    } else {
        control->last_task_ms = now_ms;
        control->task_time_ready = 1U;
    }

    state_elapsed_ms = (uint32_t) (now_ms - control->state_started_ms);
    switch (control->calibration_state) {
        case PIPE_CALIBRATION_SETTLE_LEVEL:
            if (PipeAttitudeControl_SettleElapsed(
                    control, now_ms, PIPE_ATTITUDE_LEVEL_SETTLE_MS) != 0U) {
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

        case PIPE_CALIBRATION_SETTLE_MAPPING_POINT:
            if (PipeAttitudeControl_SettleElapsed(
                    control, now_ms, PIPE_ATTITUDE_POINT_SETTLE_MS) != 0U) {
                PipeAttitudeControl_SetState(
                    control,
                    PIPE_CALIBRATION_SAMPLE_MAPPING_POINT,
                    now_ms,
                    PIPE_CALIBRATION_EVENT_NONE);
            }
            break;

        case PIPE_CALIBRATION_SETTLE_RETURN:
            if (PipeAttitudeControl_SettleElapsed(
                    control, now_ms, PIPE_ATTITUDE_POINT_SETTLE_MS) != 0U) {
                PipeAttitudeControl_SetState(
                    control,
                    PIPE_CALIBRATION_SAMPLE_RETURN,
                    now_ms,
                    PIPE_CALIBRATION_EVENT_NONE);
            }
            break;

        case PIPE_CALIBRATION_SAMPLE_MAPPING_POINT:
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

        case PIPE_CALIBRATION_RETURN_ABORTED:
            if (PipeAttitudeControl_ServoTargetReached(control) != 0U) {
                control->calibration_state = PIPE_CALIBRATION_IDLE;
                control->state_started_ms = now_ms;
            }
            break;

        case PIPE_CALIBRATION_RETURN_FAILED:
            if (PipeAttitudeControl_ServoTargetReached(control) != 0U) {
                control->calibration_state = PIPE_CALIBRATION_FAILED;
                control->state_started_ms = now_ms;
            }
            break;

        default:
            break;
    }

    PipeAttitudeControl_FillOutput(control, now_ms, output);
}
