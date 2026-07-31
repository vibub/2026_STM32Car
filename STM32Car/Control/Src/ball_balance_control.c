#include "ball_balance_control.h"

#include "linkage_kinematics.h"

#include <math.h>
#include <string.h>

#define BALL_BALANCE_MIN_ANGLE_DEG               0.0f  /* 重新装配后的最小安全舵机内部角。 */
#define BALL_BALANCE_MAX_ANGLE_DEG             180.0f  /* 重新装配后的最大安全舵机内部角。 */
#define BALL_BALANCE_MIN_PIPE_PITCH_DEG         -6.0f  /* 钢球外环允许的最大下降目标角，仍受九点实测范围裁剪。 */
#define BALL_BALANCE_MAX_PIPE_PITCH_DEG          3.0f  /* 钢球外环允许的最大抬升目标角。 */
#define BALL_BALANCE_PIPE_SAFETY_LIMIT_DEG       7.0f  /* 实测俯仰超过该角度时暂停姿态闭环。 */
#define BALL_BALANCE_KP_MAX_DEG_PER_MM              0.200f /* Debug 可调 Kp 安全上限。 */
#define BALL_BALANCE_KI_MAX_DEG_PER_MM_S            0.100f /* Debug 可调 Ki 安全上限。 */
#define BALL_BALANCE_KD_MAX_DEG_PER_MM_S            0.200f /* Debug 可调 Kd 安全上限。 */
#define BALL_BALANCE_POSITION_P_LIMIT_DEG          3.00f /* PD 调试阶段允许使用完整约 3 度倾角把端部钢球拉回。 */
#define BALL_BALANCE_POSITION_D_LIMIT_DEG          2.00f /* 限制视觉速度尖峰造成的瞬时杆子抖动。 */
#define BALL_BALANCE_POSITION_DEADBAND_MM          5.0f  /* PD 阶段在零点附近允许小范围静止。 */
#define BALL_BALANCE_STOPPED_SPEED_MM_S            3.0f  /* 只有球真正接近静止才进入零点保持。 */
#define BALL_BALANCE_BALL_MOVING_SPEED_MM_S         8.0f
#define BALL_BALANCE_BALL_STOPPED_SPEED_MM_S        5.0f
#define BALL_BALANCE_HOLD_ENTER_MM                  10.0f
#define BALL_BALANCE_HOLD_EXIT_MM                   15.0f
#define BALL_BALANCE_PASSIVE_CAPTURE_MM             18.0f
#define BALL_BALANCE_STATIONARY_SAMPLE_COUNT         2U
#define BALL_BALANCE_PIPE_SETTLE_ERROR_DEG            0.45f
#define BALL_BALANCE_PIPE_SETTLE_RATE_DPS             1.00f
#define BALL_BALANCE_PIPE_SETTLE_MS                  120U
#define BALL_BALANCE_ACCEL_STAGE_MS                  600U
#define BALL_BALANCE_ZERO_HOLD_FREEZE_MS             500U
#define BALL_BALANCE_TURNAROUND_HOLD_MS              500U
#define BALL_BALANCE_BRAKE_MIN_MS                     80U
#define BALL_BALANCE_BRAKE_MAX_MS                    220U
#define BALL_BALANCE_BRAKE_RELEASE_SPEED_MM_S          4.0f
#define BALL_BALANCE_ACCEL_PITCH_MIN_DEG              0.75f
#define BALL_BALANCE_ACCEL_POSITION_GAIN_DEG_PER_MM   0.012f
#define BALL_BALANCE_KICK_ATTEMPT_STEP_DEG              0.20f
#define BALL_BALANCE_KICK_MAX_PITCH_DEG                 3.00f
#define BALL_BALANCE_AWAY_MIN_PITCH_DEG                0.45f
#define BALL_BALANCE_AWAY_POSITION_GAIN_DEG_PER_MM     0.014f
#define BALL_BALANCE_AWAY_SPEED_GAIN_DEG_PER_MM_S      0.003f
#define BALL_BALANCE_AWAY_MAX_PITCH_DEG                2.00f
#define BALL_BALANCE_TURNAROUND_LEVEL_SPEED_MM_S       12.0f
#define BALL_BALANCE_RETURN_BRAKE_MAX_PITCH_DEG          1.10f
#define BALL_BALANCE_RETURN_BRAKE_FULL_SPEED_MM_S      150.0f
#define BALL_BALANCE_BRAKE_ACCEL_PER_DEG_MM_S2         70.0f
#define BALL_BALANCE_BRAKE_VISION_GUARD_S               0.55f
#define BALL_BALANCE_BRAKE_POSITION_MARGIN_MM           10.0f
#define BALL_BALANCE_INTEGRAL_OUTPUT_LIMIT_DEG     0.80f /* 积分最大只补偿 0.8 度。 */
#define BALL_BALANCE_STICTION_INTEGRAL_BOOST       3.0f  /* 钢球静止时加快积分以突破静摩擦。 */
#define BALL_BALANCE_MOVING_INTEGRAL_LEAK_TAU_S    0.80f /* 钢球运动后泄放积分以降低过冲。 */
#define BALL_BALANCE_GAIN_EPSILON                  0.000001f
#define BALL_BALANCE_PIPE_KP                       0.35f
#define BALL_BALANCE_PIPE_KD_S                     0.080f
#define BALL_BALANCE_MIN_QUALITY                  50U    /* 接受 K230 测量所需的最低质量。 */
#define BALL_BALANCE_DATA_TIMEOUT_MS            350U    /* 兼容 K230 约 5 FPS 及偶发一帧抖动，避免误判为新球。 */
#define BALL_BALANCE_ATTITUDE_TIMEOUT_MS        100U    /* IMU 姿态数据超时时间。 */
#define BALL_BALANCE_ATTITUDE_STABLE_MS         150U    /* 启用前只确认短时间稳定，避免点击后反应迟钝。 */
#define BALL_BALANCE_ENABLE_PITCH_LIMIT_DEG       0.60f
#define BALL_BALANCE_ENABLE_RATE_LIMIT_DPS        1.00f
#define BALL_BALANCE_TARGET_LIMIT_MM            100.0f  /* 可设置钢球目标位置的绝对上限。 */
#define BALL_BALANCE_POSITION_LIMIT_MM          130.0f  /* 合法钢球测量位置的绝对上限。 */
#define BALL_BALANCE_PIPE_PROFILE_MAX_RATE_DPS    12.0f
#define BALL_BALANCE_PIPE_PROFILE_ACCEL_DPS2      60.0f
#define BALL_BALANCE_PIPE_BRAKE_MAX_RATE_DPS      20.0f
#define BALL_BALANCE_PIPE_BRAKE_ACCEL_DPS2       120.0f
#define BALL_BALANCE_SERVO_SLEW_DEG_S            180.0f
#define BALL_BALANCE_DT_MIN_S                     0.010f /* 视觉测量 dt 的最小值。 */
#define BALL_BALANCE_DT_MAX_S                     0.100f /* 视觉测量和任务 dt 的最大值。 */
#define BALL_BALANCE_MEASUREMENT_DT_MAX_S         0.250f /* 兼容 K230 约 5~6 FPS 的真实帧间隔。 */
#define BALL_BALANCE_POSITION_FILTER_TAU_S        0.080f /* 适度平滑位置检测抖动。 */
#define BALL_BALANCE_VELOCITY_FILTER_TAU_S        0.120f /* 平滑速度同时保留低帧率下的制动响应。 */
#define BALL_BALANCE_VISION_LOOKAHEAD_S            0.25f /* 补偿 K230 推理和串口位置数据的固有延迟。 */
#define BALL_BALANCE_PREDICTION_MAX_S              0.45f /* 限制丢帧时的位置外推时间。 */
#define BALL_BALANCE_ACCEL_PER_PIPE_DEG_MM_S2     100.0f /* 实机方向：正俯仰使 K230 位置向正向加速。 */
#define BALL_BALANCE_MODEL_SCALE_MIN               0.35f  /* 柔性水管可接受模型校正比例的最小值。 */
#define BALL_BALANCE_MODEL_SCALE_MAX               1.25f  /* 可接受模型校正比例的最大值。 */
#define BALL_BALANCE_MODEL_OFFSET_MAX_DEG           0.75f  /* 可接受模型校正零偏的最大绝对值。 */
#define BALL_BALANCE_CACHE_EPSILON_DEG              0.0001f /* 复用运动学缓存允许的输入误差。 */
#define BALL_BALANCE_MAPPING_MIN_STEP_DEG             0.05f /* 九点实测映射允许的最小单调角度差。 */

static uint8_t BallBalanceControl_AttitudeIsFresh(
    const BallBalanceControl *control,
    uint32_t now_ms);

static float BallBalanceControl_Abs(float value)
{
    return (value < 0.0f) ? -value : value;
}

static float BallBalanceControl_Limit(float value,
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

static float BallBalanceControl_LimitActualPitch(
    const BallBalanceControl *control,
    float value)
{
    if ((control == 0) || (control->mapping_valid == 0U)) {
        return BallBalanceControl_Limit(
            value,
            BALL_BALANCE_MIN_PIPE_PITCH_DEG,
            BALL_BALANCE_MAX_PIPE_PITCH_DEG);
    }
    return BallBalanceControl_Limit(value,
                                    control->minimum_pipe_pitch_deg,
                                    control->maximum_pipe_pitch_deg);
}

static float BallBalanceControl_Approach(float value,
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

static void BallBalanceControl_UpdatePipeProfile(
    BallBalanceControl *control,
    float task_dt_s)
{
    float distance_deg;
    float target_rate_dps;
    float maximum_rate_change_dps;
    float profile_accel_dps2;
    float profile_max_rate_dps;
    float next_pitch_deg;

    if (task_dt_s <= 0.0f) {
        return;
    }
    distance_deg = control->desired_pipe_pitch_deg -
                   control->command_pipe_pitch_deg;
    if ((control->trajectory_phase == BALL_BALANCE_PHASE_BRAKE) ||
        (control->trajectory_phase == BALL_BALANCE_PHASE_RETURN_LEVEL)) {
        profile_accel_dps2 = BALL_BALANCE_PIPE_BRAKE_ACCEL_DPS2;
        profile_max_rate_dps = BALL_BALANCE_PIPE_BRAKE_MAX_RATE_DPS;
    } else {
        profile_accel_dps2 = BALL_BALANCE_PIPE_PROFILE_ACCEL_DPS2;
        profile_max_rate_dps = BALL_BALANCE_PIPE_PROFILE_MAX_RATE_DPS;
    }
    if ((BallBalanceControl_Abs(distance_deg) <= 0.002f) &&
        (BallBalanceControl_Abs(control->pipe_command_rate_dps) <= 0.02f)) {
        control->command_pipe_pitch_deg = control->desired_pipe_pitch_deg;
        control->pipe_command_rate_dps = 0.0f;
        return;
    }

    target_rate_dps = sqrtf(
        2.0f * profile_accel_dps2 *
        BallBalanceControl_Abs(distance_deg));
    target_rate_dps = BallBalanceControl_Limit(
        target_rate_dps,
        0.0f,
        profile_max_rate_dps);
    if (distance_deg < 0.0f) {
        target_rate_dps = -target_rate_dps;
    }
    maximum_rate_change_dps =
        profile_accel_dps2 * task_dt_s;
    control->pipe_command_rate_dps = BallBalanceControl_Approach(
        control->pipe_command_rate_dps,
        target_rate_dps,
        maximum_rate_change_dps);
    next_pitch_deg = control->command_pipe_pitch_deg +
                     control->pipe_command_rate_dps * task_dt_s;
    if (((distance_deg >= 0.0f) &&
         (next_pitch_deg >= control->desired_pipe_pitch_deg)) ||
        ((distance_deg < 0.0f) &&
         (next_pitch_deg <= control->desired_pipe_pitch_deg))) {
        next_pitch_deg = control->desired_pipe_pitch_deg;
        control->pipe_command_rate_dps = 0.0f;
    }
    control->command_pipe_pitch_deg = BallBalanceControl_LimitActualPitch(
        control, next_pitch_deg);
}

static void BallBalanceControl_ClearVisionState(BallBalanceControl *control,
                                                uint8_t clear_sequence)
{
    control->tracking_valid = 0U;
    control->timed_out = 0U;
    control->filter_ready = 0U;
    control->integral_active = 0U;
    control->braking_active = 0U;
    control->brake_direction = 0;
    control->outward_recovery_active = 0U;
    control->turnaround_hold_active = 0U;
    control->zero_hold_freeze_active = 0U;
    control->pipe_settled = 0U;
    control->pipe_settle_timer_ready = 0U;
    control->stationary_sample_count = 0U;
    control->kick_attempt_count = 0U;
    control->trajectory_phase = BALL_BALANCE_PHASE_WAIT_PIPE;
    if (clear_sequence != 0U) {
        control->sequence_ready = 0U;
        control->last_sequence = 0U;
    }
    control->last_valid_ms = 0U;
    control->pipe_settle_since_ms = 0U;
    control->phase_started_ms = 0U;
    control->kick_stage_started_ms = 0U;
    control->last_measurement_dt_s = 0.0f;
    control->filtered_position_mm = 0.0f;
    control->predicted_position_mm = 0.0f;
    control->predicted_accel_mm_s2 = 0.0f;
    control->velocity_mm_s = 0.0f;
    control->integral_mm_s = 0.0f;
    control->error_mm = 0.0f;
    control->p_term_deg = 0.0f;
    control->i_term_deg = 0.0f;
    control->d_term_deg = 0.0f;
    control->brake_term_deg = 0.0f;
    control->phase_pitch_deg = 0.0f;
    control->stop_distance_mm = 0.0f;
    control->desired_pipe_pitch_deg = 0.0f;
    control->pipe_command_rate_dps = 0.0f;
}

static void BallBalanceControl_InvalidateMapping(BallBalanceControl *control)
{
    control->mapping_valid = 0U;
    control->mapping_fault = 1U;
    control->enabled_requested = 0U;
    control->enabled = 0U;
    control->attitude_stable_ready = 0U;
    control->feedforward_cache_valid = 0U;
    control->desired_cache_valid = 0U;
    BallBalanceControl_ClearVisionState(control, 1U);
}

static uint8_t BallBalanceControl_MapActualPitch(
    const BallBalanceControl *control,
    float actual_pitch_deg,
    float *model_pitch_deg,
    float *servo_angle_deg)
{
    float lower_model_pitch_deg;
    float upper_model_pitch_deg;
    float lower_servo_angle_deg;
    float upper_servo_angle_deg;
    float lower_actual_pitch_deg;
    float upper_actual_pitch_deg;
    float ratio;
    uint8_t index;

    if ((model_pitch_deg == 0) || (servo_angle_deg == 0) ||
        (control->mapping_valid == 0U) ||
        (control->model_scale < BALL_BALANCE_MODEL_SCALE_MIN) ||
        (control->model_scale > BALL_BALANCE_MODEL_SCALE_MAX)) {
        return 0U;
    }

    actual_pitch_deg = BallBalanceControl_LimitActualPitch(
        control, actual_pitch_deg);
    if (actual_pitch_deg <= control->mapping_pitch_deg[0]) {
        return LinkageKinematics_GetCalibrationPoint(
            0U, model_pitch_deg, servo_angle_deg);
    }

    for (index = 1U;
         index < LINKAGE_KINEMATICS_CALIBRATION_POINT_COUNT;
         index++) {
        upper_actual_pitch_deg = control->mapping_pitch_deg[index];
        if (actual_pitch_deg <= upper_actual_pitch_deg) {
            lower_actual_pitch_deg = control->mapping_pitch_deg[index - 1U];
            if ((LinkageKinematics_GetCalibrationPoint(
                     (uint8_t) (index - 1U),
                     &lower_model_pitch_deg,
                     &lower_servo_angle_deg) == 0U) ||
                (LinkageKinematics_GetCalibrationPoint(
                     index,
                     &upper_model_pitch_deg,
                     &upper_servo_angle_deg) == 0U) ||
                (upper_actual_pitch_deg <= lower_actual_pitch_deg)) {
                return 0U;
            }
            ratio = (actual_pitch_deg - lower_actual_pitch_deg) /
                    (upper_actual_pitch_deg - lower_actual_pitch_deg);
            *model_pitch_deg = lower_model_pitch_deg + ratio *
                (upper_model_pitch_deg - lower_model_pitch_deg);
            *servo_angle_deg = lower_servo_angle_deg + ratio *
                (upper_servo_angle_deg - lower_servo_angle_deg);
            return 1U;
        }
    }

    return LinkageKinematics_GetCalibrationPoint(
        LINKAGE_KINEMATICS_CALIBRATION_POINT_COUNT - 1U,
        model_pitch_deg,
        servo_angle_deg);
}

static uint8_t BallBalanceControl_MapFeedforward(
    BallBalanceControl *control,
    float actual_pitch_deg,
    float *servo_angle_deg)
{
    if (BallBalanceControl_MapActualPitch(
            control,
            actual_pitch_deg,
            &control->feedforward_model_pitch_deg,
            servo_angle_deg) == 0U) {
        return 0U;
    }
    control->feedforward_cache_valid = 1U;
    control->servo_feedforward_deg = *servo_angle_deg;
    return 1U;
}

static uint8_t BallBalanceControl_MapDesired(
    BallBalanceControl *control,
    float actual_pitch_deg,
    float *servo_angle_deg)
{
    if (BallBalanceControl_MapActualPitch(
            control,
            actual_pitch_deg,
            &control->desired_model_pitch_deg,
            servo_angle_deg) == 0U) {
        return 0U;
    }
    control->desired_cache_valid = 1U;
    control->desired_angle_deg = *servo_angle_deg;
    return 1U;
}

static void BallBalanceControl_UpdatePrediction(BallBalanceControl *control,
                                                uint32_t now_ms)
{
    float effective_pipe_pitch_deg;
    float sample_age_s;
    float prediction_s;

    if ((control->tracking_valid == 0U) ||
        (control->filter_ready == 0U)) {
        control->predicted_position_mm = control->filtered_position_mm;
        control->predicted_accel_mm_s2 = 0.0f;
        return;
    }

    sample_age_s = (float) ((uint32_t) (now_ms - control->last_valid_ms)) *
                   0.001f;
    prediction_s = BallBalanceControl_Limit(
        sample_age_s + BALL_BALANCE_VISION_LOOKAHEAD_S,
        0.0f,
        BALL_BALANCE_PREDICTION_MAX_S);
    effective_pipe_pitch_deg = control->measured_pipe_pitch_deg;
    if ((control->attitude_ready != 0U) &&
        ((uint32_t) (now_ms - control->last_attitude_ms) <=
         BALL_BALANCE_ATTITUDE_TIMEOUT_MS)) {
        effective_pipe_pitch_deg +=
            0.5f * control->measured_pipe_rate_dps * prediction_s;
        effective_pipe_pitch_deg = BallBalanceControl_LimitActualPitch(
            control, effective_pipe_pitch_deg);
        control->predicted_accel_mm_s2 =
            BALL_BALANCE_ACCEL_PER_PIPE_DEG_MM_S2 *
            effective_pipe_pitch_deg;
    } else {
        control->predicted_accel_mm_s2 = 0.0f;
    }
    control->predicted_position_mm = BallBalanceControl_Limit(
        control->filtered_position_mm +
        control->velocity_mm_s * prediction_s +
        0.5f * control->predicted_accel_mm_s2 *
        prediction_s * prediction_s,
        -BALL_BALANCE_POSITION_LIMIT_MM,
        BALL_BALANCE_POSITION_LIMIT_MM);
}

static void BallBalanceControl_EnterPhase(BallBalanceControl *control,
                                          BallBalancePhase phase,
                                          uint32_t now_ms,
                                          float pitch_deg)
{
    pitch_deg = BallBalanceControl_LimitActualPitch(control, pitch_deg);
    if (BallBalanceControl_Abs(control->desired_pipe_pitch_deg - pitch_deg) >
        BALL_BALANCE_CACHE_EPSILON_DEG) {
        control->pipe_settled = 0U;
        control->pipe_settle_timer_ready = 0U;
    }
    control->trajectory_phase = (uint8_t) phase;
    control->outward_recovery_active = 0U;
    control->turnaround_hold_active = 0U;
    control->zero_hold_freeze_active = 0U;
    control->phase_started_ms = now_ms;
    if (phase == BALL_BALANCE_PHASE_ACCELERATE) {
        control->kick_stage_started_ms = now_ms;
    }
    control->phase_pitch_deg = pitch_deg;
    control->desired_pipe_pitch_deg = pitch_deg;
    control->stationary_sample_count = 0U;
    control->braking_active =
        (uint8_t) (phase == BALL_BALANCE_PHASE_BRAKE);
    control->brake_direction =
        (phase == BALL_BALANCE_PHASE_BRAKE) ?
        ((pitch_deg >= 0.0f) ? 1 : -1) : 0;
    control->brake_term_deg =
        (phase == BALL_BALANCE_PHASE_BRAKE) ? pitch_deg : 0.0f;
}

static void BallBalanceControl_EnterZeroHoldFreeze(
    BallBalanceControl *control,
    uint32_t now_ms)
{
    BallBalanceControl_EnterPhase(
        control,
        BALL_BALANCE_PHASE_HOLD,
        now_ms,
        control->command_pipe_pitch_deg);
    control->zero_hold_freeze_active = 1U;
    control->pipe_command_rate_dps = 0.0f;
}

static float BallBalanceControl_SelectAccelerationPitch(
    float error_mm,
    uint8_t kick_attempt_count)
{
    float magnitude = BallBalanceControl_Abs(error_mm);
    float pitch_deg = BALL_BALANCE_ACCEL_PITCH_MIN_DEG +
        BALL_BALANCE_ACCEL_POSITION_GAIN_DEG_PER_MM * magnitude;

    pitch_deg += BALL_BALANCE_KICK_ATTEMPT_STEP_DEG *
                 (float) kick_attempt_count;
    pitch_deg = BallBalanceControl_Limit(
        pitch_deg, 0.0f, BALL_BALANCE_KICK_MAX_PITCH_DEG);
    return (error_mm >= 0.0f) ? -pitch_deg : pitch_deg;
}

static void BallBalanceControl_EnterStationaryAcceleration(
    BallBalanceControl *control,
    uint32_t now_ms)
{
    BallBalanceControl_EnterPhase(
        control,
        BALL_BALANCE_PHASE_ACCELERATE,
        now_ms,
        BallBalanceControl_SelectAccelerationPitch(
            control->error_mm, control->kick_attempt_count));
    /* The transition into this helper is already based on two consecutive
     * stopped K230 frames.  Preserve that evidence for staged breakaway. */
    control->stationary_sample_count =
        BALL_BALANCE_STATIONARY_SAMPLE_COUNT;
}

static float BallBalanceControl_SelectAwayRecoveryPitch(
    float error_mm,
    float velocity_mm_s)
{
    float pitch_deg = BALL_BALANCE_AWAY_MIN_PITCH_DEG +
        BALL_BALANCE_AWAY_POSITION_GAIN_DEG_PER_MM *
        BallBalanceControl_Abs(error_mm) +
        BALL_BALANCE_AWAY_SPEED_GAIN_DEG_PER_MM_S *
        BallBalanceControl_Abs(velocity_mm_s);

    pitch_deg = BallBalanceControl_Limit(
        pitch_deg,
        BALL_BALANCE_AWAY_MIN_PITCH_DEG,
        BALL_BALANCE_AWAY_MAX_PITCH_DEG);
    return (error_mm >= 0.0f) ? -pitch_deg : pitch_deg;
}

static void BallBalanceControl_EnterTurnaroundHold(
    BallBalanceControl *control,
    uint32_t now_ms)
{
    /* Freeze the pipe exactly where it is at the side turning point.  This
     * prevents either an extra lift or a return-level movement while ball
     * speed is close to zero. */
    BallBalanceControl_EnterPhase(
        control,
        BALL_BALANCE_PHASE_ACCELERATE,
        now_ms,
        control->command_pipe_pitch_deg);
    control->turnaround_hold_active = 1U;
    control->pipe_command_rate_dps = 0.0f;
}

static void BallBalanceControl_EnterOutwardRecovery(
    BallBalanceControl *control,
    uint32_t now_ms)
{
    if (BallBalanceControl_Abs(control->velocity_mm_s) <=
        BALL_BALANCE_TURNAROUND_LEVEL_SPEED_MM_S) {
        BallBalanceControl_EnterTurnaroundHold(control, now_ms);
        return;
    }
    BallBalanceControl_EnterPhase(
        control,
        BALL_BALANCE_PHASE_ACCELERATE,
        now_ms,
        BallBalanceControl_SelectAwayRecoveryPitch(
            control->error_mm, control->velocity_mm_s));
    control->outward_recovery_active = 1U;
}

static float BallBalanceControl_SelectReturnBrakePitch(float velocity_mm_s)
{
    float velocity_abs = BallBalanceControl_Abs(velocity_mm_s);
    float speed_scale = BallBalanceControl_Limit(
        (velocity_abs - BALL_BALANCE_BRAKE_RELEASE_SPEED_MM_S) /
        (BALL_BALANCE_RETURN_BRAKE_FULL_SPEED_MM_S -
         BALL_BALANCE_BRAKE_RELEASE_SPEED_MM_S),
        0.0f,
        1.0f);
    float pitch_deg = BALL_BALANCE_RETURN_BRAKE_MAX_PITCH_DEG * speed_scale;

    /* Returning-to-zero braking is deliberately position independent.  It
     * fades to level with speed and only reaches full strength when fast. */
    return (velocity_mm_s >= 0.0f) ? -pitch_deg : pitch_deg;
}

static uint8_t BallBalanceControl_BrakeIsDue(
    BallBalanceControl *control,
    float error_abs,
    float velocity_mm_s,
    float *brake_pitch_deg)
{
    float velocity_abs;
    float braking_accel_mm_s2;
    float remaining_distance_mm;

    velocity_abs = BallBalanceControl_Abs(velocity_mm_s);
    if (velocity_abs < BALL_BALANCE_BALL_MOVING_SPEED_MM_S) {
        control->stop_distance_mm = 0.0f;
        return 0U;
    }

    *brake_pitch_deg = BallBalanceControl_SelectReturnBrakePitch(
        velocity_mm_s);
    braking_accel_mm_s2 = BALL_BALANCE_BRAKE_ACCEL_PER_DEG_MM_S2 *
                          BallBalanceControl_Abs(*brake_pitch_deg);
    control->stop_distance_mm =
        velocity_abs * velocity_abs / (2.0f * braking_accel_mm_s2) +
        velocity_abs * BALL_BALANCE_BRAKE_VISION_GUARD_S;
    remaining_distance_mm = error_abs -
                            BALL_BALANCE_BRAKE_POSITION_MARGIN_MM;
    if (remaining_distance_mm < 0.0f) {
        remaining_distance_mm = 0.0f;
    }
    return (uint8_t) (control->stop_distance_mm >= remaining_distance_mm);
}

static void BallBalanceControl_UpdatePipeSettled(BallBalanceControl *control,
                                                 uint32_t now_ms)
{
    uint8_t stable;

    stable = (uint8_t) (
        (BallBalanceControl_AttitudeIsFresh(control, now_ms) != 0U) &&
        (BallBalanceControl_Abs(control->command_pipe_pitch_deg -
                                control->measured_pipe_pitch_deg) <=
         BALL_BALANCE_PIPE_SETTLE_ERROR_DEG) &&
        (BallBalanceControl_Abs(control->measured_pipe_rate_dps) <=
         BALL_BALANCE_PIPE_SETTLE_RATE_DPS));
    if (stable == 0U) {
        control->pipe_settled = 0U;
        control->pipe_settle_timer_ready = 0U;
        return;
    }
    if (control->pipe_settle_timer_ready == 0U) {
        control->pipe_settle_timer_ready = 1U;
        control->pipe_settle_since_ms = now_ms;
        return;
    }
    if ((uint32_t) (now_ms - control->pipe_settle_since_ms) >=
        BALL_BALANCE_PIPE_SETTLE_MS) {
        control->pipe_settled = 1U;
    }
}

static void BallBalanceControl_UpdateTrajectory(
    BallBalanceControl *control,
    uint32_t now_ms,
    uint8_t measurement_accepted)
{
    uint32_t phase_elapsed_ms;
    uint32_t kick_stage_elapsed_ms;
    float error_abs;
    float velocity_abs;
    float brake_pitch_deg;
    float candidate_pitch_deg;
    uint8_t moving_away_from_target;
    uint8_t moving_toward_target;

    control->error_mm = control->filtered_position_mm - control->target_mm;
    error_abs = BallBalanceControl_Abs(control->error_mm);
    velocity_abs = BallBalanceControl_Abs(control->velocity_mm_s);
    moving_toward_target =
        (uint8_t) ((control->error_mm * control->velocity_mm_s) < 0.0f);
    moving_away_from_target =
        (uint8_t) ((control->error_mm * control->velocity_mm_s) > 0.0f);
    phase_elapsed_ms = (uint32_t) (now_ms - control->phase_started_ms);
    kick_stage_elapsed_ms =
        (uint32_t) (now_ms - control->kick_stage_started_ms);

    /* The position PID is intentionally inactive in trajectory mode. */
    control->integral_mm_s = 0.0f;
    control->integral_active = 0U;
    control->p_term_deg = 0.0f;
    control->i_term_deg = 0.0f;
    control->d_term_deg = 0.0f;

    switch ((BallBalancePhase) control->trajectory_phase) {
        case BALL_BALANCE_PHASE_WAIT_PIPE:
            control->desired_pipe_pitch_deg = 0.0f;
            if ((control->pipe_settled == 0U) ||
                (measurement_accepted == 0U)) {
                break;
            }
            /* The first valid frame initializes position only.  Use the next
             * frame to distinguish a freely rolling ball from stiction. */
            if (control->last_measurement_dt_s <= 0.0f) {
                break;
            }
            if ((error_abs <= BALL_BALANCE_HOLD_ENTER_MM) &&
                (velocity_abs <= BALL_BALANCE_BALL_STOPPED_SPEED_MM_S)) {
                BallBalanceControl_EnterZeroHoldFreeze(control, now_ms);
            } else if ((moving_toward_target != 0U) &&
                       (velocity_abs >
                        BALL_BALANCE_BALL_STOPPED_SPEED_MM_S)) {
                if (BallBalanceControl_BrakeIsDue(
                        control,
                        error_abs,
                        control->velocity_mm_s,
                        &brake_pitch_deg) != 0U) {
                    BallBalanceControl_EnterPhase(
                        control,
                        BALL_BALANCE_PHASE_BRAKE,
                        now_ms,
                        brake_pitch_deg);
                } else {
                    BallBalanceControl_EnterPhase(
                        control, BALL_BALANCE_PHASE_COAST, now_ms, 0.0f);
                }
            } else if ((moving_away_from_target != 0U) &&
                       (velocity_abs >
                        BALL_BALANCE_BALL_STOPPED_SPEED_MM_S)) {
                BallBalanceControl_EnterOutwardRecovery(control, now_ms);
            } else {
                control->kick_attempt_count = 0U;
                BallBalanceControl_EnterStationaryAcceleration(
                    control, now_ms);
            }
            break;

        case BALL_BALANCE_PHASE_ACCELERATE:
            if (control->turnaround_hold_active != 0U) {
                control->desired_pipe_pitch_deg = control->phase_pitch_deg;
                control->pipe_command_rate_dps = 0.0f;
                if ((measurement_accepted != 0U) &&
                    (moving_toward_target != 0U) &&
                    (velocity_abs >=
                     BALL_BALANCE_BALL_STOPPED_SPEED_MM_S)) {
                    control->kick_attempt_count = 0U;
                    BallBalanceControl_EnterPhase(
                        control,
                        BALL_BALANCE_PHASE_RETURN_LEVEL,
                        now_ms,
                        0.0f);
                } else if (phase_elapsed_ms >=
                           BALL_BALANCE_TURNAROUND_HOLD_MS) {
                    /* A side-turn freeze is only a short no-motion window.
                     * Resume normal two-frame stop classification after it. */
                    BallBalanceControl_EnterPhase(
                        control,
                        BALL_BALANCE_PHASE_ACCELERATE,
                        now_ms,
                        control->phase_pitch_deg);
                    control->pipe_command_rate_dps = 0.0f;
                }
                break;
            }
            /* At an outward turning point, freeze the pipe before the ball
             * reaches zero speed.  Do not move it or raise a stiction stage
             * until the measured ball direction has turned inward. */
            if ((measurement_accepted != 0U) &&
                (control->outward_recovery_active != 0U) &&
                (velocity_abs <=
                 BALL_BALANCE_TURNAROUND_LEVEL_SPEED_MM_S)) {
                BallBalanceControl_EnterTurnaroundHold(control, now_ms);
                break;
            }
            if (measurement_accepted != 0U) {
                if (velocity_abs <=
                    BALL_BALANCE_BALL_STOPPED_SPEED_MM_S) {
                    if (control->stationary_sample_count < 0xFFU) {
                        control->stationary_sample_count++;
                    }
                } else {
                    control->stationary_sample_count = 0U;
                }
            }
            /* Raise exactly one level only after a fresh vision frame still
             * confirms that the ball is stationary.  Hold each level for a
             * full interval; never catch up several levels at once. */
            if ((measurement_accepted != 0U) &&
                (control->stationary_sample_count >=
                 BALL_BALANCE_STATIONARY_SAMPLE_COUNT) &&
                (control->kick_attempt_count < 0xFFU) &&
                (kick_stage_elapsed_ms >= BALL_BALANCE_ACCEL_STAGE_MS)) {
                control->kick_attempt_count++;
                control->kick_stage_started_ms = now_ms;
                kick_stage_elapsed_ms = 0U;
                candidate_pitch_deg =
                    BallBalanceControl_SelectAccelerationPitch(
                        control->error_mm,
                        control->kick_attempt_count);
                if (BallBalanceControl_Abs(candidate_pitch_deg) >
                    BallBalanceControl_Abs(control->phase_pitch_deg)) {
                    control->phase_pitch_deg = candidate_pitch_deg;
                }
            }
            if ((measurement_accepted != 0U) &&
                (moving_away_from_target != 0U) &&
                (velocity_abs >
                 BALL_BALANCE_TURNAROUND_LEVEL_SPEED_MM_S)) {
                candidate_pitch_deg =
                    BallBalanceControl_SelectAwayRecoveryPitch(
                        control->error_mm,
                        control->velocity_mm_s);
                control->phase_pitch_deg = candidate_pitch_deg;
                control->outward_recovery_active = 1U;
            }
            control->desired_pipe_pitch_deg = control->phase_pitch_deg;
            if ((measurement_accepted != 0U) &&
                (((moving_toward_target != 0U) &&
                  (velocity_abs >= BALL_BALANCE_BALL_MOVING_SPEED_MM_S)) ||
                 (error_abs <= BALL_BALANCE_HOLD_ENTER_MM))) {
                control->kick_attempt_count = 0U;
                if ((moving_toward_target != 0U) &&
                    (BallBalanceControl_BrakeIsDue(
                         control,
                         error_abs,
                         control->velocity_mm_s,
                         &brake_pitch_deg) != 0U)) {
                    BallBalanceControl_EnterPhase(
                        control,
                        BALL_BALANCE_PHASE_BRAKE,
                        now_ms,
                        brake_pitch_deg);
                } else {
                    BallBalanceControl_EnterPhase(
                        control,
                        BALL_BALANCE_PHASE_RETURN_LEVEL,
                        now_ms,
                        0.0f);
                }
            }
            break;

        case BALL_BALANCE_PHASE_RETURN_LEVEL:
            control->desired_pipe_pitch_deg = 0.0f;
            if ((measurement_accepted != 0U) &&
                (moving_away_from_target != 0U) &&
                (velocity_abs >= BALL_BALANCE_BALL_MOVING_SPEED_MM_S)) {
                BallBalanceControl_EnterOutwardRecovery(control, now_ms);
                break;
            }
            if (control->pipe_settled != 0U) {
                BallBalanceControl_EnterPhase(
                    control, BALL_BALANCE_PHASE_COAST, now_ms, 0.0f);
            }
            break;

        case BALL_BALANCE_PHASE_COAST:
            control->desired_pipe_pitch_deg = 0.0f;
            if (measurement_accepted == 0U) {
                break;
            }
            if ((moving_away_from_target != 0U) &&
                (velocity_abs > BALL_BALANCE_BALL_STOPPED_SPEED_MM_S)) {
                BallBalanceControl_EnterOutwardRecovery(control, now_ms);
                break;
            }
            if ((moving_toward_target != 0U) &&
                (BallBalanceControl_BrakeIsDue(
                     control,
                     error_abs,
                     control->velocity_mm_s,
                     &brake_pitch_deg) != 0U)) {
                BallBalanceControl_EnterPhase(
                    control,
                    BALL_BALANCE_PHASE_BRAKE,
                    now_ms,
                    brake_pitch_deg);
            } else if (velocity_abs <=
                       BALL_BALANCE_BALL_STOPPED_SPEED_MM_S) {
                BallBalanceControl_EnterPhase(
                    control, BALL_BALANCE_PHASE_SETTLE, now_ms, 0.0f);
            }
            break;

        case BALL_BALANCE_PHASE_BRAKE:
            if (((measurement_accepted != 0U) &&
                 (phase_elapsed_ms >= BALL_BALANCE_BRAKE_MIN_MS) &&
                 ((velocity_abs <= BALL_BALANCE_BRAKE_RELEASE_SPEED_MM_S) ||
                  ((control->velocity_mm_s *
                    (float) control->brake_direction) > 0.0f))) ||
                (phase_elapsed_ms >= BALL_BALANCE_BRAKE_MAX_MS)) {
                BallBalanceControl_EnterPhase(
                    control, BALL_BALANCE_PHASE_SETTLE, now_ms, 0.0f);
                break;
            }
            if (measurement_accepted != 0U) {
                candidate_pitch_deg = BallBalanceControl_Abs(
                    BallBalanceControl_SelectReturnBrakePitch(
                        control->velocity_mm_s));
                control->phase_pitch_deg =
                    (float) control->brake_direction * candidate_pitch_deg;
                control->brake_term_deg = control->phase_pitch_deg;
            }
            control->desired_pipe_pitch_deg = control->phase_pitch_deg;
            break;

        case BALL_BALANCE_PHASE_SETTLE:
            control->desired_pipe_pitch_deg = 0.0f;
            if ((control->pipe_settled == 0U) ||
                (measurement_accepted == 0U)) {
                break;
            }
            if ((moving_away_from_target != 0U) &&
                (velocity_abs >= BALL_BALANCE_BALL_MOVING_SPEED_MM_S)) {
                BallBalanceControl_EnterOutwardRecovery(control, now_ms);
                break;
            }
            if ((moving_toward_target != 0U) &&
                (velocity_abs >= BALL_BALANCE_BALL_MOVING_SPEED_MM_S)) {
                if (BallBalanceControl_BrakeIsDue(
                        control,
                        error_abs,
                        control->velocity_mm_s,
                        &brake_pitch_deg) != 0U) {
                    BallBalanceControl_EnterPhase(
                        control,
                        BALL_BALANCE_PHASE_BRAKE,
                        now_ms,
                        brake_pitch_deg);
                } else {
                    BallBalanceControl_EnterPhase(
                        control, BALL_BALANCE_PHASE_COAST, now_ms, 0.0f);
                }
                break;
            }
            if (velocity_abs > BALL_BALANCE_BALL_STOPPED_SPEED_MM_S) {
                control->stationary_sample_count = 0U;
                break;
            }
            if (control->stationary_sample_count < 0xFFU) {
                control->stationary_sample_count++;
            }
            if (control->stationary_sample_count >=
                BALL_BALANCE_STATIONARY_SAMPLE_COUNT) {
                if (error_abs <= BALL_BALANCE_HOLD_ENTER_MM) {
                    control->kick_attempt_count = 0U;
                    BallBalanceControl_EnterZeroHoldFreeze(control, now_ms);
                } else {
                    BallBalanceControl_EnterStationaryAcceleration(
                        control, now_ms);
                }
            }
            break;

        case BALL_BALANCE_PHASE_HOLD:
        default:
            if (control->zero_hold_freeze_active != 0U) {
                control->desired_pipe_pitch_deg = control->phase_pitch_deg;
                control->pipe_command_rate_dps = 0.0f;
                if (phase_elapsed_ms >= BALL_BALANCE_ZERO_HOLD_FREEZE_MS) {
                    BallBalanceControl_EnterPhase(
                        control,
                        BALL_BALANCE_PHASE_HOLD,
                        now_ms,
                        0.0f);
                }
                break;
            }
            control->desired_pipe_pitch_deg = 0.0f;
            control->kick_attempt_count = 0U;
            if ((control->pipe_settled == 0U) ||
                (measurement_accepted == 0U)) {
                break;
            }
            if ((moving_away_from_target != 0U) &&
                (velocity_abs >= BALL_BALANCE_BALL_MOVING_SPEED_MM_S)) {
                BallBalanceControl_EnterOutwardRecovery(control, now_ms);
                break;
            }
            if ((moving_toward_target != 0U) &&
                (velocity_abs >= BALL_BALANCE_BALL_MOVING_SPEED_MM_S)) {
                if (BallBalanceControl_BrakeIsDue(
                        control,
                        error_abs,
                        control->velocity_mm_s,
                        &brake_pitch_deg) != 0U) {
                    BallBalanceControl_EnterPhase(
                        control,
                        BALL_BALANCE_PHASE_BRAKE,
                        now_ms,
                        brake_pitch_deg);
                } else {
                    BallBalanceControl_EnterPhase(
                        control, BALL_BALANCE_PHASE_COAST, now_ms, 0.0f);
                }
                break;
            }
            if (velocity_abs > BALL_BALANCE_BALL_STOPPED_SPEED_MM_S) {
                BallBalanceControl_EnterPhase(
                    control, BALL_BALANCE_PHASE_SETTLE, now_ms, 0.0f);
                break;
            }
            if (error_abs >= BALL_BALANCE_HOLD_EXIT_MM) {
                if (control->stationary_sample_count < 0xFFU) {
                    control->stationary_sample_count++;
                }
            } else {
                control->stationary_sample_count = 0U;
            }
            if (control->stationary_sample_count >=
                BALL_BALANCE_STATIONARY_SAMPLE_COUNT) {
                BallBalanceControl_EnterStationaryAcceleration(
                    control, now_ms);
            }
            break;
    }
}

static uint8_t BallBalanceControl_AcceptMeasurement(
    BallBalanceControl *control,
    uint32_t now_ms,
    const BallBalanceMeasurement *measurement)
{
    uint32_t elapsed_ms;
    float measurement_dt_s;
    float position_mm;
    float previous_position;
    float raw_velocity;
    float alpha;
    uint8_t filter_was_ready;

    if (measurement == 0) {
        return 0U;
    }
    if ((control->sequence_ready != 0U) &&
        (measurement->sequence == control->last_sequence)) {
        return 0U;
    }

    control->sequence_ready = 1U;
    control->last_sequence = measurement->sequence;
    position_mm = (float) measurement->position_d10_mm * 0.1f;
    if ((measurement->valid == 0U) ||
        (measurement->quality < BALL_BALANCE_MIN_QUALITY) ||
        (BallBalanceControl_Abs(position_mm) >
         BALL_BALANCE_POSITION_LIMIT_MM)) {
        return 0U;
    }

    filter_was_ready = control->filter_ready;
    elapsed_ms = (uint32_t) (now_ms - control->last_valid_ms);
    if ((filter_was_ready == 0U) ||
        (elapsed_ms > BALL_BALANCE_DATA_TIMEOUT_MS)) {
        control->filtered_position_mm = position_mm;
        control->velocity_mm_s = 0.0f;
        control->integral_mm_s = 0.0f;
        control->filter_ready = 1U;
        measurement_dt_s = 0.0f;
    } else {
        measurement_dt_s = BallBalanceControl_Limit(
            (float) elapsed_ms * 0.001f,
            BALL_BALANCE_DT_MIN_S,
            BALL_BALANCE_MEASUREMENT_DT_MAX_S);
        previous_position = control->filtered_position_mm;
        alpha = measurement_dt_s /
                (BALL_BALANCE_POSITION_FILTER_TAU_S + measurement_dt_s);
        control->filtered_position_mm += alpha *
            (position_mm - control->filtered_position_mm);
        raw_velocity = (control->filtered_position_mm - previous_position) /
                       measurement_dt_s;
        alpha = measurement_dt_s /
                (BALL_BALANCE_VELOCITY_FILTER_TAU_S + measurement_dt_s);
        control->velocity_mm_s += alpha *
            (raw_velocity - control->velocity_mm_s);
    }

    control->last_valid_ms = now_ms;
    control->last_measurement_dt_s = measurement_dt_s;
    control->tracking_valid = 1U;
    control->timed_out = 0U;
    return 1U;
}

static void BallBalanceControl_AcceptAttitude(
    BallBalanceControl *control,
    uint32_t now_ms,
    const BallBalanceAttitude *attitude)
{
    if ((attitude == 0) || (attitude->valid == 0U)) {
        return;
    }
    if ((control->attitude_ready != 0U) &&
        (attitude->timestamp_ms == control->last_attitude_timestamp_ms)) {
        return;
    }

    control->attitude_ready = 1U;
    control->last_attitude_ms = now_ms;
    control->last_attitude_timestamp_ms = attitude->timestamp_ms;
    control->measured_pipe_pitch_deg = attitude->pitch_deg;
    control->measured_pipe_rate_dps = attitude->pitch_rate_dps;
}

static uint8_t BallBalanceControl_AttitudeIsFresh(
    const BallBalanceControl *control,
    uint32_t now_ms)
{
    return (uint8_t) ((control->attitude_ready != 0U) &&
                      ((uint32_t) (now_ms - control->last_attitude_ms) <=
                       BALL_BALANCE_ATTITUDE_TIMEOUT_MS));
}

static void BallBalanceControl_UpdateEnableState(BallBalanceControl *control,
                                                 uint32_t now_ms)
{
    uint8_t attitude_fresh;
    uint8_t attitude_safe;
    uint8_t attitude_stable;

    if ((control->enabled_requested == 0U) ||
        (control->mapping_valid == 0U)) {
        control->enabled = 0U;
        control->attitude_stable_ready = 0U;
        return;
    }

    attitude_fresh = BallBalanceControl_AttitudeIsFresh(control, now_ms);
    attitude_safe =
        (uint8_t) ((attitude_fresh != 0U) &&
                   (BallBalanceControl_Abs(control->measured_pipe_pitch_deg) <=
                    BALL_BALANCE_PIPE_SAFETY_LIMIT_DEG));
    if (control->enabled != 0U) {
        if (attitude_safe == 0U) {
            control->enabled = 0U;
            control->imu_fault = 1U;
            control->attitude_stable_ready = 0U;
        }
        return;
    }

    attitude_stable =
        (uint8_t) ((attitude_safe != 0U) &&
                   (BallBalanceControl_Abs(control->measured_pipe_pitch_deg) <=
                    BALL_BALANCE_ENABLE_PITCH_LIMIT_DEG) &&
                   (BallBalanceControl_Abs(control->measured_pipe_rate_dps) <=
                    BALL_BALANCE_ENABLE_RATE_LIMIT_DPS));
    if (attitude_stable == 0U) {
        control->attitude_stable_ready = 0U;
        return;
    }

    if (control->attitude_stable_ready == 0U) {
        control->attitude_stable_ready = 1U;
        control->attitude_stable_since_ms = now_ms;
        return;
    }
    if ((uint32_t) (now_ms - control->attitude_stable_since_ms) >=
        BALL_BALANCE_ATTITUDE_STABLE_MS) {
        control->enabled = 1U;
        control->imu_fault = 0U;
        control->enabled_since_ms = now_ms;
        BallBalanceControl_ClearVisionState(control, 1U);
        /* The enable gate has already held the same strict stillness
         * condition longer than the trajectory settle gate. */
        control->pipe_settled = 1U;
        control->pipe_settle_timer_ready = 1U;
        control->pipe_settle_since_ms = now_ms;
    }
}

static void BallBalanceControl_UpdateServoCommand(BallBalanceControl *control,
                                                  float task_dt_s,
                                                  uint32_t now_ms)
{
    float corrected_pipe_command = 0.0f;
    float feedforward_angle_deg;
    float desired_angle_deg;
    float maximum_delta;
    uint8_t attitude_fresh;
    uint8_t mapping_ok;

    attitude_fresh = BallBalanceControl_AttitudeIsFresh(control, now_ms);
    if ((control->enabled != 0U) && (attitude_fresh != 0U)) {
        control->pipe_pitch_error_deg = control->command_pipe_pitch_deg -
                                        control->measured_pipe_pitch_deg;
        if (BallBalanceControl_Abs(control->measured_pipe_rate_dps) <=
            BALL_BALANCE_PIPE_SETTLE_RATE_DPS) {
            control->pipe_p_term_deg = BALL_BALANCE_PIPE_KP *
                                       control->pipe_pitch_error_deg;
            control->pipe_d_term_deg = -BALL_BALANCE_PIPE_KD_S *
                                       control->measured_pipe_rate_dps;
        } else {
            /* While the linkage is moving or ringing, keep only the mapped
             * feedforward trajectory.  Repeated IMU corrections at this
             * point pump energy into the crank-rocker oscillation. */
            control->pipe_p_term_deg = 0.0f;
            control->pipe_d_term_deg = 0.0f;
        }
        corrected_pipe_command = BallBalanceControl_LimitActualPitch(
            control,
            control->command_pipe_pitch_deg + control->pipe_p_term_deg +
            control->pipe_d_term_deg);
    } else {
        control->pipe_pitch_error_deg = 0.0f;
        control->pipe_p_term_deg = 0.0f;
        control->pipe_d_term_deg = 0.0f;
    }

    if (control->mapping_valid != 0U) {
        mapping_ok = (uint8_t) (
            (BallBalanceControl_MapFeedforward(
                control,
                (control->enabled != 0U) ?
                control->command_pipe_pitch_deg : 0.0f,
                &feedforward_angle_deg) != 0U) &&
            (BallBalanceControl_MapDesired(
                control,
                (control->enabled != 0U) ?
                corrected_pipe_command : 0.0f,
                &desired_angle_deg) != 0U));
        if (mapping_ok != 0U) {
            control->servo_feedforward_deg = feedforward_angle_deg;
            control->desired_angle_deg = desired_angle_deg;
        } else {
            BallBalanceControl_InvalidateMapping(control);
        }
    }

    if (control->mapping_valid == 0U) {
        control->feedforward_model_pitch_deg = 0.0f;
        control->desired_model_pitch_deg = 0.0f;
        control->servo_feedforward_deg =
            LinkageKinematics_GetNominalServoCenterDeg();
        control->desired_angle_deg =
            LinkageKinematics_GetNominalServoCenterDeg();
    }

    control->desired_angle_deg = BallBalanceControl_Limit(
        control->desired_angle_deg,
        BALL_BALANCE_MIN_ANGLE_DEG,
        BALL_BALANCE_MAX_ANGLE_DEG);
    if (task_dt_s > 0.0f) {
        maximum_delta = BALL_BALANCE_SERVO_SLEW_DEG_S * task_dt_s;
        control->command_angle_deg = BallBalanceControl_Approach(
            control->command_angle_deg,
            control->desired_angle_deg,
            maximum_delta);
    }
    control->command_angle_deg = BallBalanceControl_Limit(
        control->command_angle_deg,
        BALL_BALANCE_MIN_ANGLE_DEG,
        BALL_BALANCE_MAX_ANGLE_DEG);
}

static void BallBalanceControl_FillOutput(const BallBalanceControl *control,
                                          uint8_t measurement_accepted,
                                          uint32_t now_ms,
                                          BallBalanceOutput *output)
{
    if (output == 0) {
        return;
    }

    memset(output, 0, sizeof(*output));
    if (control == 0) {
        return;
    }

    output->enable_requested = control->enabled_requested;
    output->enabled = control->enabled;
    output->enable_pending =
        (uint8_t) ((control->enabled_requested != 0U) &&
                   (control->enabled == 0U));
    output->tracking_valid = control->tracking_valid;
    output->timed_out = control->timed_out;
    output->measurement_accepted = measurement_accepted;
    output->attitude_valid = BallBalanceControl_AttitudeIsFresh(control,
                                                                now_ms);
    output->imu_fault = control->imu_fault;
    output->mapping_valid = control->mapping_valid;
    output->mapping_fault = control->mapping_fault;
    output->integral_active = control->integral_active;
    output->braking_active = control->braking_active;
    output->pipe_settled = control->pipe_settled;
    output->kick_attempt_count = control->kick_attempt_count;
    output->trajectory_phase = control->trajectory_phase;
    output->target_mm = control->target_mm;
    output->position_kp_deg_per_mm = control->position_kp_deg_per_mm;
    output->position_ki_deg_per_mm_s =
        control->position_ki_deg_per_mm_s;
    output->position_kd_deg_per_mm_s =
        control->position_kd_deg_per_mm_s;
    output->filtered_position_mm = control->filtered_position_mm;
    output->predicted_position_mm = control->predicted_position_mm;
    output->predicted_accel_mm_s2 = control->predicted_accel_mm_s2;
    output->velocity_mm_s = control->velocity_mm_s;
    output->integral_mm_s = control->integral_mm_s;
    output->error_mm = control->error_mm;
    output->p_term_deg = control->p_term_deg;
    output->i_term_deg = control->i_term_deg;
    output->d_term_deg = control->d_term_deg;
    output->brake_term_deg = control->brake_term_deg;
    output->phase_pitch_deg = control->phase_pitch_deg;
    output->stop_distance_mm = control->stop_distance_mm;
    output->desired_pipe_pitch_deg = control->desired_pipe_pitch_deg;
    output->command_pipe_pitch_deg = control->command_pipe_pitch_deg;
    output->pipe_command_rate_dps = control->pipe_command_rate_dps;
    output->measured_pipe_pitch_deg = control->measured_pipe_pitch_deg;
    output->measured_pipe_rate_dps = control->measured_pipe_rate_dps;
    output->pipe_pitch_error_deg = control->pipe_pitch_error_deg;
    output->pipe_p_term_deg = control->pipe_p_term_deg;
    output->pipe_d_term_deg = control->pipe_d_term_deg;
    output->model_scale = control->model_scale;
    output->model_offset_deg = control->model_offset_deg;
    memcpy(output->mapping_pitch_deg,
           control->mapping_pitch_deg,
           sizeof(output->mapping_pitch_deg));
    output->minimum_pipe_pitch_deg = control->minimum_pipe_pitch_deg;
    output->maximum_pipe_pitch_deg = control->maximum_pipe_pitch_deg;
    output->feedforward_model_pitch_deg =
        control->feedforward_model_pitch_deg;
    output->desired_model_pitch_deg = control->desired_model_pitch_deg;
    output->servo_feedforward_deg = control->servo_feedforward_deg;
    output->desired_angle_deg = control->desired_angle_deg;
    output->command_angle_deg = control->command_angle_deg;
}

void BallBalanceControl_Init(BallBalanceControl *control)
{
    float neutral_angle_deg;

    if (control == 0) {
        return;
    }

    memset(control, 0, sizeof(*control));
    neutral_angle_deg = LinkageKinematics_GetNominalServoCenterDeg();
    control->desired_angle_deg = neutral_angle_deg;
    control->command_angle_deg = neutral_angle_deg;
    control->servo_feedforward_deg = neutral_angle_deg;
    control->position_kp_deg_per_mm =
        BALL_BALANCE_DEFAULT_KP_DEG_PER_MM;
    control->position_ki_deg_per_mm_s =
        BALL_BALANCE_DEFAULT_KI_DEG_PER_MM_S;
    control->position_kd_deg_per_mm_s =
        BALL_BALANCE_DEFAULT_KD_DEG_PER_MM_S;
}

void BallBalanceControl_SetEnabled(BallBalanceControl *control,
                                   uint8_t enabled,
                                   uint32_t now_ms)
{
    uint8_t normalized_enabled;

    if (control == 0) {
        return;
    }

    normalized_enabled = (enabled != 0U) ? 1U : 0U;
    if ((normalized_enabled != 0U) && (control->mapping_valid == 0U)) {
        return;
    }
    if (control->enabled_requested == normalized_enabled) {
        return;
    }

    control->enabled_requested = normalized_enabled;
    control->enabled = 0U;
    control->imu_fault = 0U;
    control->attitude_stable_ready = 0U;
    control->enabled_since_ms = now_ms;
    control->last_task_ms = now_ms;
    control->task_time_ready = 1U;
    BallBalanceControl_ClearVisionState(control, 1U);
}

uint8_t BallBalanceControl_IsEnabled(const BallBalanceControl *control)
{
    return (control != 0) ? control->enabled_requested : 0U;
}

uint8_t BallBalanceControl_IsActive(const BallBalanceControl *control)
{
    return (control != 0) ? control->enabled : 0U;
}

void BallBalanceControl_SetTargetMm(BallBalanceControl *control,
                                    float target_mm)
{
    float limited_target;

    if (control == 0) {
        return;
    }

    limited_target = BallBalanceControl_Limit(
        target_mm,
        -BALL_BALANCE_TARGET_LIMIT_MM,
        BALL_BALANCE_TARGET_LIMIT_MM);
    if (limited_target != control->target_mm) {
        control->target_mm = limited_target;
        control->integral_mm_s = 0.0f;
        control->i_term_deg = 0.0f;
        control->trajectory_phase = BALL_BALANCE_PHASE_WAIT_PIPE;
        control->phase_pitch_deg = 0.0f;
        control->desired_pipe_pitch_deg = 0.0f;
        control->stationary_sample_count = 0U;
    }
}

void BallBalanceControl_SetPidGains(BallBalanceControl *control,
                                    float kp_deg_per_mm,
                                    float ki_deg_per_mm_s,
                                    float kd_deg_per_mm_s)
{
    if (control == 0) {
        return;
    }

    if ((kp_deg_per_mm >= 0.0f) &&
        (kp_deg_per_mm <= BALL_BALANCE_KP_MAX_DEG_PER_MM)) {
        control->position_kp_deg_per_mm = kp_deg_per_mm;
    }
    if ((ki_deg_per_mm_s >= 0.0f) &&
        (ki_deg_per_mm_s <= BALL_BALANCE_KI_MAX_DEG_PER_MM_S)) {
        control->position_ki_deg_per_mm_s = ki_deg_per_mm_s;
        if (ki_deg_per_mm_s <= BALL_BALANCE_GAIN_EPSILON) {
            control->integral_mm_s = 0.0f;
            control->i_term_deg = 0.0f;
            control->integral_active = 0U;
        }
    }
    if ((kd_deg_per_mm_s >= 0.0f) &&
        (kd_deg_per_mm_s <= BALL_BALANCE_KD_MAX_DEG_PER_MM_S)) {
        control->position_kd_deg_per_mm_s = kd_deg_per_mm_s;
    }
}

void BallBalanceControl_SetPipeCalibration(
    BallBalanceControl *control,
    uint8_t valid,
    float model_scale,
    float model_offset_deg,
    const float mapping_pitch_deg[
        LINKAGE_KINEMATICS_CALIBRATION_POINT_COUNT])
{
    float neutral_model_pitch_deg;
    float neutral_servo_angle_deg;
    float minimum_pipe_pitch_deg;
    float maximum_pipe_pitch_deg;
    uint8_t calibration_valid;
    uint8_t mapping_changed = 0U;
    uint8_t index;

    if (control == 0) {
        return;
    }

    calibration_valid =
        (uint8_t) ((valid != 0U) && (mapping_pitch_deg != 0) &&
                   (model_scale >= BALL_BALANCE_MODEL_SCALE_MIN) &&
                   (model_scale <= BALL_BALANCE_MODEL_SCALE_MAX) &&
                   (BallBalanceControl_Abs(model_offset_deg) <=
                    BALL_BALANCE_MODEL_OFFSET_MAX_DEG));
    if (calibration_valid != 0U) {
        for (index = 1U;
             index < LINKAGE_KINEMATICS_CALIBRATION_POINT_COUNT;
             index++) {
            if ((mapping_pitch_deg[index] -
                 mapping_pitch_deg[index - 1U]) <
                BALL_BALANCE_MAPPING_MIN_STEP_DEG) {
                calibration_valid = 0U;
                break;
            }
        }
    }

    if (calibration_valid == 0U) {
        if (control->mapping_valid != 0U) {
            BallBalanceControl_InvalidateMapping(control);
        }
        return;
    }

    minimum_pipe_pitch_deg = BallBalanceControl_Limit(
        mapping_pitch_deg[0],
        BALL_BALANCE_MIN_PIPE_PITCH_DEG,
        BALL_BALANCE_MAX_PIPE_PITCH_DEG);
    maximum_pipe_pitch_deg = BallBalanceControl_Limit(
        mapping_pitch_deg[
            LINKAGE_KINEMATICS_CALIBRATION_POINT_COUNT - 1U],
        BALL_BALANCE_MIN_PIPE_PITCH_DEG,
        BALL_BALANCE_MAX_PIPE_PITCH_DEG);
    if ((minimum_pipe_pitch_deg >= 0.0f) ||
        (maximum_pipe_pitch_deg <= 0.0f)) {
        if (control->mapping_valid != 0U) {
            BallBalanceControl_InvalidateMapping(control);
        }
        return;
    }

    if ((control->mapping_valid == 0U) ||
        (BallBalanceControl_Abs(control->model_scale - model_scale) >
         BALL_BALANCE_CACHE_EPSILON_DEG) ||
        (BallBalanceControl_Abs(control->model_offset_deg - model_offset_deg) >
         BALL_BALANCE_CACHE_EPSILON_DEG)) {
        mapping_changed = 1U;
    }
    for (index = 0U;
         index < LINKAGE_KINEMATICS_CALIBRATION_POINT_COUNT;
         index++) {
        if (BallBalanceControl_Abs(
                control->mapping_pitch_deg[index] -
                mapping_pitch_deg[index]) >
            BALL_BALANCE_CACHE_EPSILON_DEG) {
            mapping_changed = 1U;
        }
    }
    if (mapping_changed == 0U) {
        return;
    }

    control->model_scale = model_scale;
    control->model_offset_deg = model_offset_deg;
    memcpy(control->mapping_pitch_deg,
           mapping_pitch_deg,
           sizeof(control->mapping_pitch_deg));
    control->minimum_pipe_pitch_deg = minimum_pipe_pitch_deg;
    control->maximum_pipe_pitch_deg = maximum_pipe_pitch_deg;
    control->mapping_valid = 1U;
    control->mapping_fault = 0U;
    control->feedforward_cache_valid = 0U;
    control->desired_cache_valid = 0U;

    if ((BallBalanceControl_MapActualPitch(
             control,
             0.0f,
             &neutral_model_pitch_deg,
             &neutral_servo_angle_deg) == 0U) ||
        (neutral_servo_angle_deg < BALL_BALANCE_MIN_ANGLE_DEG) ||
        (neutral_servo_angle_deg > BALL_BALANCE_MAX_ANGLE_DEG)) {
        BallBalanceControl_InvalidateMapping(control);
    }
}

float BallBalanceControl_GetNeutralAngleDeg(void)
{
    return LinkageKinematics_GetNominalServoCenterDeg();
}

void BallBalanceControl_Update(BallBalanceControl *control,
                               uint32_t now_ms,
                               const BallBalanceMeasurement *measurement,
                               const BallBalanceAttitude *attitude,
                               BallBalanceOutput *output)
{
    uint32_t elapsed_ms;
    float task_dt_s = 0.0f;
    uint8_t measurement_accepted = 0U;

    if (control == 0) {
        BallBalanceControl_FillOutput(0, 0U, now_ms, output);
        return;
    }

    if (control->task_time_ready != 0U) {
        elapsed_ms = (uint32_t) (now_ms - control->last_task_ms);
        task_dt_s = BallBalanceControl_Limit(
            (float) elapsed_ms * 0.001f,
            0.0f,
            BALL_BALANCE_DT_MAX_S);
    } else {
        control->task_time_ready = 1U;
    }
    control->last_task_ms = now_ms;

    BallBalanceControl_AcceptAttitude(control, now_ms, attitude);
    BallBalanceControl_UpdateEnableState(control, now_ms);
    BallBalanceControl_UpdatePipeSettled(control, now_ms);

    if (control->enabled != 0U) {
        measurement_accepted = BallBalanceControl_AcceptMeasurement(
            control, now_ms, measurement);
        if (control->tracking_valid != 0U) {
            elapsed_ms = (uint32_t) (now_ms - control->last_valid_ms);
            if (elapsed_ms > BALL_BALANCE_DATA_TIMEOUT_MS) {
                BallBalanceControl_ClearVisionState(control, 0U);
                control->timed_out = 1U;
            } else {
                BallBalanceControl_UpdatePrediction(control, now_ms);
                BallBalanceControl_UpdateTrajectory(
                    control, now_ms, measurement_accepted);
            }
        } else if ((uint32_t) (now_ms - control->enabled_since_ms) >
                   BALL_BALANCE_DATA_TIMEOUT_MS) {
            control->timed_out = 1U;
            control->desired_pipe_pitch_deg = 0.0f;
        }
    } else {
        control->desired_pipe_pitch_deg = 0.0f;
        control->trajectory_phase = BALL_BALANCE_PHASE_WAIT_PIPE;
    }

    BallBalanceControl_UpdatePipeProfile(control, task_dt_s);

    BallBalanceControl_UpdateServoCommand(control, task_dt_s, now_ms);
    BallBalanceControl_FillOutput(control,
                                  measurement_accepted,
                                  now_ms,
                                  output);
}
