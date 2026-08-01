#include "ball_balance_control.h"

#include <math.h>
#include <string.h>

#define BALL_BALANCE_MIN_PIPE_PITCH_DEG       (-5.5495f) /* 500 us对应的实测最小相对水管角。 */
#define BALL_BALANCE_MAX_PIPE_PITCH_DEG          5.5495f  /* 1600 us对应的实测最大相对水管角。 */
#define BALL_BALANCE_PIPE_KP_MAX                  2.00f   /* IAR在线调整姿态Kp的安全上限。 */
#define BALL_BALANCE_PIPE_KD_MAX_S                0.30f   /* IAR在线调整姿态Kd的安全上限。 */
#define BALL_BALANCE_PIPE_SAFETY_LIMIT_DEG       8.5f  /* 实测俯仰超过该角度才暂停姿态闭环；实测范围±5.55°，起步变速惯性摆动可达7°左右，故留出余量。 */
#define BALL_BALANCE_IMU_FAULT_DEBOUNCE_MS        150U /* IMU姿态持续不安全达到该时长才判故障，容忍单帧抖动与瞬时过冲。 */
#define BALL_BALANCE_KP_MAX_DEG_PER_MM              0.200f /* Debug 可调 Kp 安全上限。 */
#define BALL_BALANCE_KI_MAX_DEG_PER_MM_S            0.100f /* Debug 可调 Ki 安全上限。 */
#define BALL_BALANCE_KD_MAX_DEG_PER_MM_S            0.200f /* Debug 可调 Kd 安全上限。 */
#define BALL_BALANCE_POSITION_P_LIMIT_DEG          5.30f /* 允许位置外环使用接近完整实测角度拉回端部钢球。 */
#define BALL_BALANCE_POSITION_D_LIMIT_DEG          3.0f /* 速度反馈只负责柔和制动，避免越零后反打过强。 */
#define BALL_BALANCE_RETURN_BRAKE_KP_DEG_PER_MM      0.008f /* 回程附加制动P的基准增益，单位deg/mm。 */
#define BALL_BALANCE_RETURN_BRAKE_SPEED_SCALE_MM_S  60.0f  /* 回程制动P随速度变增益的比例基准，速度越大制动越强。 */
#define BALL_BALANCE_RETURN_BRAKE_LIMIT_DEG          2.00f /* 回程附加制动P的输出限幅，单位deg。 */
#define BALL_BALANCE_RETURN_BRAKE_RANGE_MM          15.0f /* 回程附加制动P仅在距目标15毫米内启用，避免远离目标时提前刹车。 */
#define BALL_BALANCE_POSITION_DEADBAND_MM          4.0f  /* 目标点正负4毫米内不再主动修正位置。 */
#define BALL_BALANCE_STOPPED_SPEED_MM_S            3.0f  /* 只有球真正接近静止才进入零点保持。 */
#define BALL_BALANCE_BALL_MOVING_SPEED_MM_S         8.0f
#define BALL_BALANCE_BALL_STOPPED_SPEED_MM_S        5.0f
#define BALL_BALANCE_HOLD_ENTER_MM                  10.0f
#define BALL_BALANCE_HOLD_EXIT_MM                   15.0f
#define BALL_BALANCE_STATIONARY_SAMPLE_COUNT         2U
#define BALL_BALANCE_MOVING_SAMPLE_COUNT             2U
#define BALL_BALANCE_PIPE_SETTLE_ERROR_DEG            0.45f
#define BALL_BALANCE_PIPE_SETTLE_RATE_DPS             1.00f
#define BALL_BALANCE_PIPE_SETTLE_MS                  120U
#define BALL_BALANCE_ACCEL_STAGE_MS                  400U
#define BALL_BALANCE_ZERO_HOLD_FREEZE_MS             500U
#define BALL_BALANCE_TURNAROUND_FREEZE_MS            500U
#define BALL_BALANCE_ACCEL_PITCH_MIN_DEG              0.75f
#define BALL_BALANCE_ACCEL_POSITION_GAIN_DEG_PER_MM   0.014f
#define BALL_BALANCE_KICK_ATTEMPT_STEP_DEG              0.40f
#define BALL_BALANCE_KICK_MAX_PITCH_DEG                 5.30f /* 破静摩擦阶段允许接近实测安全角上限。 */
#define BALL_BALANCE_TURNAROUND_LEVEL_SPEED_MM_S       12.0f
#define BALL_BALANCE_INTEGRAL_OUTPUT_LIMIT_DEG     0.80f /* 普通目标点积分最大只补偿0.8度。 */
#define BALL_BALANCE_INTEGRAL_MIN_ERROR_MM          4.0f /* 距离目标不超过4毫米（死区内）时禁用位置积分。 */
#define BALL_BALANCE_NEAR_TARGET_I_RANGE_MM        20.0f /* 仅在距离目标4～20毫米时启用位置积分。 */
#define BALL_BALANCE_ENDPOINT_KI_DEG_PER_MM_S     0.020f /* 第三题最终端点保持使用的独立积分增益。 */
#define BALL_BALANCE_ENDPOINT_I_LIMIT_DEG          1.00f /* 端点积分允许补偿的最大持续俯仰角。 */
#define BALL_BALANCE_STICTION_INTEGRAL_BOOST       3.0f  /* 钢球静止时加快积分以突破静摩擦。 */
#define BALL_BALANCE_MOVING_INTEGRAL_LEAK_TAU_S    0.80f /* 钢球运动后泄放积分以降低过冲。 */
#define BALL_BALANCE_GAIN_EPSILON                  0.000001f
#define BALL_BALANCE_MIN_QUALITY                  50U    /* 接受 K230 测量所需的最低质量。 */
#define BALL_BALANCE_VISION_SPIKE_JUMP_MM         15.0f  /* K230 一帧位置跳变超过该值视为跳变，抑制假速度。 */
#define BALL_BALANCE_SPIKE_STUCK_MS               400U   /* 跳变持续超过该时间仍偏离，判定为真实新位置并重采样。 */
#define BALL_BALANCE_TRANSPORT_SPIKE_JUMP_MM      30.0f  /* AB/ABCD 行驶中允许相邻低帧率视觉样本产生更大真实位移。 */
#define BALL_BALANCE_TRANSPORT_SPIKE_STUCK_MS     150U   /* 行驶时更快接受持续位移，避免长期使用旧位置盲控。 */
#define BALL_BALANCE_IDLE_DATA_TIMEOUT_MS      1000U    /* 等待/破静摩擦时容忍 YOLO 短时漏检，避免杆反复回平重启。 */
#define BALL_BALANCE_MOVING_DATA_TIMEOUT_MS     350U    /* 球已运动后仍快速退出盲控，避免失联时继续加速。 */
#define BALL_BALANCE_ATTITUDE_TIMEOUT_MS        100U    /* IMU 姿态数据超时时间。 */
#define BALL_BALANCE_ATTITUDE_STABLE_MS         150U    /* 启用前只确认短时间稳定，避免点击后反应迟钝。 */
#define BALL_BALANCE_ENABLE_PITCH_LIMIT_DEG       0.60f
#define BALL_BALANCE_ENABLE_RATE_LIMIT_DPS        1.00f
#define BALL_BALANCE_POSITION_LIMIT_MM          130.0f  /* 合法钢球测量位置的绝对上限。 */
#define BALL_BALANCE_PIPE_PROFILE_MAX_RATE_DPS    12.0f
#define BALL_BALANCE_PIPE_PROFILE_ACCEL_DPS2      60.0f
#define BALL_BALANCE_PIPE_BRAKE_MAX_RATE_DPS      20.0f
#define BALL_BALANCE_PIPE_BRAKE_ACCEL_DPS2       120.0f
#define BALL_BALANCE_ACTUATOR_SLEW_DPS             20.0f /* 姿态修正后最终相对水管角的变化率上限。 */
#define BALL_BALANCE_DT_MIN_S                     0.010f /* 视觉测量 dt 的最小值。 */
#define BALL_BALANCE_DT_MAX_S                     0.100f /* 视觉测量和任务 dt 的最大值。 */
#define BALL_BALANCE_MEASUREMENT_DT_MAX_S         0.250f /* 兼容 K230 约 5~6 FPS 的真实帧间隔。 */
#define BALL_BALANCE_POSITION_FILTER_TAU_S        0.080f /* 适度平滑位置检测抖动。 */
#define BALL_BALANCE_VELOCITY_FILTER_TAU_S        0.120f /* 平滑速度同时保留低帧率下的制动响应。 */
#define BALL_BALANCE_VISION_LOOKAHEAD_S            0.25f /* 补偿 K230 推理和串口位置数据的固有延迟。 */
#define BALL_BALANCE_PREDICTION_MAX_S              0.45f /* 限制丢帧时的位置外推时间。 */
#define BALL_BALANCE_BALL_ACCEL_SIGN_PER_PITCH    (-1.0f) /* 统一物理约定：正杆角使 K230 位置向负向加速。 */
#define BALL_BALANCE_ACCEL_PER_PIPE_DEG_MM_S2     100.0f
#define BALL_BALANCE_CACHE_EPSILON_DEG              0.0001f /* 状态切换允许的浮点角度误差。 */
#define BALL_BALANCE_TRANSPORT_ACCEL_TO_DEG       0.122f   /* 实测 1/g_eff（g_eff≈8.2 cm/s2/°），输入为 cm/s^2。 */
#define BALL_BALANCE_TRANSPORT_IMU_BLEND              0.0f /* 匀速直线段砍掉 IMU 混合，前馈只用规划值。 */
#define BALL_BALANCE_TRANSPORT_IMU_LIMIT_DEG           2.00f /* 防止振动或侧向冲击污染纵向补偿。 */
#define BALL_BALANCE_TRANSPORT_FF_LIMIT_DEG            3.00f /* 起步和停车前馈的最大杆角（配合 0.122 系数）。 */
#define BALL_BALANCE_TRANSPORT_WALL_ACCEL_LIMIT_CM_S2  30.0f /* 管壁等效轴向加速度的实车调参保护。 */
#define BALL_BALANCE_TRANSPORT_WALL_FF_LIMIT_DEG        0.50f /* 弯道管壁补偿不得占满水管角度余量。 */
#define BALL_BALANCE_TRANSPORT_KP_DEG_PER_MM          0.012f /* 行驶中小幅位置修正增益。 */
#define BALL_BALANCE_TRANSPORT_KD_DEG_PER_MM_S        0.000f /* 低帧率下速度阻尼不可靠，匀速段砍掉。 */
#define BALL_BALANCE_TRANSPORT_TRANSIENT_DEADBAND_MM    2.0f /* 起步/停车时保留较灵敏的位置修正。 */
#define BALL_BALANCE_TRANSPORT_CRUISE_DEADBAND_MM       5.0f /* 匀速段不追逐中心附近的视觉与车体小扰动。 */
#define BALL_BALANCE_TRANSPORT_CRUISE_ACCEL_LIMIT_CM_S2 1.0f /* 与底盘匀速判定的加速度容差一致。 */
#define BALL_BALANCE_TRANSPORT_P_LIMIT_DEG             0.38f /* 弯道位置修正保持小幅。 */
#define BALL_BALANCE_TRANSPORT_D_LIMIT_DEG             0.18f /* 弯道速度阻尼限幅。 */
#define BALL_BALANCE_TRANSPORT_FB_LIMIT_DEG            0.45f /* 行驶中总位置修正限幅。 */
#define BALL_BALANCE_TRANSPORT_RECOVERY_ENTER_MM        7.0f /* 预测位置接近题目边界前提前强回收。 */
#define BALL_BALANCE_TRANSPORT_RECOVERY_EXIT_MM         5.0f /* 回到目标且降速后才退出。 */
#define BALL_BALANCE_POSITION_COMPARE_EPSILON_MM      0.001f /* 浮点位置边界比较容差，单位为毫米。 */
#define BALL_BALANCE_TRANSPORT_RECOVERY_EXIT_SPEED_MM_S 10.0f
#define BALL_BALANCE_TRANSPORT_RECOVERY_KP_DEG_PER_MM   0.020f
#define BALL_BALANCE_TRANSPORT_RECOVERY_KD_DEG_PER_MM_S 0.016f
#define BALL_BALANCE_TRANSPORT_RECOVERY_MIN_P_DEG       0.75f /* 越界后给足克服机构迟滞的最小回拉角。 */
#define BALL_BALANCE_TRANSPORT_RECOVERY_P_LIMIT_DEG     1.20f
#define BALL_BALANCE_TRANSPORT_RECOVERY_D_LIMIT_DEG     1.00f
#define BALL_BALANCE_TRANSPORT_RECOVERY_FB_LIMIT_DEG    1.50f

static uint8_t BallBalanceControl_AttitudeIsFresh(
    const BallBalanceControl *control,
    uint32_t now_ms);

static uint32_t BallBalanceControl_DataTimeoutMs(
    const BallBalanceControl *control)
{
    if ((control != 0) &&
        (control->trajectory_phase == BALL_BALANCE_PHASE_MOVING)) {
        return BALL_BALANCE_MOVING_DATA_TIMEOUT_MS;
    }
    return BALL_BALANCE_IDLE_DATA_TIMEOUT_MS;
}

static float BallBalanceControl_Abs(float value)
{
    return (value < 0.0f) ? -value : value;
}

static float BallBalanceControl_ApplyPositionDeadband(float error_mm)
{
    if (error_mm > BALL_BALANCE_POSITION_DEADBAND_MM) {
        return error_mm - BALL_BALANCE_POSITION_DEADBAND_MM;
    }
    if (error_mm < -BALL_BALANCE_POSITION_DEADBAND_MM) {
        return error_mm + BALL_BALANCE_POSITION_DEADBAND_MM;
    }
    return 0.0f;
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
    profile_accel_dps2 = BALL_BALANCE_PIPE_PROFILE_ACCEL_DPS2;
    profile_max_rate_dps = BALL_BALANCE_PIPE_PROFILE_MAX_RATE_DPS;
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
    control->transport_recovery_active = 0U;
    control->zero_hold_freeze_active = 0U;
    control->turnaround_freeze_active = 0U;
    control->outward_motion_seen = 0U;
    control->previous_error_ready = 0U;
    control->zero_crossed = 0U;
    control->pipe_settled = 0U;
    control->pipe_settle_timer_ready = 0U;
    control->stationary_sample_count = 0U;
    control->moving_sample_count = 0U;
    control->moving_direction = 0;
    control->kick_attempt_count = 0U;
    control->trajectory_phase = BALL_BALANCE_PHASE_WAIT_PIPE;
    if (clear_sequence != 0U) {
        control->sequence_ready = 0U;
        control->last_sequence = 0U;
    }
    control->last_valid_ms = 0U;
    control->spike_reject_active = 0U;
    control->last_spike_reject_ms = 0U;
    control->pipe_settle_since_ms = 0U;
    control->phase_started_ms = 0U;
    control->kick_stage_started_ms = 0U;
    control->turnaround_freeze_started_ms = 0U;
    control->last_measurement_dt_s = 0.0f;
    control->filtered_position_mm = 0.0f;
    control->predicted_position_mm = 0.0f;
    control->predicted_accel_mm_s2 = 0.0f;
    control->velocity_mm_s = 0.0f;
    control->integral_mm_s = 0.0f;
    control->error_mm = 0.0f;
    control->previous_error_mm = 0.0f;
    control->p_term_deg = 0.0f;
    control->i_term_deg = 0.0f;
    control->d_term_deg = 0.0f;
    control->brake_term_deg = 0.0f;
    control->phase_pitch_deg = 0.0f;
    control->stop_distance_mm = 0.0f;
    control->desired_pipe_pitch_deg = 0.0f;
    control->pipe_command_rate_dps = 0.0f;
}

static void BallBalanceControl_ResetControlStatePreserveVision(
    BallBalanceControl *control)
{
    /* 第六题启用时保留最新位置和速度观测，只清除上一任务的执行状态。 */
    control->timed_out = 0U;
    control->integral_active = 0U;
    control->braking_active = 0U;
    control->transport_recovery_active = 0U;
    control->zero_hold_freeze_active = 0U;
    control->turnaround_freeze_active = 0U;
    control->outward_motion_seen = 0U;
    control->previous_error_ready = 0U;
    control->zero_crossed = 0U;
    control->pipe_settled = 0U;
    control->pipe_settle_timer_ready = 0U;
    control->stationary_sample_count = 0U;
    control->moving_sample_count = 0U;
    control->moving_direction = 0;
    control->kick_attempt_count = 0U;
    control->trajectory_phase = BALL_BALANCE_PHASE_WAIT_PIPE;
    control->spike_reject_active = 0U;
    control->last_spike_reject_ms = 0U;
    control->pipe_settle_since_ms = 0U;
    control->phase_started_ms = 0U;
    control->kick_stage_started_ms = 0U;
    control->turnaround_freeze_started_ms = 0U;
    control->last_measurement_dt_s = 0.0f;
    control->integral_mm_s = 0.0f;
    control->error_mm = 0.0f;
    control->previous_error_mm = 0.0f;
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
    control->preserve_vision_on_enable = 0U;
    control->attitude_stable_ready = 0U;
    control->feedforward_cache_valid = 0U;
    control->desired_cache_valid = 0U;
    BallBalanceControl_ClearVisionState(control, 1U);
}

static void BallBalanceControl_FillFixedMapping(BallBalanceControl *control)
{
    float span = BALL_BALANCE_MAX_PIPE_PITCH_DEG -
                 BALL_BALANCE_MIN_PIPE_PITCH_DEG;
    uint8_t index;

    for (index = 0U;
         index < LINKAGE_KINEMATICS_CALIBRATION_POINT_COUNT;
         index++) {
        control->mapping_pitch_deg[index] =
            BALL_BALANCE_MIN_PIPE_PITCH_DEG +
            span * (float) index /
            (float) (LINKAGE_KINEMATICS_CALIBRATION_POINT_COUNT - 1U);
    }
    control->model_scale = 1.0f;
    control->model_offset_deg = 0.0f;
    control->minimum_pipe_pitch_deg = BALL_BALANCE_MIN_PIPE_PITCH_DEG;
    control->maximum_pipe_pitch_deg = BALL_BALANCE_MAX_PIPE_PITCH_DEG;
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
            BALL_BALANCE_BALL_ACCEL_SIGN_PER_PITCH *
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
    control->zero_hold_freeze_active = 0U;
    control->turnaround_freeze_active = 0U;
    control->outward_motion_seen = 0U;
    control->phase_started_ms = now_ms;
    if (phase == BALL_BALANCE_PHASE_BREAKAWAY) {
        control->kick_stage_started_ms = now_ms;
    }
    control->phase_pitch_deg = pitch_deg;
    control->desired_pipe_pitch_deg = pitch_deg;
    control->stationary_sample_count = 0U;
    control->moving_sample_count = 0U;
    control->moving_direction = 0;
    control->braking_active = 0U;
    control->brake_term_deg = 0.0f;
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
    /* With ball_acceleration = -K * pipe_pitch, pitch has the same sign as
     * position error to produce restoring acceleration. */
    return (error_mm >= 0.0f) ? pitch_deg : -pitch_deg;
}

static void BallBalanceControl_EnterStationaryAcceleration(
    BallBalanceControl *control,
    uint32_t now_ms)
{
    BallBalanceControl_EnterPhase(
        control,
        BALL_BALANCE_PHASE_BREAKAWAY,
        now_ms,
        BallBalanceControl_SelectAccelerationPitch(
            control->error_mm, control->kick_attempt_count));
    /* The transition into this helper is already based on two consecutive
     * stopped K230 frames.  Preserve that evidence for staged breakaway. */
    control->stationary_sample_count =
        BALL_BALANCE_STATIONARY_SAMPLE_COUNT;
}

static void BallBalanceControl_UpdateNearTargetIntegral(
    BallBalanceControl *control,
    uint8_t measurement_accepted,
    float error_abs,
    float velocity_abs)
{
    float ki_deg_per_mm_s;
    float output_limit_deg;
    float integral_limit_mm_s;
    float integral_error_mm;
    float leak_ratio;

    integral_error_mm = BallBalanceControl_ApplyPositionDeadband(
        control->error_mm);
    ki_deg_per_mm_s = control->position_ki_deg_per_mm_s;
    output_limit_deg = BALL_BALANCE_INTEGRAL_OUTPUT_LIMIT_DEG;
    if (control->endpoint_hold_enabled != 0U) {
        /* 第三题最终端点继续使用更强的独立积分，避免终点静摩擦残差。 */
        ki_deg_per_mm_s = BALL_BALANCE_ENDPOINT_KI_DEG_PER_MM_S;
        output_limit_deg = BALL_BALANCE_ENDPOINT_I_LIMIT_DEG;
    }

    if ((ki_deg_per_mm_s <= BALL_BALANCE_GAIN_EPSILON) ||
        (error_abs <= BALL_BALANCE_INTEGRAL_MIN_ERROR_MM) ||
        (error_abs > BALL_BALANCE_NEAR_TARGET_I_RANGE_MM) ||
        (control->zero_crossed != 0U)) {
        control->integral_mm_s = 0.0f;
        control->i_term_deg = 0.0f;
        control->integral_active = 0U;
        return;
    }

    if ((measurement_accepted != 0U) &&
        (control->last_measurement_dt_s > 0.0f)) {
        if (velocity_abs <= BALL_BALANCE_BALL_STOPPED_SPEED_MM_S) {
            /* 静止残差通常来自机构静摩擦，适度加快积分消除稳态误差。 */
            control->integral_mm_s += integral_error_mm *
                control->last_measurement_dt_s *
                BALL_BALANCE_STICTION_INTEGRAL_BOOST;
        } else {
            /* 小球运动时先泄放旧积分，只在继续远离目标时缓慢累积。 */
            leak_ratio = 1.0f - control->last_measurement_dt_s /
                BALL_BALANCE_MOVING_INTEGRAL_LEAK_TAU_S;
            leak_ratio = BallBalanceControl_Limit(leak_ratio, 0.0f, 1.0f);
            control->integral_mm_s *= leak_ratio;
            if ((integral_error_mm * control->velocity_mm_s) > 0.0f) {
                control->integral_mm_s += integral_error_mm *
                    control->last_measurement_dt_s;
            }
        }
    }

    /* 同时钳位积分状态和输出，防止限幅期间在内部继续积累。 */
    integral_limit_mm_s = output_limit_deg / ki_deg_per_mm_s;
    control->integral_mm_s = BallBalanceControl_Limit(
        control->integral_mm_s,
        -integral_limit_mm_s,
        integral_limit_mm_s);
    control->i_term_deg = ki_deg_per_mm_s * control->integral_mm_s;
    control->integral_active = 1U;
}

static float BallBalanceControl_PitchFromFeedbackEffort(float effort_deg)
{
    return -effort_deg / BALL_BALANCE_BALL_ACCEL_SIGN_PER_PITCH;
}

static float BallBalanceControl_CalculateMovingPitch(
    BallBalanceControl *control)
{
    float feedback_effort_deg;
    float feedback_error_mm;
    float pitch_deg;
    float ball_accel_direction;
    uint8_t moving_toward_target;

    feedback_error_mm = BallBalanceControl_ApplyPositionDeadband(
        control->error_mm);
    moving_toward_target = (uint8_t) (
        (control->error_mm * control->velocity_mm_s) < 0.0f);
    if (moving_toward_target != 0U) {
        /* 回程（球朝目标运动）时位置反馈为反向制动：
         * 若使用正Kp*error，杆角与球速同向会加速球穿过目标造成过冲，
         * 故取负号把球往回来路方向推；增益随回程速度增大（变增益），
         * 在高速滑行段补齐纯D制动不足，低速接近目标时自动减弱避免抖动。
         * 仅在距目标15毫米内启用——离目标远时保持动量冲向目标，
         * 避免整段回程都在刹车导致球永远够不到目标。 */
        if (BallBalanceControl_Abs(feedback_error_mm) <=
            BALL_BALANCE_RETURN_BRAKE_RANGE_MM) {
            control->p_term_deg = BallBalanceControl_Limit(
                -BALL_BALANCE_RETURN_BRAKE_KP_DEG_PER_MM *
                    feedback_error_mm *
                    (1.0f + BallBalanceControl_Abs(control->velocity_mm_s) /
                            BALL_BALANCE_RETURN_BRAKE_SPEED_SCALE_MM_S),
                -BALL_BALANCE_RETURN_BRAKE_LIMIT_DEG,
                BALL_BALANCE_RETURN_BRAKE_LIMIT_DEG);
        } else {
            control->p_term_deg = 0.0f;
        }
    } else {
        control->p_term_deg = BallBalanceControl_Limit(
            control->position_kp_deg_per_mm * feedback_error_mm,
            -BALL_BALANCE_POSITION_P_LIMIT_DEG,
            BALL_BALANCE_POSITION_P_LIMIT_DEG);
    }
    control->d_term_deg = BallBalanceControl_Limit(
        control->position_kd_deg_per_mm_s * control->velocity_mm_s,
        -BALL_BALANCE_POSITION_D_LIMIT_DEG,
        BALL_BALANCE_POSITION_D_LIMIT_DEG);
    feedback_effort_deg = control->p_term_deg + control->i_term_deg +
                          control->d_term_deg;
    pitch_deg = BallBalanceControl_PitchFromFeedbackEffort(
        feedback_effort_deg);
    pitch_deg = BallBalanceControl_LimitActualPitch(control, pitch_deg);

    ball_accel_direction = BALL_BALANCE_BALL_ACCEL_SIGN_PER_PITCH *
                           pitch_deg;
    control->braking_active = (uint8_t) (
        (control->velocity_mm_s * ball_accel_direction) < 0.0f);
    control->brake_term_deg = BallBalanceControl_PitchFromFeedbackEffort(
        control->p_term_deg + control->d_term_deg);
    if ((control->braking_active != 0U) &&
        (BallBalanceControl_Abs(ball_accel_direction) > 0.001f)) {
        control->stop_distance_mm =
            control->velocity_mm_s * control->velocity_mm_s /
            (2.0f * BALL_BALANCE_ACCEL_PER_PIPE_DEG_MM_S2 *
             BallBalanceControl_Abs(pitch_deg));
    } else {
        control->stop_distance_mm = 0.0f;
    }
    return pitch_deg;
}

static void BallBalanceControl_ApplyTransportCommand(
    BallBalanceControl *control)
{
    float expected_disturbance_deg;
    float effective_disturbance_deg;
    float feedback_error_mm;
    float feedback_limit_deg;
    float kd_deg_per_mm_s;
    float p_limit_deg;
    float position_deadband_mm;
    float predicted_error_mm;

    expected_disturbance_deg = BallBalanceControl_Limit(
        control->transport_command_accel_cm_s2 *
        BALL_BALANCE_TRANSPORT_ACCEL_TO_DEG,
        -BALL_BALANCE_TRANSPORT_IMU_LIMIT_DEG,
        BALL_BALANCE_TRANSPORT_IMU_LIMIT_DEG);
    effective_disturbance_deg = expected_disturbance_deg;
    if (control->transport_imu_valid != 0U) {
        effective_disturbance_deg += BALL_BALANCE_TRANSPORT_IMU_BLEND *
            (BallBalanceControl_Limit(
                 control->transport_imu_disturbance_deg,
                 -BALL_BALANCE_TRANSPORT_IMU_LIMIT_DEG,
                 BALL_BALANCE_TRANSPORT_IMU_LIMIT_DEG) -
             expected_disturbance_deg);
    }
    control->transport_wall_feedforward_pitch_deg = 0.0f;
    if (control->transport_wall_valid != 0U) {
        control->transport_wall_feedforward_pitch_deg =
            BallBalanceControl_Limit(
                -control->transport_wall_accel_cm_s2 *
                    BALL_BALANCE_TRANSPORT_ACCEL_TO_DEG,
                -BALL_BALANCE_TRANSPORT_WALL_FF_LIMIT_DEG,
                BALL_BALANCE_TRANSPORT_WALL_FF_LIMIT_DEG);
    }
    control->transport_feedforward_pitch_deg = BallBalanceControl_Limit(
        -effective_disturbance_deg +
            control->transport_wall_feedforward_pitch_deg,
        -BALL_BALANCE_TRANSPORT_FF_LIMIT_DEG,
        BALL_BALANCE_TRANSPORT_FF_LIMIT_DEG);

    position_deadband_mm = BALL_BALANCE_TRANSPORT_TRANSIENT_DEADBAND_MM;
    if (BallBalanceControl_Abs(control->transport_command_accel_cm_s2) <=
        BALL_BALANCE_TRANSPORT_CRUISE_ACCEL_LIMIT_CM_S2) {
        position_deadband_mm = BALL_BALANCE_TRANSPORT_CRUISE_DEADBAND_MM;
    }
    control->transport_position_deadband_mm = position_deadband_mm;
    predicted_error_mm = control->predicted_position_mm - control->target_mm;
    if (control->transport_recovery_active != 0U) {
        if ((BallBalanceControl_Abs(control->error_mm) <=
             (BALL_BALANCE_TRANSPORT_RECOVERY_EXIT_MM +
              BALL_BALANCE_POSITION_COMPARE_EPSILON_MM)) &&
            (BallBalanceControl_Abs(control->velocity_mm_s) <=
             BALL_BALANCE_TRANSPORT_RECOVERY_EXIT_SPEED_MM_S)) {
            control->transport_recovery_active = 0U;
        }
    } else if ((BallBalanceControl_Abs(control->error_mm) >=
                BALL_BALANCE_TRANSPORT_RECOVERY_ENTER_MM) ||
               (BallBalanceControl_Abs(predicted_error_mm) >=
                BALL_BALANCE_TRANSPORT_RECOVERY_ENTER_MM)) {
        control->transport_recovery_active = 1U;
    }
    feedback_error_mm = control->error_mm;
    if (BallBalanceControl_Abs(feedback_error_mm) <=
        position_deadband_mm) {
        feedback_error_mm = 0.0f;
    } else if (feedback_error_mm > 0.0f) {
        feedback_error_mm -= position_deadband_mm;
    } else {
        feedback_error_mm += position_deadband_mm;
    }
    p_limit_deg = BALL_BALANCE_TRANSPORT_P_LIMIT_DEG;
    kd_deg_per_mm_s = BALL_BALANCE_TRANSPORT_KD_DEG_PER_MM_S;
    feedback_limit_deg = BALL_BALANCE_TRANSPORT_FB_LIMIT_DEG;
    if (control->transport_recovery_active != 0U) {
        control->p_term_deg =
            BALL_BALANCE_TRANSPORT_RECOVERY_KP_DEG_PER_MM *
            feedback_error_mm;
        if ((feedback_error_mm > 0.0f) &&
            (control->p_term_deg <
             BALL_BALANCE_TRANSPORT_RECOVERY_MIN_P_DEG)) {
            control->p_term_deg =
                BALL_BALANCE_TRANSPORT_RECOVERY_MIN_P_DEG;
        } else if ((feedback_error_mm < 0.0f) &&
                   (control->p_term_deg >
                    -BALL_BALANCE_TRANSPORT_RECOVERY_MIN_P_DEG)) {
            control->p_term_deg =
                -BALL_BALANCE_TRANSPORT_RECOVERY_MIN_P_DEG;
        }
        p_limit_deg = BALL_BALANCE_TRANSPORT_RECOVERY_P_LIMIT_DEG;
        kd_deg_per_mm_s =
            BALL_BALANCE_TRANSPORT_RECOVERY_KD_DEG_PER_MM_S;
        feedback_limit_deg =
            BALL_BALANCE_TRANSPORT_RECOVERY_FB_LIMIT_DEG;
    } else {
        control->p_term_deg =
            BALL_BALANCE_TRANSPORT_KP_DEG_PER_MM * feedback_error_mm;
    }
    control->p_term_deg = BallBalanceControl_Limit(
        control->p_term_deg, -p_limit_deg, p_limit_deg);
    control->d_term_deg = BallBalanceControl_Limit(
        kd_deg_per_mm_s * control->velocity_mm_s,
        (control->transport_recovery_active != 0U) ?
            -BALL_BALANCE_TRANSPORT_RECOVERY_D_LIMIT_DEG :
            -BALL_BALANCE_TRANSPORT_D_LIMIT_DEG,
        (control->transport_recovery_active != 0U) ?
            BALL_BALANCE_TRANSPORT_RECOVERY_D_LIMIT_DEG :
            BALL_BALANCE_TRANSPORT_D_LIMIT_DEG);
    control->transport_feedback_pitch_deg = BallBalanceControl_Limit(
        control->p_term_deg + control->d_term_deg,
        -feedback_limit_deg,
        feedback_limit_deg);
    control->i_term_deg = 0.0f;
    control->integral_mm_s = 0.0f;
    control->integral_active = 0U;
    control->brake_term_deg = control->d_term_deg;
    control->braking_active = (uint8_t) (
        (control->velocity_mm_s *
         (BALL_BALANCE_BALL_ACCEL_SIGN_PER_PITCH *
          control->transport_feedback_pitch_deg)) < 0.0f);
    control->phase_pitch_deg = BallBalanceControl_LimitActualPitch(
        control,
        control->transport_feedforward_pitch_deg +
        control->transport_feedback_pitch_deg);
    control->desired_pipe_pitch_deg = control->phase_pitch_deg;
    control->trajectory_phase = BALL_BALANCE_PHASE_MOVING;
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


static void BallBalanceControl_ApplyMovingCommand(
    BallBalanceControl *control)
{
    control->phase_pitch_deg =
        BallBalanceControl_CalculateMovingPitch(control);
    control->desired_pipe_pitch_deg = control->phase_pitch_deg;
}

static void BallBalanceControl_UpdateTrajectory(
    BallBalanceControl *control,
    uint32_t now_ms,
    uint8_t measurement_accepted)
{
    uint32_t kick_stage_elapsed_ms;
    float error_abs;
    float velocity_abs;
    int8_t current_moving_direction;
    uint8_t moving_away_from_target;

    control->error_mm = control->filtered_position_mm - control->target_mm;
    error_abs = BallBalanceControl_Abs(control->error_mm);
    velocity_abs = BallBalanceControl_Abs(control->velocity_mm_s);
    if (control->transport_mode_enabled != 0U) {
        BallBalanceControl_ApplyTransportCommand(control);
        return;
    }
    moving_away_from_target = (uint8_t) (
        (control->error_mm * control->velocity_mm_s) > 0.0f);
    kick_stage_elapsed_ms =
        (uint32_t) (now_ms - control->kick_stage_started_ms);

    control->zero_crossed = 0U;
    if (measurement_accepted != 0U) {
        if ((control->previous_error_ready != 0U) &&
            (((control->previous_error_mm > 0.0f) &&
              (control->error_mm <= 0.0f)) ||
             ((control->previous_error_mm < 0.0f) &&
              (control->error_mm >= 0.0f)))) {
            control->zero_crossed = 1U;
            control->kick_attempt_count = 0U;
            control->turnaround_freeze_active = 0U;
            control->outward_motion_seen = 0U;
            control->phase_pitch_deg = 0.0f;
            control->desired_pipe_pitch_deg = 0.0f;
        }
        control->previous_error_mm = control->error_mm;
        control->previous_error_ready = 1U;
    }

    BallBalanceControl_UpdateNearTargetIntegral(control,
                                                measurement_accepted,
                                                error_abs,
                                                velocity_abs);

    if (measurement_accepted != 0U) {

        if (velocity_abs <= BALL_BALANCE_BALL_STOPPED_SPEED_MM_S) {
            if (control->stationary_sample_count < 0xFFU) {
                control->stationary_sample_count++;
            }
        } else {
            control->stationary_sample_count = 0U;
        }
        current_moving_direction = 0;
        if (velocity_abs >= BALL_BALANCE_BALL_MOVING_SPEED_MM_S) {
            current_moving_direction =
                (control->velocity_mm_s > 0.0f) ? 1 : -1;
        }
        if (current_moving_direction == 0) {
            control->moving_sample_count = 0U;
            control->moving_direction = 0;
        } else if (current_moving_direction ==
                   control->moving_direction) {
            if (control->moving_sample_count < 0xFFU) {
                control->moving_sample_count++;
            }
        } else {
            control->moving_sample_count = 1U;
            control->moving_direction = current_moving_direction;
        }
    }

    switch ((BallBalancePhase) control->trajectory_phase) {
        case BALL_BALANCE_PHASE_WAIT_PIPE:
            control->p_term_deg = 0.0f;
            control->d_term_deg = 0.0f;
            control->brake_term_deg = 0.0f;
            control->braking_active = 0U;
            control->desired_pipe_pitch_deg = 0.0f;
            control->phase_pitch_deg = 0.0f;
            if ((control->pipe_settled == 0U) ||
                (measurement_accepted == 0U) ||
                (control->last_measurement_dt_s <= 0.0f)) {
                break;
            }
            if (control->moving_sample_count >=
                BALL_BALANCE_MOVING_SAMPLE_COUNT) {
                BallBalanceControl_EnterPhase(
                    control, BALL_BALANCE_PHASE_MOVING, now_ms, 0.0f);
                BallBalanceControl_ApplyMovingCommand(control);
            } else if (control->stationary_sample_count >=
                       BALL_BALANCE_STATIONARY_SAMPLE_COUNT) {
                if (error_abs <= BALL_BALANCE_HOLD_ENTER_MM) {
                    control->kick_attempt_count = 0U;
                    BallBalanceControl_EnterZeroHoldFreeze(control, now_ms);
                } else {
                    control->kick_attempt_count = 0U;
                    BallBalanceControl_EnterStationaryAcceleration(
                        control, now_ms);
                }
            }
            break;

        case BALL_BALANCE_PHASE_BREAKAWAY:
            control->p_term_deg = 0.0f;
            control->d_term_deg = 0.0f;
            control->brake_term_deg = 0.0f;
            control->braking_active = 0U;
            if (measurement_accepted == 0U) {
                control->desired_pipe_pitch_deg = control->phase_pitch_deg;
                break;
            }
            if ((control->zero_crossed != 0U) ||
                (control->moving_sample_count >=
                 BALL_BALANCE_MOVING_SAMPLE_COUNT)) {
                control->kick_attempt_count = 0U;
                BallBalanceControl_EnterPhase(
                    control, BALL_BALANCE_PHASE_MOVING, now_ms, 0.0f);
                BallBalanceControl_ApplyMovingCommand(control);
                break;
            }
            if ((error_abs <= BALL_BALANCE_HOLD_ENTER_MM) &&
                (control->stationary_sample_count >=
                 BALL_BALANCE_STATIONARY_SAMPLE_COUNT)) {
                control->kick_attempt_count = 0U;
                BallBalanceControl_EnterZeroHoldFreeze(control, now_ms);
                break;
            }
            if ((kick_stage_elapsed_ms >= BALL_BALANCE_ACCEL_STAGE_MS) &&
                (control->kick_attempt_count < 0xFFU)) {
                control->kick_attempt_count++;
                control->kick_stage_started_ms = now_ms;
            }
            /* Recompute sign on every accepted K230 frame.  A sign change can
             * never retain the previous breakaway direction. */
            control->phase_pitch_deg =
                BallBalanceControl_SelectAccelerationPitch(
                    control->error_mm, control->kick_attempt_count);
            control->phase_pitch_deg = BallBalanceControl_LimitActualPitch(
                control, control->phase_pitch_deg);
            control->desired_pipe_pitch_deg = control->phase_pitch_deg;
            break;

        case BALL_BALANCE_PHASE_MOVING:
            if (control->turnaround_freeze_active != 0U) {
                if ((control->zero_crossed != 0U) ||
                    (moving_away_from_target == 0U) ||
                    ((uint32_t) (now_ms -
                     control->turnaround_freeze_started_ms) >=
                     BALL_BALANCE_TURNAROUND_FREEZE_MS)) {
                    control->turnaround_freeze_active = 0U;
                    control->outward_motion_seen = 0U;
                } else {
                    control->desired_pipe_pitch_deg =
                        control->phase_pitch_deg;
                    control->pipe_command_rate_dps = 0.0f;
                    break;
                }
            }
            if ((measurement_accepted != 0U) &&
                (moving_away_from_target != 0U) &&
                (velocity_abs >
                 BALL_BALANCE_TURNAROUND_LEVEL_SPEED_MM_S)) {
                control->outward_motion_seen = 1U;
            }
            if ((measurement_accepted != 0U) &&
                (control->outward_motion_seen != 0U) &&
                (moving_away_from_target != 0U) &&
                (velocity_abs <=
                 BALL_BALANCE_TURNAROUND_LEVEL_SPEED_MM_S)) {
                control->turnaround_freeze_active = 1U;
                control->turnaround_freeze_started_ms = now_ms;
                control->phase_pitch_deg = control->command_pipe_pitch_deg;
                control->desired_pipe_pitch_deg =
                    control->command_pipe_pitch_deg;
                control->pipe_command_rate_dps = 0.0f;
                break;
            }
            if ((measurement_accepted != 0U) &&
                (control->stationary_sample_count >=
                 BALL_BALANCE_STATIONARY_SAMPLE_COUNT)) {
                if (error_abs <= BALL_BALANCE_HOLD_ENTER_MM) {
                    control->kick_attempt_count = 0U;
                    BallBalanceControl_EnterZeroHoldFreeze(control, now_ms);
                } else {
                    BallBalanceControl_EnterStationaryAcceleration(
                        control, now_ms);
                }
                break;
            }
            BallBalanceControl_ApplyMovingCommand(control);
            break;

        case BALL_BALANCE_PHASE_HOLD:
        default:
            control->p_term_deg = 0.0f;
            control->d_term_deg = 0.0f;
            control->brake_term_deg = 0.0f;
            control->braking_active = 0U;
            control->kick_attempt_count = 0U;
            if (control->zero_hold_freeze_active != 0U) {
                control->desired_pipe_pitch_deg = control->phase_pitch_deg;
                control->pipe_command_rate_dps = 0.0f;
                if ((uint32_t) (now_ms - control->phase_started_ms) >=
                    BALL_BALANCE_ZERO_HOLD_FREEZE_MS) {
                    control->zero_hold_freeze_active = 0U;
                    control->phase_pitch_deg = 0.0f;
                    control->desired_pipe_pitch_deg = 0.0f;
                    control->pipe_settled = 0U;
                    control->pipe_settle_timer_ready = 0U;
                    /* 冻结期结束立即建立接近目标的积分状态：
                     * 若球已在目标附近，下一次 UpdateTrajectory 会因
                     * 误差在4~20毫米区间而启用积分，弥补静摩擦保持力，
                     * 避免球在零角度下失稳漂走。 */
                    BallBalanceControl_UpdateNearTargetIntegral(
                        control, 1U, 0.0f, 0.0f);
                }
                break;
            }
            if (control->integral_active != 0U) {
                /* 目标点附近退出冻结后恢复PI/D微调，消除水平偏差和静摩擦残差。 */
                BallBalanceControl_ApplyMovingCommand(control);
            } else {
                control->phase_pitch_deg = 0.0f;
                control->desired_pipe_pitch_deg = 0.0f;
            }
            if (measurement_accepted == 0U) {
                break;
            }
            if ((control->moving_sample_count >=
                 BALL_BALANCE_MOVING_SAMPLE_COUNT) ||
                (control->zero_crossed != 0U)) {
                BallBalanceControl_EnterPhase(
                    control, BALL_BALANCE_PHASE_MOVING, now_ms, 0.0f);
                BallBalanceControl_ApplyMovingCommand(control);
            } else if ((error_abs >= BALL_BALANCE_HOLD_EXIT_MM) &&
                       (control->stationary_sample_count >=
                        BALL_BALANCE_STATIONARY_SAMPLE_COUNT)) {
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
    float spike_jump_limit_mm;
    uint32_t spike_stuck_ms;
    uint8_t filter_was_ready;
    uint8_t accepted;

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
    spike_jump_limit_mm = (control->transport_mode_enabled != 0U) ?
        BALL_BALANCE_TRANSPORT_SPIKE_JUMP_MM :
        BALL_BALANCE_VISION_SPIKE_JUMP_MM;
    spike_stuck_ms = (control->transport_mode_enabled != 0U) ?
        BALL_BALANCE_TRANSPORT_SPIKE_STUCK_MS :
        BALL_BALANCE_SPIKE_STUCK_MS;
    accepted = 1U;
    if ((filter_was_ready == 0U) ||
        (elapsed_ms > BallBalanceControl_DataTimeoutMs(control))) {
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
        /* 跳变抑制：停车平衡仍使用严格阈值；AB/ABCD 运输模式考虑 K230
         * 只有约 5~6 FPS，允许球在相邻样本间产生更大的真实位移。
         * 一帧位置跳变超过当前模式阈值时才视为检测错误。
         * 不更新滤波位置与速度，只让测量超时保护去接管，避免
         * 假速度触发 P/D 反打把钢球推向端部。
         * 但跳变持续超过 SPIKE_STUCK_MS 仍偏离，判定为真实位移
         * 或持续错检，直接重采样新位置，避免滤波值永久冻结。 */
        if (BallBalanceControl_Abs(position_mm - previous_position) <=
            spike_jump_limit_mm) {
            control->spike_reject_active = 0U;
            control->last_spike_reject_ms = 0U;
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
        } else if ((control->spike_reject_active != 0U) &&
                   ((uint32_t) (now_ms - control->last_spike_reject_ms) >=
                    spike_stuck_ms)) {
            /* 持续跳变：重采样为新位置，速度清零，避免卡死旧值。 */
            control->spike_reject_active = 0U;
            control->last_spike_reject_ms = 0U;
            control->filtered_position_mm = position_mm;
            control->velocity_mm_s = 0.0f;
            control->integral_mm_s = 0.0f;
            control->filter_ready = 1U;
            measurement_dt_s = 0.0f;
        } else {
            /* 单帧/短时跳变：不当作有效测量，避免干扰采样计数与积分。 */
            if (control->spike_reject_active == 0U) {
                control->spike_reject_active = 1U;
                control->last_spike_reject_ms = now_ms;
            }
            measurement_dt_s = 0.0f;
            accepted = 0U;
        }
    }

    /* 始终推进 last_valid_ms：保证主循环超时检测不把跳变抑制误判为
     * 视觉失联而清除状态。跳变持续判定由独立的 SPIKE_STUCK_MS 计时负责。 */
    control->last_valid_ms = now_ms;
    control->last_measurement_dt_s = measurement_dt_s;
    control->tracking_valid = 1U;
    control->timed_out = 0U;
    return accepted;
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

static uint8_t BallBalanceControl_AttitudeUnsafeDebounce(
    BallBalanceControl *control,
    uint8_t attitude_safe,
    uint32_t now_ms)
{
    /* 姿态瞬时超限或短暂不新鲜时先让内环自行纠正，持续超过
     * BALL_BALANCE_IMU_FAULT_DEBOUNCE_MS 才认定 IMU 故障，避免起步
     * 与变速造成的惯性摆动被误判为传感器故障。 */
    if (attitude_safe == 0U) {
        if (control->attitude_unsafe_since_ms == 0U) {
            control->attitude_unsafe_since_ms = now_ms;
        }
        return (uint8_t) ((uint32_t) (now_ms -
                                      control->attitude_unsafe_since_ms) >=
                          BALL_BALANCE_IMU_FAULT_DEBOUNCE_MS);
    }
    control->attitude_unsafe_since_ms = 0U;
    return 0U;
}

static void BallBalanceControl_UpdateEnableState(BallBalanceControl *control,
                                                 uint32_t now_ms)
{
    uint8_t attitude_fresh;
    uint8_t attitude_safe;
    uint8_t attitude_stable;

    if (control->enabled_requested == 0U) {
        control->enabled = 0U;
        control->attitude_stable_ready = 0U;
        return;
    }

    if (control->manual_pipe_mode_enabled != 0U) {
        /* 开环水管角模式：不依赖 IMU 姿态与标定映射，直接启用，
         * 输出仅由限速斜坡和角度限幅决定。 */
        if (control->enabled == 0U) {
            control->enabled = 1U;
            control->enabled_since_ms = now_ms;
            control->preserve_vision_on_enable = 0U;
            BallBalanceControl_ClearVisionState(control, 1U);
        }
        control->imu_fault = 0U;
        control->attitude_stable_ready = 1U;
        return;
    }

    if (control->mapping_valid == 0U) {
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
        if (BallBalanceControl_AttitudeUnsafeDebounce(control,
                                                      attitude_safe,
                                                      now_ms) != 0U) {
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
        if ((control->preserve_vision_on_enable != 0U) &&
            (control->tracking_valid != 0U) &&
            (control->filter_ready != 0U)) {
            BallBalanceControl_ResetControlStatePreserveVision(control);
        } else {
            BallBalanceControl_ClearVisionState(control, 1U);
        }
        control->preserve_vision_on_enable = 0U;
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
    float maximum_delta;

    if (control->manual_pipe_mode_enabled != 0U) {
        /* 开环水管角模式：目标角直接前馈到舵机，不叠加 IMU 姿态修正。 */
        control->pipe_pitch_error_deg = 0.0f;
        control->pipe_p_term_deg = 0.0f;
        control->pipe_d_term_deg = 0.0f;
        corrected_pipe_command = control->command_pipe_pitch_deg;
    } else if ((control->enabled != 0U) &&
               (BallBalanceControl_AttitudeIsFresh(control, now_ms) != 0U)) {
        control->pipe_pitch_error_deg = control->command_pipe_pitch_deg -
                                        control->measured_pipe_pitch_deg;
        control->pipe_p_term_deg = control->pipe_kp *
                                   control->pipe_pitch_error_deg;
        /* 直接使用120 Hz陀螺仪X轴角速度反向修正，抑制水管快速摆动。 */
        control->pipe_d_term_deg = -control->pipe_kd_s *
                                   control->measured_pipe_rate_dps;
        corrected_pipe_command = BallBalanceControl_LimitActualPitch(
            control,
            control->command_pipe_pitch_deg +
            control->pipe_p_term_deg +
            control->pipe_d_term_deg);
    } else {
        control->pipe_pitch_error_deg = 0.0f;
        control->pipe_p_term_deg = 0.0f;
        control->pipe_d_term_deg = 0.0f;
    }

    if ((control->manual_pipe_mode_enabled != 0U) ||
        (control->mapping_valid != 0U)) {
        /* 线性PWM换算由BSP执行；控制层输出经过IMU内环修正的相对水管角。 */
        control->feedforward_model_pitch_deg =
            (control->enabled != 0U) ?
            control->command_pipe_pitch_deg : 0.0f;
        control->desired_model_pitch_deg =
            (control->enabled != 0U) ? corrected_pipe_command : 0.0f;
        control->servo_feedforward_deg =
            control->feedforward_model_pitch_deg;
        control->desired_angle_deg = control->desired_model_pitch_deg;
    } else {
        control->feedforward_model_pitch_deg = 0.0f;
        control->desired_model_pitch_deg = 0.0f;
        control->servo_feedforward_deg = 0.0f;
        control->desired_angle_deg = 0.0f;
    }

    control->desired_angle_deg = BallBalanceControl_LimitActualPitch(
        control, control->desired_angle_deg);
    if (task_dt_s > 0.0f) {
        maximum_delta = BALL_BALANCE_ACTUATOR_SLEW_DPS * task_dt_s;
        control->command_angle_deg = BallBalanceControl_Approach(
            control->command_angle_deg,
            control->desired_angle_deg,
            maximum_delta);
    }
    control->command_angle_deg = BallBalanceControl_LimitActualPitch(
        control, control->command_angle_deg);
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
    output->zero_crossed = control->zero_crossed;
    output->turnaround_freeze_active =
        control->turnaround_freeze_active;
    output->pipe_settled = control->pipe_settled;
    output->moving_sample_count = control->moving_sample_count;
    output->moving_direction = control->moving_direction;
    output->kick_attempt_count = control->kick_attempt_count;
    output->trajectory_phase = control->trajectory_phase;
    output->endpoint_hold_enabled = control->endpoint_hold_enabled;
    output->transport_mode_enabled = control->transport_mode_enabled;
    output->manual_pipe_mode_enabled = control->manual_pipe_mode_enabled;
    output->target_mm = control->target_mm;
    output->manual_pipe_target_deg = control->manual_pipe_target_deg;
    output->position_kp_deg_per_mm = control->position_kp_deg_per_mm;
    output->position_ki_deg_per_mm_s =
        control->position_ki_deg_per_mm_s;
    output->position_kd_deg_per_mm_s =
        control->position_kd_deg_per_mm_s;
    output->pipe_kp = control->pipe_kp;
    output->pipe_kd_s = control->pipe_kd_s;
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
    output->transport_command_accel_cm_s2 =
        control->transport_command_accel_cm_s2;
    output->transport_position_deadband_mm =
        control->transport_position_deadband_mm;
    output->transport_recovery_active =
        control->transport_recovery_active;
    output->transport_imu_disturbance_deg =
        control->transport_imu_disturbance_deg;
    output->transport_wall_accel_cm_s2 =
        control->transport_wall_accel_cm_s2;
    output->transport_wall_feedforward_pitch_deg =
        control->transport_wall_feedforward_pitch_deg;
    output->transport_feedforward_pitch_deg =
        control->transport_feedforward_pitch_deg;
    output->transport_feedback_pitch_deg =
        control->transport_feedback_pitch_deg;
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
    if (control == 0) {
        return;
    }

    memset(control, 0, sizeof(*control));
    control->position_kp_deg_per_mm =
        BALL_BALANCE_DEFAULT_KP_DEG_PER_MM;
    control->position_ki_deg_per_mm_s =
        BALL_BALANCE_DEFAULT_KI_DEG_PER_MM_S;
    control->position_kd_deg_per_mm_s =
        BALL_BALANCE_DEFAULT_KD_DEG_PER_MM_S;
    control->pipe_kp = BALL_BALANCE_DEFAULT_PIPE_KP;
    control->pipe_kd_s = BALL_BALANCE_DEFAULT_PIPE_KD_S;
    BallBalanceControl_FillFixedMapping(control);
}

static void BallBalanceControl_SetEnabledInternal(
    BallBalanceControl *control,
    uint8_t enabled,
    uint32_t now_ms,
    uint8_t preserve_vision)
{
    uint8_t normalized_enabled;
    uint8_t preserve_active;

    if (control == 0) {
        return;
    }

    normalized_enabled = (enabled != 0U) ? 1U : 0U;
    if ((normalized_enabled != 0U) &&
        (control->manual_pipe_mode_enabled == 0U) &&
        (control->mapping_valid == 0U)) {
        return;
    }
    preserve_active = (uint8_t) (
        (normalized_enabled != 0U) &&
        (preserve_vision != 0U) &&
        (control->tracking_valid != 0U) &&
        (control->filter_ready != 0U));
    if (normalized_enabled == 0U) {
        control->endpoint_hold_enabled = 0U;
        control->transport_mode_enabled = 0U;
        control->manual_pipe_mode_enabled = 0U;
        control->manual_pipe_target_deg = 0.0f;
        control->transport_imu_valid = 0U;
        control->transport_wall_valid = 0U;
        control->transport_command_accel_cm_s2 = 0.0f;
        control->transport_imu_disturbance_deg = 0.0f;
        control->transport_wall_accel_cm_s2 = 0.0f;
        control->transport_wall_feedforward_pitch_deg = 0.0f;
        control->transport_feedforward_pitch_deg = 0.0f;
        control->transport_feedback_pitch_deg = 0.0f;
        control->integral_mm_s = 0.0f;
        control->i_term_deg = 0.0f;
        control->integral_active = 0U;
        control->preserve_vision_on_enable = 0U;
    }
    if (control->enabled_requested == normalized_enabled) {
        if ((normalized_enabled != 0U) && (control->enabled == 0U)) {
            control->preserve_vision_on_enable = preserve_active;
        } else if ((normalized_enabled != 0U) &&
                   (preserve_active != 0U)) {
            /* 已处于停车平衡时，第六题仍需清除旧控制状态并保留观测。 */
            BallBalanceControl_ResetControlStatePreserveVision(control);
        }
        return;
    }

    control->enabled_requested = normalized_enabled;
    control->enabled = 0U;
    control->imu_fault = 0U;
    control->attitude_stable_ready = 0U;
    control->enabled_since_ms = now_ms;
    control->last_task_ms = now_ms;
    control->task_time_ready = 1U;
    control->preserve_vision_on_enable = preserve_active;
    if (preserve_active != 0U) {
        BallBalanceControl_ResetControlStatePreserveVision(control);
    } else {
        BallBalanceControl_ClearVisionState(control, 1U);
    }
}

void BallBalanceControl_SetEnabled(BallBalanceControl *control,
                                   uint8_t enabled,
                                   uint32_t now_ms)
{
    BallBalanceControl_SetEnabledInternal(control, enabled, now_ms, 0U);
}

void BallBalanceControl_SetEnabledPreserveVision(
    BallBalanceControl *control,
    uint8_t enabled,
    uint32_t now_ms)
{
    BallBalanceControl_SetEnabledInternal(control, enabled, now_ms, 1U);
}

uint8_t BallBalanceControl_IsEnabled(const BallBalanceControl *control)
{
    return (control != 0) ? control->enabled_requested : 0U;
}

uint8_t BallBalanceControl_IsActive(const BallBalanceControl *control)
{
    return (control != 0) ? control->enabled : 0U;
}

uint8_t BallBalanceControl_GetFreshObservedPosition(
    const BallBalanceControl *control,
    uint32_t now_ms,
    uint32_t maximum_age_ms,
    float *position_mm)
{
    if ((control == 0) || (position_mm == 0) ||
        (control->tracking_valid == 0U) ||
        (control->filter_ready == 0U) ||
        (control->timed_out != 0U) ||
        ((uint32_t) (now_ms - control->last_valid_ms) > maximum_age_ms)) {
        return 0U;
    }

    *position_mm = control->filtered_position_mm;
    return 1U;
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
        control->moving_sample_count = 0U;
        control->moving_direction = 0;
        control->previous_error_ready = 0U;
        control->zero_crossed = 0U;
        control->turnaround_freeze_active = 0U;
        control->outward_motion_seen = 0U;
    }
}

void BallBalanceControl_SetEndpointHoldEnabled(BallBalanceControl *control,
                                               uint8_t enabled)
{
    uint8_t normalized_enabled;

    if (control == 0) {
        return;
    }

    normalized_enabled = (enabled != 0U) ? 1U : 0U;
    if (normalized_enabled == control->endpoint_hold_enabled) {
        return;
    }

    control->endpoint_hold_enabled = normalized_enabled;
    control->integral_mm_s = 0.0f;
    control->i_term_deg = 0.0f;
    control->integral_active = 0U;
    if (normalized_enabled == 0U) {
        control->phase_pitch_deg = 0.0f;
        control->desired_pipe_pitch_deg = 0.0f;
    }
}

void BallBalanceControl_SetTransportMode(BallBalanceControl *control,
                                         uint8_t enabled)
{
    uint8_t normalized_enabled;

    if (control == 0) {
        return;
    }

    normalized_enabled = (enabled != 0U) ? 1U : 0U;
    if (normalized_enabled == control->transport_mode_enabled) {
        return;
    }

    control->transport_mode_enabled = normalized_enabled;
    control->transport_recovery_active = 0U;
    /* 模式切换时丢弃停车阶段尚未确认的跳变候选，避免旧的 400 ms
     * 计时在刚起步时把第一帧真实位移错误地重采样为零速度。 */
    control->spike_reject_active = 0U;
    control->last_spike_reject_ms = 0U;
    if (normalized_enabled != 0U) {
        control->manual_pipe_mode_enabled = 0U;
        control->manual_pipe_target_deg = 0.0f;
    }
    control->transport_imu_valid = 0U;
    control->transport_wall_valid = 0U;
    control->transport_command_accel_cm_s2 = 0.0f;
    control->transport_imu_disturbance_deg = 0.0f;
    control->transport_wall_accel_cm_s2 = 0.0f;
    control->transport_wall_feedforward_pitch_deg = 0.0f;
    control->transport_feedforward_pitch_deg = 0.0f;
    control->transport_feedback_pitch_deg = 0.0f;
    control->endpoint_hold_enabled = 0U;
    control->integral_mm_s = 0.0f;
    control->i_term_deg = 0.0f;
    control->integral_active = 0U;
    control->trajectory_phase = BALL_BALANCE_PHASE_WAIT_PIPE;
    control->phase_pitch_deg = 0.0f;
    control->desired_pipe_pitch_deg = 0.0f;
    control->pipe_command_rate_dps = 0.0f;
}

void BallBalanceControl_SetManualPipeMode(BallBalanceControl *control,
                                         uint8_t enabled,
                                         float target_pipe_pitch_deg)
{
    uint8_t normalized_enabled;

    if (control == 0) {
        return;
    }

    normalized_enabled = (enabled != 0U) ? 1U : 0U;
    if (normalized_enabled == 0U) {
        control->manual_pipe_mode_enabled = 0U;
        control->manual_pipe_target_deg = 0.0f;
        control->desired_pipe_pitch_deg = 0.0f;
        control->command_pipe_pitch_deg = 0.0f;
        control->pipe_command_rate_dps = 0.0f;
        control->desired_angle_deg = 0.0f;
        control->command_angle_deg = 0.0f;
        control->trajectory_phase = BALL_BALANCE_PHASE_WAIT_PIPE;
        return;
    }

    control->manual_pipe_target_deg = BallBalanceControl_LimitActualPitch(
        control, target_pipe_pitch_deg);
    if (control->manual_pipe_mode_enabled == 0U) {
        control->transport_mode_enabled = 0U;
        control->endpoint_hold_enabled = 0U;
        control->integral_mm_s = 0.0f;
        control->p_term_deg = 0.0f;
        control->i_term_deg = 0.0f;
        control->d_term_deg = 0.0f;
        control->brake_term_deg = 0.0f;
        control->integral_active = 0U;
        control->braking_active = 0U;
        BallBalanceControl_ClearVisionState(control, 1U);
    }
    control->manual_pipe_mode_enabled = 1U;
    control->desired_pipe_pitch_deg = control->manual_pipe_target_deg;
    control->trajectory_phase = BALL_BALANCE_PHASE_MANUAL_PIPE;
}

void BallBalanceControl_SetTransportInputs(
    BallBalanceControl *control,
    float command_accel_cm_s2,
    float imu_disturbance_pitch_deg,
    uint8_t imu_valid)
{
    if (control == 0) {
        return;
    }

    control->transport_command_accel_cm_s2 = BallBalanceControl_Limit(
        command_accel_cm_s2, -100.0f, 100.0f);
    control->transport_imu_disturbance_deg = BallBalanceControl_Limit(
        imu_disturbance_pitch_deg,
        -BALL_BALANCE_TRANSPORT_IMU_LIMIT_DEG,
        BALL_BALANCE_TRANSPORT_IMU_LIMIT_DEG);
    control->transport_imu_valid = (imu_valid != 0U) ? 1U : 0U;
}

void BallBalanceControl_SetTransportWallAcceleration(
    BallBalanceControl *control,
    float wall_accel_cm_s2,
    uint8_t wall_valid)
{
    if (control == 0) {
        return;
    }

    control->transport_wall_accel_cm_s2 = BallBalanceControl_Limit(
        wall_accel_cm_s2,
        -BALL_BALANCE_TRANSPORT_WALL_ACCEL_LIMIT_CM_S2,
        BALL_BALANCE_TRANSPORT_WALL_ACCEL_LIMIT_CM_S2);
    control->transport_wall_valid = (wall_valid != 0U) ? 1U : 0U;
    if (control->transport_wall_valid == 0U) {
        control->transport_wall_accel_cm_s2 = 0.0f;
        control->transport_wall_feedforward_pitch_deg = 0.0f;
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
        if ((ki_deg_per_mm_s <= BALL_BALANCE_GAIN_EPSILON) &&
            (control->endpoint_hold_enabled == 0U)) {
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

void BallBalanceControl_SetPipeGains(BallBalanceControl *control,
                                     float pipe_kp,
                                     float pipe_kd_s)
{
    if (control == 0) {
        return;
    }
    if ((pipe_kp >= 0.0f) && (pipe_kp <= BALL_BALANCE_PIPE_KP_MAX)) {
        control->pipe_kp = pipe_kp;
    }
    if ((pipe_kd_s >= 0.0f) && (pipe_kd_s <= BALL_BALANCE_PIPE_KD_MAX_S)) {
        control->pipe_kd_s = pipe_kd_s;
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
    (void) model_scale;
    (void) model_offset_deg;
    (void) mapping_pitch_deg;

    if (control == 0) {
        return;
    }
    if (valid == 0U) {
        if (control->mapping_valid != 0U) {
            BallBalanceControl_InvalidateMapping(control);
        }
        return;
    }

    BallBalanceControl_FillFixedMapping(control);
    control->mapping_valid = 1U;
    control->mapping_fault = 0U;
    control->feedforward_cache_valid = 1U;
    control->desired_cache_valid = 1U;
}

float BallBalanceControl_GetNeutralAngleDeg(void)
{
    /* 兼容旧接口：线性PWM映射下相对水管水平角就是0°。 */
    return 0.0f;
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

    if (control->manual_pipe_mode_enabled == 0U) {
        /* 平衡关闭时也被动维护同一套位置滤波，供第六题捕获目标；
         * 该分支不会进入轨迹控制，也不会产生舵机输出。 */
        measurement_accepted = BallBalanceControl_AcceptMeasurement(
            control, now_ms, measurement);
    }

    if (control->enabled != 0U) {
        if (control->manual_pipe_mode_enabled != 0U) {
            control->tracking_valid = 0U;
            control->timed_out = 0U;
            control->error_mm = 0.0f;
            control->p_term_deg = 0.0f;
            control->i_term_deg = 0.0f;
            control->d_term_deg = 0.0f;
            control->brake_term_deg = 0.0f;
            control->desired_pipe_pitch_deg =
                control->manual_pipe_target_deg;
            control->trajectory_phase = BALL_BALANCE_PHASE_MANUAL_PIPE;
        } else if (control->tracking_valid != 0U) {
            elapsed_ms = (uint32_t) (now_ms - control->last_valid_ms);
            if (elapsed_ms > BallBalanceControl_DataTimeoutMs(control)) {
                BallBalanceControl_ClearVisionState(control, 0U);
                control->timed_out = 1U;
            } else {
                BallBalanceControl_UpdatePrediction(control, now_ms);
                BallBalanceControl_UpdateTrajectory(
                    control, now_ms, measurement_accepted);
            }
        } else if ((uint32_t) (now_ms - control->enabled_since_ms) >
                   BALL_BALANCE_IDLE_DATA_TIMEOUT_MS) {
            control->timed_out = 1U;
            control->desired_pipe_pitch_deg = 0.0f;
        }
    } else {
        if ((control->manual_pipe_mode_enabled == 0U) &&
            (control->tracking_valid != 0U)) {
            elapsed_ms = (uint32_t) (now_ms - control->last_valid_ms);
            if (elapsed_ms > BallBalanceControl_DataTimeoutMs(control)) {
                BallBalanceControl_ClearVisionState(control, 0U);
                control->timed_out = 1U;
            } else {
                BallBalanceControl_UpdatePrediction(control, now_ms);
            }
        }
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
