#include "ball_balance_control.h"

#include <string.h>

#define BALL_BALANCE_NEUTRAL_ANGLE_DEG          90.0f  /* 水管水平时的舵机中位角。 */
#define BALL_BALANCE_MIN_ANGLE_DEG              85.0f  /* 未标定机械结构时的最小安全舵机角。 */
#define BALL_BALANCE_MAX_ANGLE_DEG              95.0f  /* 未标定机械结构时的最大安全舵机角。 */
#define BALL_BALANCE_MAX_PIPE_PITCH_DEG          3.0f  /* 钢球位置外环允许的最大目标水管俯仰角。 */
#define BALL_BALANCE_PIPE_SAFETY_LIMIT_DEG       6.0f  /* 实测俯仰超过该角度时暂停姿态闭环。 */
#define BALL_BALANCE_POSITION_KP_DEG_PER_MM       0.040f /* 钢球位置到目标水管角的比例增益。 */
#define BALL_BALANCE_POSITION_KI_DEG_PER_MM_S     0.000f /* 钢球位置积分增益，首版保持为 0。 */
#define BALL_BALANCE_POSITION_KD_DEG_PER_MM_S     0.012f /* 钢球速度到目标水管角的阻尼增益。 */
#define BALL_BALANCE_INTEGRAL_LIMIT_MM_S        200.0f  /* 钢球位置误差积分绝对上限。 */
#define BALL_BALANCE_PIPE_KP                       0.60f  /* 水管姿态角误差的比例修正系数。 */
#define BALL_BALANCE_PIPE_KD_S                     0.050f /* 水管俯仰角速度阻尼系数，单位为秒。 */
#define BALL_BALANCE_MIN_QUALITY                  50U    /* 接受 K230 测量所需的最低质量。 */
#define BALL_BALANCE_DATA_TIMEOUT_MS            200U    /* 无合格视觉数据后目标回水平的超时时间。 */
#define BALL_BALANCE_ATTITUDE_TIMEOUT_MS        100U    /* IMU 姿态数据超时时间。 */
#define BALL_BALANCE_ATTITUDE_STABLE_MS         500U    /* 启用或故障恢复所需的连续稳定时间。 */
#define BALL_BALANCE_ENABLE_PITCH_LIMIT_DEG       1.0f  /* 启用前允许的最大水平角误差。 */
#define BALL_BALANCE_ENABLE_RATE_LIMIT_DPS        3.0f  /* 启用前允许的最大俯仰角速度。 */
#define BALL_BALANCE_TARGET_LIMIT_MM            100.0f  /* 可设置钢球目标位置的绝对上限。 */
#define BALL_BALANCE_POSITION_LIMIT_MM          130.0f  /* 合法钢球测量位置的绝对上限。 */
#define BALL_BALANCE_PIPE_SLEW_DEG_S             12.0f  /* 目标水管俯仰角最大变化率。 */
#define BALL_BALANCE_SERVO_SLEW_DEG_S            30.0f  /* 舵机命令最大变化率。 */
#define BALL_BALANCE_DT_MIN_S                     0.010f /* 视觉测量 dt 的最小值。 */
#define BALL_BALANCE_DT_MAX_S                     0.100f /* 视觉测量和任务 dt 的最大值。 */
#define BALL_BALANCE_POSITION_FILTER_TAU_S        0.050f /* 钢球位置低通滤波时间常数。 */
#define BALL_BALANCE_VELOCITY_FILTER_TAU_S        0.080f /* 钢球速度低通滤波时间常数。 */
#define BALL_BALANCE_MIN_SERVO_PER_PIPE           0.20f  /* 有效舵机/水管角比例绝对值下限。 */
#define BALL_BALANCE_MAX_SERVO_PER_PIPE          10.0f   /* 有效舵机/水管角比例绝对值上限。 */

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

static void BallBalanceControl_ClearVisionState(BallBalanceControl *control,
                                                uint8_t clear_sequence)
{
    control->tracking_valid = 0U;
    control->timed_out = 0U;
    control->filter_ready = 0U;
    if (clear_sequence != 0U) {
        control->sequence_ready = 0U;
        control->last_sequence = 0U;
    }
    control->last_valid_ms = 0U;
    control->filtered_position_mm = 0.0f;
    control->velocity_mm_s = 0.0f;
    control->integral_mm_s = 0.0f;
    control->error_mm = 0.0f;
    control->p_term_deg = 0.0f;
    control->i_term_deg = 0.0f;
    control->d_term_deg = 0.0f;
    control->desired_pipe_pitch_deg = 0.0f;
}

static void BallBalanceControl_CalculateOuterTarget(
    BallBalanceControl *control,
    float measurement_dt_s,
    uint8_t allow_integral)
{
    float candidate_integral = control->integral_mm_s;
    float candidate_i_term;
    float candidate_pitch;
    float integral_direction;

    /* 正位置表示钢球偏向右侧，右端抬高的正俯仰可使钢球滚回左侧。 */
    control->error_mm = control->filtered_position_mm - control->target_mm;
    control->p_term_deg = BALL_BALANCE_POSITION_KP_DEG_PER_MM *
                          control->error_mm;
    control->d_term_deg = BALL_BALANCE_POSITION_KD_DEG_PER_MM_S *
                          control->velocity_mm_s;

    if (allow_integral != 0U) {
        candidate_integral = BallBalanceControl_Limit(
            control->integral_mm_s + control->error_mm * measurement_dt_s,
            -BALL_BALANCE_INTEGRAL_LIMIT_MM_S,
            BALL_BALANCE_INTEGRAL_LIMIT_MM_S);
    }

    candidate_i_term = BALL_BALANCE_POSITION_KI_DEG_PER_MM_S *
                       candidate_integral;
    candidate_pitch = control->p_term_deg + candidate_i_term +
                      control->d_term_deg;
    integral_direction = BALL_BALANCE_POSITION_KI_DEG_PER_MM_S *
                         control->error_mm;
    if ((allow_integral != 0U) &&
        (((candidate_pitch >= -BALL_BALANCE_MAX_PIPE_PITCH_DEG) &&
          (candidate_pitch <= BALL_BALANCE_MAX_PIPE_PITCH_DEG)) ||
         ((candidate_pitch > BALL_BALANCE_MAX_PIPE_PITCH_DEG) &&
          (integral_direction < 0.0f)) ||
         ((candidate_pitch < -BALL_BALANCE_MAX_PIPE_PITCH_DEG) &&
          (integral_direction > 0.0f)))) {
        control->integral_mm_s = candidate_integral;
    }

    control->i_term_deg = BALL_BALANCE_POSITION_KI_DEG_PER_MM_S *
                          control->integral_mm_s;
    control->desired_pipe_pitch_deg = BallBalanceControl_Limit(
        control->p_term_deg + control->i_term_deg + control->d_term_deg,
        -BALL_BALANCE_MAX_PIPE_PITCH_DEG,
        BALL_BALANCE_MAX_PIPE_PITCH_DEG);
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
            BALL_BALANCE_DT_MAX_S);
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
    control->tracking_valid = 1U;
    control->timed_out = 0U;
    BallBalanceControl_CalculateOuterTarget(control,
                                            measurement_dt_s,
                                            filter_was_ready);
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
    }
}

static void BallBalanceControl_UpdateServoCommand(BallBalanceControl *control,
                                                  float task_dt_s,
                                                  uint32_t now_ms)
{
    float corrected_pipe_command;
    float maximum_delta;
    uint8_t attitude_fresh;

    attitude_fresh = BallBalanceControl_AttitudeIsFresh(control, now_ms);
    if ((control->enabled != 0U) && (attitude_fresh != 0U)) {
        control->pipe_pitch_error_deg = control->command_pipe_pitch_deg -
                                        control->measured_pipe_pitch_deg;
        control->pipe_p_term_deg = BALL_BALANCE_PIPE_KP *
                                   control->pipe_pitch_error_deg;
        control->pipe_d_term_deg = -BALL_BALANCE_PIPE_KD_S *
                                   control->measured_pipe_rate_dps;
        corrected_pipe_command = BallBalanceControl_Limit(
            control->command_pipe_pitch_deg + control->pipe_p_term_deg +
            control->pipe_d_term_deg,
            -BALL_BALANCE_MAX_PIPE_PITCH_DEG,
            BALL_BALANCE_MAX_PIPE_PITCH_DEG);
        control->servo_feedforward_deg = BALL_BALANCE_NEUTRAL_ANGLE_DEG +
            control->servo_deg_per_pipe_deg *
            control->command_pipe_pitch_deg;
        control->desired_angle_deg = BALL_BALANCE_NEUTRAL_ANGLE_DEG +
            control->servo_deg_per_pipe_deg * corrected_pipe_command;
    } else {
        control->pipe_pitch_error_deg = 0.0f;
        control->pipe_p_term_deg = 0.0f;
        control->pipe_d_term_deg = 0.0f;
        control->servo_feedforward_deg = BALL_BALANCE_NEUTRAL_ANGLE_DEG;
        control->desired_angle_deg = BALL_BALANCE_NEUTRAL_ANGLE_DEG;
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
    output->target_mm = control->target_mm;
    output->filtered_position_mm = control->filtered_position_mm;
    output->velocity_mm_s = control->velocity_mm_s;
    output->error_mm = control->error_mm;
    output->p_term_deg = control->p_term_deg;
    output->i_term_deg = control->i_term_deg;
    output->d_term_deg = control->d_term_deg;
    output->desired_pipe_pitch_deg = control->desired_pipe_pitch_deg;
    output->command_pipe_pitch_deg = control->command_pipe_pitch_deg;
    output->measured_pipe_pitch_deg = control->measured_pipe_pitch_deg;
    output->measured_pipe_rate_dps = control->measured_pipe_rate_dps;
    output->pipe_pitch_error_deg = control->pipe_pitch_error_deg;
    output->pipe_p_term_deg = control->pipe_p_term_deg;
    output->pipe_d_term_deg = control->pipe_d_term_deg;
    output->servo_deg_per_pipe_deg = control->servo_deg_per_pipe_deg;
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
    control->desired_angle_deg = BALL_BALANCE_NEUTRAL_ANGLE_DEG;
    control->command_angle_deg = BALL_BALANCE_NEUTRAL_ANGLE_DEG;
    control->servo_feedforward_deg = BALL_BALANCE_NEUTRAL_ANGLE_DEG;
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
    }
}

void BallBalanceControl_SetPipeCalibration(BallBalanceControl *control,
                                           uint8_t valid,
                                           float servo_deg_per_pipe_deg)
{
    uint8_t mapping_valid;

    if (control == 0) {
        return;
    }

    mapping_valid =
        (uint8_t) ((valid != 0U) &&
                   (BallBalanceControl_Abs(servo_deg_per_pipe_deg) >=
                    BALL_BALANCE_MIN_SERVO_PER_PIPE) &&
                   (BallBalanceControl_Abs(servo_deg_per_pipe_deg) <=
                    BALL_BALANCE_MAX_SERVO_PER_PIPE));
    control->mapping_valid = mapping_valid;
    if (mapping_valid != 0U) {
        control->servo_deg_per_pipe_deg = servo_deg_per_pipe_deg;
    } else {
        control->servo_deg_per_pipe_deg = 0.0f;
        BallBalanceControl_SetEnabled(control, 0U, control->last_task_ms);
    }
}

float BallBalanceControl_GetNeutralAngleDeg(void)
{
    return BALL_BALANCE_NEUTRAL_ANGLE_DEG;
}

void BallBalanceControl_Update(BallBalanceControl *control,
                               uint32_t now_ms,
                               const BallBalanceMeasurement *measurement,
                               const BallBalanceAttitude *attitude,
                               BallBalanceOutput *output)
{
    uint32_t elapsed_ms;
    float task_dt_s = 0.0f;
    float maximum_delta;
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

    if (control->enabled != 0U) {
        measurement_accepted = BallBalanceControl_AcceptMeasurement(
            control, now_ms, measurement);
        if (control->tracking_valid != 0U) {
            elapsed_ms = (uint32_t) (now_ms - control->last_valid_ms);
            if (elapsed_ms > BALL_BALANCE_DATA_TIMEOUT_MS) {
                BallBalanceControl_ClearVisionState(control, 0U);
                control->timed_out = 1U;
            }
        } else if ((uint32_t) (now_ms - control->enabled_since_ms) >
                   BALL_BALANCE_DATA_TIMEOUT_MS) {
            control->timed_out = 1U;
            control->desired_pipe_pitch_deg = 0.0f;
        }
    } else {
        control->desired_pipe_pitch_deg = 0.0f;
    }

    if (task_dt_s > 0.0f) {
        maximum_delta = BALL_BALANCE_PIPE_SLEW_DEG_S * task_dt_s;
        control->command_pipe_pitch_deg = BallBalanceControl_Approach(
            control->command_pipe_pitch_deg,
            control->desired_pipe_pitch_deg,
            maximum_delta);
    }
    control->command_pipe_pitch_deg = BallBalanceControl_Limit(
        control->command_pipe_pitch_deg,
        -BALL_BALANCE_MAX_PIPE_PITCH_DEG,
        BALL_BALANCE_MAX_PIPE_PITCH_DEG);

    BallBalanceControl_UpdateServoCommand(control, task_dt_s, now_ms);
    BallBalanceControl_FillOutput(control,
                                  measurement_accepted,
                                  now_ms,
                                  output);
}
