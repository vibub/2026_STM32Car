#include "ball_balance_control.h"

#include <string.h>

#define BALL_BALANCE_NEUTRAL_ANGLE_DEG      90.0f  /* 摆杆水平时的舵机初始中位角。 */
#define BALL_BALANCE_MIN_ANGLE_DEG          85.0f  /* 未标定机械结构时的最小安全舵机角。 */
#define BALL_BALANCE_MAX_ANGLE_DEG          95.0f  /* 未标定机械结构时的最大安全舵机角。 */
#define BALL_BALANCE_DIRECTION_SIGN          1.0f  /* 舵机角度方向修正，实测反向时改为 -1。 */
#define BALL_BALANCE_KP_DEG_PER_MM            0.040f /* 钢球位置误差的比例增益。 */
#define BALL_BALANCE_KI_DEG_PER_MM_S          0.000f /* 钢球位置误差的积分增益，首版保持为 0。 */
#define BALL_BALANCE_KD_DEG_PER_MM_S          0.012f /* 钢球速度阻尼增益。 */
#define BALL_BALANCE_INTEGRAL_LIMIT_MM_S    200.0f  /* 位置误差积分绝对上限。 */
#define BALL_BALANCE_MIN_QUALITY             50U    /* 接受 K230 测量所需的最低质量。 */
#define BALL_BALANCE_DATA_TIMEOUT_MS        200U    /* 无合格视觉数据后回中位的超时时间。 */
#define BALL_BALANCE_TARGET_LIMIT_MM        100.0f  /* 可设置目标位置的绝对上限。 */
#define BALL_BALANCE_POSITION_LIMIT_MM      130.0f  /* 合法钢球测量位置的绝对上限。 */
#define BALL_BALANCE_SERVO_SLEW_DEG_S        30.0f  /* 舵机命令最大变化率。 */
#define BALL_BALANCE_DT_MIN_S                 0.010f /* 视觉测量 dt 的最小值。 */
#define BALL_BALANCE_DT_MAX_S                 0.100f /* 视觉测量和任务 dt 的最大值。 */
#define BALL_BALANCE_POSITION_FILTER_TAU_S    0.050f /* 钢球位置低通滤波时间常数。 */
#define BALL_BALANCE_VELOCITY_FILTER_TAU_S    0.080f /* 钢球速度低通滤波时间常数。 */

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

static void BallBalanceControl_ClearDynamicState(BallBalanceControl *control,
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
    control->desired_angle_deg = BALL_BALANCE_NEUTRAL_ANGLE_DEG;
}

static void BallBalanceControl_CalculateCommand(BallBalanceControl *control,
                                                float measurement_dt_s,
                                                uint8_t allow_integral)
{
    float candidate_integral = control->integral_mm_s;
    float candidate_i_term;
    float candidate_angle;
    float integral_direction;

    control->error_mm = control->target_mm - control->filtered_position_mm;
    control->p_term_deg = BALL_BALANCE_KP_DEG_PER_MM * control->error_mm;
    control->d_term_deg = -BALL_BALANCE_KD_DEG_PER_MM_S *
                          control->velocity_mm_s;

    if (allow_integral != 0U) {
        candidate_integral = BallBalanceControl_Limit(
            control->integral_mm_s + control->error_mm * measurement_dt_s,
            -BALL_BALANCE_INTEGRAL_LIMIT_MM_S,
            BALL_BALANCE_INTEGRAL_LIMIT_MM_S);
    }

    candidate_i_term = BALL_BALANCE_KI_DEG_PER_MM_S * candidate_integral;
    candidate_angle = BALL_BALANCE_NEUTRAL_ANGLE_DEG +
                      BALL_BALANCE_DIRECTION_SIGN *
                      (control->p_term_deg + candidate_i_term +
                       control->d_term_deg);
    integral_direction = BALL_BALANCE_DIRECTION_SIGN *
                         BALL_BALANCE_KI_DEG_PER_MM_S * control->error_mm;

    if ((allow_integral != 0U) &&
        (((candidate_angle >= BALL_BALANCE_MIN_ANGLE_DEG) &&
          (candidate_angle <= BALL_BALANCE_MAX_ANGLE_DEG)) ||
         ((candidate_angle > BALL_BALANCE_MAX_ANGLE_DEG) &&
          (integral_direction < 0.0f)) ||
         ((candidate_angle < BALL_BALANCE_MIN_ANGLE_DEG) &&
          (integral_direction > 0.0f)))) {
        control->integral_mm_s = candidate_integral;
    }

    control->i_term_deg = BALL_BALANCE_KI_DEG_PER_MM_S *
                          control->integral_mm_s;
    control->desired_angle_deg = BallBalanceControl_Limit(
        BALL_BALANCE_NEUTRAL_ANGLE_DEG + BALL_BALANCE_DIRECTION_SIGN *
        (control->p_term_deg + control->i_term_deg + control->d_term_deg),
        BALL_BALANCE_MIN_ANGLE_DEG,
        BALL_BALANCE_MAX_ANGLE_DEG);
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
    BallBalanceControl_CalculateCommand(control,
                                        measurement_dt_s,
                                        filter_was_ready);
    return 1U;
}

static void BallBalanceControl_FillOutput(const BallBalanceControl *control,
                                          uint8_t measurement_accepted,
                                          BallBalanceOutput *output)
{
    if (output == 0) {
        return;
    }

    memset(output, 0, sizeof(*output));
    if (control == 0) {
        return;
    }

    output->enabled = control->enabled;
    output->tracking_valid = control->tracking_valid;
    output->timed_out = control->timed_out;
    output->measurement_accepted = measurement_accepted;
    output->target_mm = control->target_mm;
    output->filtered_position_mm = control->filtered_position_mm;
    output->velocity_mm_s = control->velocity_mm_s;
    output->error_mm = control->error_mm;
    output->p_term_deg = control->p_term_deg;
    output->i_term_deg = control->i_term_deg;
    output->d_term_deg = control->d_term_deg;
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
    if (control->enabled == normalized_enabled) {
        return;
    }

    control->enabled = normalized_enabled;
    control->enabled_since_ms = now_ms;
    control->last_task_ms = now_ms;
    control->task_time_ready = 1U;
    BallBalanceControl_ClearDynamicState(control, 1U);
}

uint8_t BallBalanceControl_IsEnabled(const BallBalanceControl *control)
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

float BallBalanceControl_GetNeutralAngleDeg(void)
{
    return BALL_BALANCE_NEUTRAL_ANGLE_DEG;
}

void BallBalanceControl_Update(BallBalanceControl *control,
                               uint32_t now_ms,
                               const BallBalanceMeasurement *measurement,
                               BallBalanceOutput *output)
{
    uint32_t elapsed_ms;
    float task_dt_s = 0.0f;
    float maximum_delta;
    uint8_t measurement_accepted = 0U;

    if (control == 0) {
        BallBalanceControl_FillOutput(0, 0U, output);
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

    if (control->enabled != 0U) {
        measurement_accepted = BallBalanceControl_AcceptMeasurement(
            control, now_ms, measurement);

        if (control->tracking_valid != 0U) {
            elapsed_ms = (uint32_t) (now_ms - control->last_valid_ms);
            if (elapsed_ms > BALL_BALANCE_DATA_TIMEOUT_MS) {
                BallBalanceControl_ClearDynamicState(control, 0U);
                control->timed_out = 1U;
            }
        } else if ((uint32_t) (now_ms - control->enabled_since_ms) >
                   BALL_BALANCE_DATA_TIMEOUT_MS) {
            control->timed_out = 1U;
            control->desired_angle_deg = BALL_BALANCE_NEUTRAL_ANGLE_DEG;
        }
    } else {
        control->desired_angle_deg = BALL_BALANCE_NEUTRAL_ANGLE_DEG;
    }

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

    BallBalanceControl_FillOutput(control, measurement_accepted, output);
}
