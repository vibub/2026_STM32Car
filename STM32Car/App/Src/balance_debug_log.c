#include "balance_debug_log.h"

#if APP_ENABLE_BALANCE_LOG

volatile uint32_t g_balance_debug_log_command;
BalanceDebugLogBuffer g_balance_debug_log;

static uint32_t s_last_sample_ms;

typedef char BalanceDebugLogSampleSizeMustBe196Bytes[
    (sizeof(BalanceDebugLogSample) == 196U) ? 1 : -1];

typedef char BalanceDebugLogHeaderSizeMustBe32Bytes[
    ((sizeof(BalanceDebugLogBuffer) -
      sizeof(g_balance_debug_log.samples)) == 32U) ? 1 : -1];

static uint32_t BalanceDebugLog_BuildFlags(
    const BalanceDebugLogInput *input,
    const BallBalanceOutput *balance_output,
    const PipeAttitudeOutput *attitude_output)
{
    uint32_t flags = 0U;

    if (balance_output->enable_requested != 0U) {
        flags |= BALANCE_DEBUG_LOG_FLAG_ENABLE_REQUESTED;
    }
    if (balance_output->enabled != 0U) {
        flags |= BALANCE_DEBUG_LOG_FLAG_ENABLED;
    }
    if (balance_output->enable_pending != 0U) {
        flags |= BALANCE_DEBUG_LOG_FLAG_ENABLE_PENDING;
    }
    if (input->ball_sample_valid != 0U) {
        flags |= BALANCE_DEBUG_LOG_FLAG_BALL_SAMPLE_VALID;
    }
    if (balance_output->tracking_valid != 0U) {
        flags |= BALANCE_DEBUG_LOG_FLAG_TRACKING_VALID;
    }
    if (balance_output->timed_out != 0U) {
        flags |= BALANCE_DEBUG_LOG_FLAG_TIMED_OUT;
    }
    if (balance_output->measurement_accepted != 0U) {
        flags |= BALANCE_DEBUG_LOG_FLAG_MEASUREMENT_ACCEPTED;
    }
    if (balance_output->attitude_valid != 0U) {
        flags |= BALANCE_DEBUG_LOG_FLAG_ATTITUDE_VALID;
    }
    if (balance_output->imu_fault != 0U) {
        flags |= BALANCE_DEBUG_LOG_FLAG_IMU_FAULT;
    }
    if (balance_output->mapping_valid != 0U) {
        flags |= BALANCE_DEBUG_LOG_FLAG_MAPPING_VALID;
    }
    if (balance_output->mapping_fault != 0U) {
        flags |= BALANCE_DEBUG_LOG_FLAG_MAPPING_FAULT;
    }
    if (balance_output->integral_active != 0U) {
        flags |= BALANCE_DEBUG_LOG_FLAG_INTEGRAL_ACTIVE;
    }
    if (balance_output->braking_active != 0U) {
        flags |= BALANCE_DEBUG_LOG_FLAG_BRAKING_ACTIVE;
    }
    if (balance_output->zero_crossed != 0U) {
        flags |= BALANCE_DEBUG_LOG_FLAG_ZERO_CROSSED;
    }
    if (balance_output->turnaround_freeze_active != 0U) {
        flags |= BALANCE_DEBUG_LOG_FLAG_TURNAROUND_FREEZE;
    }
    if (balance_output->pipe_settled != 0U) {
        flags |= BALANCE_DEBUG_LOG_FLAG_PIPE_SETTLED;
    }
    if (balance_output->endpoint_hold_enabled != 0U) {
        flags |= BALANCE_DEBUG_LOG_FLAG_ENDPOINT_HOLD;
    }
    if (balance_output->transport_mode_enabled != 0U) {
        flags |= BALANCE_DEBUG_LOG_FLAG_TRANSPORT_MODE;
    }
    if (attitude_output->accel_trusted != 0U) {
        flags |= BALANCE_DEBUG_LOG_FLAG_ACCEL_TRUSTED;
    }
    if (attitude_output->servo_override_active != 0U) {
        flags |= BALANCE_DEBUG_LOG_FLAG_SERVO_OVERRIDE;
    }
    if (attitude_output->calibration_valid != 0U) {
        flags |= BALANCE_DEBUG_LOG_FLAG_CALIBRATION_VALID;
    }
    return flags;
}

static uint32_t BalanceDebugLog_BuildStateWord(
    const BallBalanceOutput *balance_output)
{
    return ((uint32_t) balance_output->trajectory_phase) |
           ((uint32_t) balance_output->moving_sample_count << 8) |
           ((uint32_t) balance_output->kick_attempt_count << 16) |
           ((uint32_t) (uint8_t) balance_output->moving_direction << 24);
}

static void BalanceDebugLog_Start(uint32_t now_ms)
{
    g_balance_debug_log.count = 0U;
    g_balance_debug_log.state = (uint32_t) BALANCE_DEBUG_LOG_STATE_RECORDING;
    /* 首次调用立即保存一帧，便于记录启用平衡前的初始状态。 */
    s_last_sample_ms = now_ms - BALANCE_DEBUG_LOG_PERIOD_MS;
}

static void BalanceDebugLog_Capture(
    const BalanceDebugLogInput *input,
    const BallBalanceOutput *balance_output,
    const PipeAttitudeOutput *attitude_output)
{
    BalanceDebugLogSample *sample;
    uint32_t index = g_balance_debug_log.count;

    if (index >= BALANCE_DEBUG_LOG_CAPACITY) {
        g_balance_debug_log.state = (uint32_t) BALANCE_DEBUG_LOG_STATE_FULL;
        return;
    }

    sample = &g_balance_debug_log.samples[index];
    sample->timestamp_ms = input->now_ms;
    sample->imu_timestamp_ms = input->imu_timestamp_ms;
    sample->raw_position_d10_mm = input->raw_position_d10_mm;
    sample->ball_sequence = input->ball_sequence;
    sample->ball_quality = input->ball_quality;
    sample->servo_pulse_us = input->servo_pulse_us;
    sample->flags = BalanceDebugLog_BuildFlags(input,
                                               balance_output,
                                               attitude_output);
    sample->state_word = BalanceDebugLog_BuildStateWord(balance_output);
    sample->target_mm = balance_output->target_mm;
    sample->raw_position_mm = 0.1f * (float) input->raw_position_d10_mm;
    sample->filtered_position_mm = balance_output->filtered_position_mm;
    sample->predicted_position_mm = balance_output->predicted_position_mm;
    sample->predicted_accel_mm_s2 = balance_output->predicted_accel_mm_s2;
    sample->velocity_mm_s = balance_output->velocity_mm_s;
    sample->integral_mm_s = balance_output->integral_mm_s;
    sample->error_mm = balance_output->error_mm;
    sample->p_term_deg = balance_output->p_term_deg;
    sample->i_term_deg = balance_output->i_term_deg;
    sample->d_term_deg = balance_output->d_term_deg;
    sample->brake_term_deg = balance_output->brake_term_deg;
    sample->desired_pipe_pitch_deg = balance_output->desired_pipe_pitch_deg;
    sample->command_pipe_pitch_deg = balance_output->command_pipe_pitch_deg;
    sample->pipe_command_rate_dps = balance_output->pipe_command_rate_dps;
    sample->measured_pipe_pitch_deg = balance_output->measured_pipe_pitch_deg;
    sample->measured_pipe_rate_dps = balance_output->measured_pipe_rate_dps;
    sample->pipe_pitch_error_deg = balance_output->pipe_pitch_error_deg;
    sample->pipe_p_term_deg = balance_output->pipe_p_term_deg;
    sample->pipe_d_term_deg = balance_output->pipe_d_term_deg;
    sample->minimum_pipe_pitch_deg = balance_output->minimum_pipe_pitch_deg;
    sample->maximum_pipe_pitch_deg = balance_output->maximum_pipe_pitch_deg;
    sample->feedforward_model_pitch_deg =
        balance_output->feedforward_model_pitch_deg;
    sample->desired_model_pitch_deg = balance_output->desired_model_pitch_deg;
    sample->servo_feedforward_deg = balance_output->servo_feedforward_deg;
    sample->desired_angle_deg = balance_output->desired_angle_deg;
    sample->command_angle_deg = balance_output->command_angle_deg;
    sample->actual_servo_angle_deg = input->actual_servo_angle_deg;
    sample->raw_accel_pitch_deg = attitude_output->raw_accel_pitch_deg;
    sample->accel_pitch_deg = attitude_output->accel_pitch_deg;
    sample->raw_pitch_deg = attitude_output->raw_pitch_deg;
    sample->raw_pitch_rate_dps = attitude_output->raw_pitch_rate_dps;
    sample->imu_accel_y_g = input->imu_accel_y_g;
    sample->imu_accel_z_g = input->imu_accel_z_g;
    sample->imu_gyro_x_dps = input->imu_gyro_x_dps;
    sample->transport_command_accel_cm_s2 =
        balance_output->transport_command_accel_cm_s2;
    sample->transport_imu_disturbance_deg =
        balance_output->transport_imu_disturbance_deg;
    sample->transport_feedforward_pitch_deg =
        balance_output->transport_feedforward_pitch_deg;
    sample->transport_feedback_pitch_deg =
        balance_output->transport_feedback_pitch_deg;
    sample->phase_pitch_deg = balance_output->phase_pitch_deg;
    sample->stop_distance_mm = balance_output->stop_distance_mm;

    g_balance_debug_log.count = index + 1U;
    if (g_balance_debug_log.count >= BALANCE_DEBUG_LOG_CAPACITY) {
        g_balance_debug_log.state = (uint32_t) BALANCE_DEBUG_LOG_STATE_FULL;
    }
}

void BalanceDebugLog_Init(void)
{
    g_balance_debug_log.magic = BALANCE_DEBUG_LOG_MAGIC;
    g_balance_debug_log.format_version = BALANCE_DEBUG_LOG_FORMAT_VERSION;
    g_balance_debug_log.sample_size = (uint32_t) sizeof(BalanceDebugLogSample);
    g_balance_debug_log.capacity = BALANCE_DEBUG_LOG_CAPACITY;
    g_balance_debug_log.count = 0U;
    g_balance_debug_log.state = (uint32_t) BALANCE_DEBUG_LOG_STATE_IDLE;
    g_balance_debug_log.period_ms = BALANCE_DEBUG_LOG_PERIOD_MS;
    g_balance_debug_log.buffer_size =
        (uint32_t) sizeof(BalanceDebugLogBuffer);
    g_balance_debug_log_command = BALANCE_DEBUG_LOG_COMMAND_NONE;
    s_last_sample_ms = 0U;
}

void BalanceDebugLog_Update(const BalanceDebugLogInput *input,
                            const BallBalanceOutput *balance_output,
                            const PipeAttitudeOutput *attitude_output)
{
    uint32_t command;

    if ((input == 0) || (balance_output == 0) || (attitude_output == 0)) {
        return;
    }

    command = g_balance_debug_log_command;
    if (command == BALANCE_DEBUG_LOG_COMMAND_START) {
        BalanceDebugLog_Start(input->now_ms);
        g_balance_debug_log_command = BALANCE_DEBUG_LOG_COMMAND_NONE;
    } else if (command == BALANCE_DEBUG_LOG_COMMAND_STOP) {
        if (g_balance_debug_log.state ==
            (uint32_t) BALANCE_DEBUG_LOG_STATE_RECORDING) {
            g_balance_debug_log.state =
                (uint32_t) BALANCE_DEBUG_LOG_STATE_STOPPED;
        }
        g_balance_debug_log_command = BALANCE_DEBUG_LOG_COMMAND_NONE;
    }

    if (g_balance_debug_log.state !=
        (uint32_t) BALANCE_DEBUG_LOG_STATE_RECORDING) {
        return;
    }
    if ((uint32_t) (input->now_ms - s_last_sample_ms) <
        BALANCE_DEBUG_LOG_PERIOD_MS) {
        return;
    }

    /* 与主控制循环相同，错过周期时不使用同一测量回填多个伪样本。 */
    s_last_sample_ms = input->now_ms;
    BalanceDebugLog_Capture(input, balance_output, attitude_output);
}

#else

void BalanceDebugLog_Init(void)
{
}

void BalanceDebugLog_Update(const BalanceDebugLogInput *input,
                            const BallBalanceOutput *balance_output,
                            const PipeAttitudeOutput *attitude_output)
{
    (void) input;
    (void) balance_output;
    (void) attitude_output;
}

#endif
