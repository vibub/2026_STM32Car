#include <assert.h>
#include <stdint.h>
#include <stdio.h>

#include "balance_debug_log.h"

static void TestCaptureTimingAndFields(void)
{
    BalanceDebugLogInput input = {0};
    BallBalanceOutput balance = {0};
    PipeAttitudeOutput attitude = {0};
    const BalanceDebugLogSample *sample;

    BalanceDebugLog_Init();
    assert(g_balance_debug_log.magic == BALANCE_DEBUG_LOG_MAGIC);
    assert(g_balance_debug_log.format_version ==
           BALANCE_DEBUG_LOG_FORMAT_VERSION);
    assert(g_balance_debug_log.sample_size == sizeof(BalanceDebugLogSample));
    assert(g_balance_debug_log.buffer_size == sizeof(BalanceDebugLogBuffer));
    assert(sizeof(BalanceDebugLogSample) == 196U);

    input.now_ms = 100U;
    input.imu_timestamp_ms = 96U;
    input.raw_position_d10_mm = 503;
    input.ball_sequence = 12U;
    input.ball_quality = 87U;
    input.ball_sample_valid = 1U;
    input.imu_accel_y_g = 0.25f;
    input.imu_accel_z_g = 0.96f;
    input.imu_gyro_x_dps = -3.5f;
    input.actual_servo_angle_deg = 121.2f;
    input.servo_pulse_us = 1398U;

    balance.enable_requested = 1U;
    balance.enabled = 1U;
    balance.tracking_valid = 1U;
    balance.measurement_accepted = 1U;
    balance.attitude_valid = 1U;
    balance.mapping_valid = 1U;
    balance.integral_active = 1U;
    balance.trajectory_phase = 3U;
    balance.moving_sample_count = 4U;
    balance.kick_attempt_count = 2U;
    balance.moving_direction = -1;
    balance.target_mm = 0.0f;
    balance.filtered_position_mm = 49.5f;
    balance.predicted_position_mm = 48.0f;
    balance.velocity_mm_s = -12.0f;
    balance.error_mm = -49.5f;
    balance.desired_pipe_pitch_deg = -3.0f;
    balance.command_pipe_pitch_deg = -2.5f;
    balance.command_angle_deg = 121.2f;
    attitude.accel_trusted = 1U;
    attitude.calibration_valid = 1U;
    attitude.pitch_deg = -2.1f;
    attitude.pitch_rate_dps = -4.0f;

    g_balance_debug_log_command = BALANCE_DEBUG_LOG_COMMAND_START;
    BalanceDebugLog_Update(&input, &balance, &attitude);
    assert(g_balance_debug_log.count == 1U);
    assert(g_balance_debug_log.state == BALANCE_DEBUG_LOG_STATE_RECORDING);

    sample = &g_balance_debug_log.samples[0];
    assert(sample->timestamp_ms == 100U);
    assert(sample->imu_timestamp_ms == 96U);
    assert(sample->raw_position_d10_mm == 503);
    assert(sample->ball_sequence == 12U);
    assert(sample->ball_quality == 87U);
    assert(sample->servo_pulse_us == 1398U);
    assert(sample->raw_position_mm > 50.29f);
    assert(sample->raw_position_mm < 50.31f);
    assert((sample->flags & BALANCE_DEBUG_LOG_FLAG_ENABLED) != 0U);
    assert((sample->flags & BALANCE_DEBUG_LOG_FLAG_BALL_SAMPLE_VALID) != 0U);
    assert((sample->flags & BALANCE_DEBUG_LOG_FLAG_ACCEL_TRUSTED) != 0U);
    assert((sample->state_word & 0xFFU) == 3U);
    assert(((sample->state_word >> 8) & 0xFFU) == 4U);
    assert(((sample->state_word >> 16) & 0xFFU) == 2U);
    assert(((sample->state_word >> 24) & 0xFFU) == 0xFFU);

    input.now_ms = 119U;
    BalanceDebugLog_Update(&input, &balance, &attitude);
    assert(g_balance_debug_log.count == 1U);
    input.now_ms = 120U;
    BalanceDebugLog_Update(&input, &balance, &attitude);
    assert(g_balance_debug_log.count == 2U);

    g_balance_debug_log_command = BALANCE_DEBUG_LOG_COMMAND_STOP;
    input.now_ms = 140U;
    BalanceDebugLog_Update(&input, &balance, &attitude);
    assert(g_balance_debug_log.count == 2U);
    assert(g_balance_debug_log.state == BALANCE_DEBUG_LOG_STATE_STOPPED);
}

static void TestBufferAutomaticallyFreezesWhenFull(void)
{
    BalanceDebugLogInput input = {0};
    BallBalanceOutput balance = {0};
    PipeAttitudeOutput attitude = {0};
    uint32_t index;

    BalanceDebugLog_Init();
    input.now_ms = 1000U;
    g_balance_debug_log_command = BALANCE_DEBUG_LOG_COMMAND_START;
    BalanceDebugLog_Update(&input, &balance, &attitude);

    for (index = 1U; index < BALANCE_DEBUG_LOG_CAPACITY; index++) {
        input.now_ms = 1000U + index * BALANCE_DEBUG_LOG_PERIOD_MS;
        BalanceDebugLog_Update(&input, &balance, &attitude);
    }

    assert(g_balance_debug_log.count == BALANCE_DEBUG_LOG_CAPACITY);
    assert(g_balance_debug_log.state == BALANCE_DEBUG_LOG_STATE_FULL);

    input.now_ms += BALANCE_DEBUG_LOG_PERIOD_MS;
    BalanceDebugLog_Update(&input, &balance, &attitude);
    assert(g_balance_debug_log.count == BALANCE_DEBUG_LOG_CAPACITY);
}

int main(void)
{
    TestCaptureTimingAndFields();
    TestBufferAutomaticallyFreezesWhenFull();
    puts("钢球平衡诊断日志主机测试通过");
    return 0;
}
