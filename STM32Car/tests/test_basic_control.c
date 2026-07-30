#include "ball_balance_control.h"
#include "ball_position_protocol.h"
#include "infrared_line_control.h"
#include "lap_stop_detector.h"
#include "linkage_kinematics.h"
#include "pipe_attitude_control.h"
#include "wheel_speed_pid.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>

#define TEST_FLOAT_EPSILON       0.0001f /* 浮点断言允许的绝对误差。 */
#define TEST_TARGET_MAX_CM_S    60.0f    /* 测试使用的单轮目标速度上限。 */
#define TEST_PI                  3.14159265f /* 测试生成姿态样本使用的圆周率。 */

static float test_abs(float value)
{
    return (value < 0.0f) ? -value : value;
}

static void assert_float_near(float actual, float expected)
{
    assert(test_abs(actual - expected) <= TEST_FLOAT_EPSILON);
}

static void assert_float_near_tolerance(float actual,
                                        float expected,
                                        float tolerance)
{
    assert(test_abs(actual - expected) <= tolerance);
}

static void assert_targets_in_range(const InfraredLineOutput *output)
{
    assert(output->left_target_cm_s >= 0.0f);
    assert(output->left_target_cm_s <= TEST_TARGET_MAX_CM_S);
    assert(output->right_target_cm_s >= 0.0f);
    assert(output->right_target_cm_s <= TEST_TARGET_MAX_CM_S);
}

static void feed_ball_protocol_text(const char *text)
{
    while (*text != '\0') {
        BallPositionProtocol_FeedByte((uint8_t) *text);
        text++;
    }
}

static void test_infrared_eight_positions(void)
{
    static const uint8_t masks[8] = {
        0x01U, 0x02U, 0x04U, 0x08U,
        0x10U, 0x20U, 0x40U, 0x80U
    };
    static const float expected_errors[8] = {
        -4.2f, -3.0f, -1.8f, -0.6f,
        0.6f, 1.8f, 3.0f, 4.2f
    };
    InfraredLineControl control;
    InfraredLineOutput output;
    uint8_t index;

    for (index = 0U; index < 8U; index++) {
        InfraredLineControl_Init(&control);
        InfraredLineControl_Update(&control, 0U, masks[index], 0U, &output);
        assert(output.command_valid != 0U);
        assert(output.active_count == 1U);
        assert_float_near(output.raw_error, expected_errors[index]);
        assert_float_near(output.filtered_error, expected_errors[index]);
        assert_targets_in_range(&output);
    }

    InfraredLineControl_Init(&control);
    InfraredLineControl_Update(&control, 0U, 0x18U, 0U, &output);
    assert(output.command_valid != 0U);
    assert(output.active_count == 2U);
    assert_float_near(output.raw_error, 0.0f);
    assert_float_near(output.filtered_error, 0.0f);
    assert_float_near(output.left_target_cm_s, output.right_target_cm_s);

    InfraredLineControl_Init(&control);
    InfraredLineControl_Update(&control, 0U, 0x03U, 0U, &output);
    assert_float_near(output.raw_error, -3.6f);

    InfraredLineControl_Init(&control);
    InfraredLineControl_Update(&control, 0U, 0xC0U, 0U, &output);
    assert_float_near(output.raw_error, 3.6f);

    InfraredLineControl_Init(&control);
    InfraredLineControl_Update(&control, 0U, 0xFFU, 0U, &output);
    assert(output.command_valid != 0U);
    assert(output.wide_line != 0U);
    assert_float_near(output.filtered_error, 0.0f);
}

static void test_infrared_forced_wide_slowdown(void)
{
    InfraredLineControl normal_control;
    InfraredLineControl slowed_control;
    InfraredLineOutput normal_output;
    InfraredLineOutput slowed_output;

    InfraredLineControl_Init(&normal_control);
    InfraredLineControl_Init(&slowed_control);
    InfraredLineControl_Update(&normal_control, 0U, 0x1FU, 0U,
                               &normal_output);
    InfraredLineControl_Update(&slowed_control, 0U, 0x1FU, 1U,
                               &slowed_output);

    assert(normal_output.active_count == 5U);
    assert(normal_output.wide_line == 0U);
    assert(slowed_output.wide_line != 0U);
    assert_float_near(normal_output.correction_cm_s,
                      slowed_output.correction_cm_s);
    assert(slowed_output.left_target_cm_s <
           normal_output.left_target_cm_s);
    assert(slowed_output.right_target_cm_s <
           normal_output.right_target_cm_s);
    assert_targets_in_range(&slowed_output);
}

static void test_infrared_target_differential(void)
{
    InfraredLineControl control;
    InfraredLineOutput output;

    InfraredLineControl_Init(&control);
    InfraredLineControl_Update(&control, 0U, 0x01U, 0U, &output);
    assert(output.left_target_cm_s < output.right_target_cm_s);
    assert_targets_in_range(&output);

    InfraredLineControl_Init(&control);
    InfraredLineControl_Update(&control, 0U, 0x08U, 0U, &output);
    assert(output.left_target_cm_s < output.right_target_cm_s);
    assert(output.left_target_cm_s != output.right_target_cm_s);
    assert_targets_in_range(&output);

    InfraredLineControl_Init(&control);
    InfraredLineControl_Update(&control, 0U, 0x10U, 0U, &output);
    assert(output.left_target_cm_s > output.right_target_cm_s);
    assert(output.left_target_cm_s != output.right_target_cm_s);
    assert_targets_in_range(&output);

    InfraredLineControl_Init(&control);
    InfraredLineControl_Update(&control, 0U, 0x80U, 0U, &output);
    assert(output.left_target_cm_s > output.right_target_cm_s);
    assert_targets_in_range(&output);
}

static void test_infrared_loss_recovery(void)
{
    InfraredLineControl control;
    InfraredLineOutput output;

    InfraredLineControl_Init(&control);
    InfraredLineControl_Update(&control, 0U, 0x08U, 0U, &output);
    assert(output.left_target_cm_s < output.right_target_cm_s);

    InfraredLineControl_Update(&control, 200U, 0x00U, 0U, &output);
    assert(output.command_valid != 0U);
    assert(output.line_lost != 0U);
    assert_float_near(output.raw_error, -4.2f);
    assert(output.correction_cm_s < 0.0f);
    assert(output.left_target_cm_s < output.right_target_cm_s);
    assert_targets_in_range(&output);

    InfraredLineControl_Update(&control, 220U, 0x00U, 0U, &output);
    assert(output.command_valid != 0U);
    assert(output.line_lost != 0U);
    assert_float_near(output.raw_error, -4.2f);
    assert(output.correction_cm_s < 0.0f);
    assert(output.left_target_cm_s < output.right_target_cm_s);
    assert_targets_in_range(&output);

    InfraredLineControl_Update(&control, 500U, 0x00U, 0U, &output);
    assert(output.command_valid != 0U);
    InfraredLineControl_Update(&control, 501U, 0x00U, 0U, &output);
    assert(output.command_valid == 0U);

    InfraredLineControl_Init(&control);
    InfraredLineControl_Update(&control, 0U, 0x10U, 0U, &output);
    InfraredLineControl_Update(&control, 100U, 0x00U, 0U, &output);
    assert(output.command_valid != 0U);
    assert_float_near(output.raw_error, 4.2f);
    assert(output.correction_cm_s > 0.0f);
    assert(output.left_target_cm_s > output.right_target_cm_s);

    InfraredLineControl_Update(&control, 120U, 0x10U, 0U, &output);
    assert(output.command_valid != 0U);
    assert(output.line_lost == 0U);
    assert_float_near(output.raw_error, 0.6f);
}

static void test_infrared_safe_loss_behavior(void)
{
    InfraredLineControl control;
    InfraredLineOutput output;

    InfraredLineControl_Init(&control);
    InfraredLineControl_Update(&control, 0U, 0x00U, 0U, &output);
    assert(output.command_valid == 0U);
    assert(output.line_lost != 0U);

    InfraredLineControl_Init(&control);
    InfraredLineControl_Update(&control, 0U, 0x18U, 0U, &output);
    assert(output.command_valid != 0U);
    InfraredLineControl_Update(&control, 20U, 0x00U, 0U, &output);
    assert(output.command_valid == 0U);
    assert(output.line_lost != 0U);
}

static void test_lap_stop_detector(void)
{
    LapStopDetector detector;

    LapStopDetector_Init(&detector);
    assert(LapStopDetector_Update(&detector, 399.0f, 1U, 0xFFU) == 0U);
    assert(detector.armed == 0U);
    assert(detector.full_black_count == 0U);

    assert(LapStopDetector_Update(&detector, -10.0f, 1U, 0xFFU) == 0U);
    assert_float_near(detector.distance_cm, 399.0f);

    /* 达到里程门槛但只有 3 路黑线时，不得停车。 */
    assert(LapStopDetector_Update(&detector, 1.0f, 1U, 0x07U) == 0U);
    assert(detector.armed != 0U);
    assert(detector.full_black_count == 0U);

    /* 4 路黑线开始连续确认，中间降到 3 路时计数清零。 */
    assert(LapStopDetector_Update(&detector, 0.0f, 1U, 0x0FU) == 0U);
    assert(detector.full_black_count == 1U);
    assert(LapStopDetector_Update(&detector, 0.0f, 1U, 0x07U) == 0U);
    assert(detector.full_black_count == 0U);

    /* I2C 无效采样不能沿用陈旧掩码完成终点确认。 */
    assert(LapStopDetector_Update(&detector, 0.0f, 1U, 0x0FU) == 0U);
    assert(LapStopDetector_Update(&detector, 0.0f, 0U, 0x0FU) == 0U);
    assert(detector.full_black_count == 0U);

    /* 4 路和 8 路黑线均满足门槛，连续两帧后完成停车。 */
    assert(LapStopDetector_Update(&detector, 0.0f, 1U, 0x0FU) == 0U);
    assert(LapStopDetector_Update(&detector, 0.0f, 1U, 0xFFU) != 0U);
    assert(detector.completed != 0U);
    assert(detector.full_black_count == 2U);
    assert(LapStopDetector_Update(&detector, 10.0f, 1U, 0x0FU) != 0U);
    assert_float_near(detector.distance_cm, 400.0f);

    LapStopDetector_Init(&detector);
    assert_float_near(detector.distance_cm, 0.0f);
    assert(detector.armed == 0U);
    assert(detector.full_black_count == 0U);
    assert(detector.completed == 0U);
}

static void test_ball_position_protocol(void)
{
    BallPositionSample sample;
    uint8_t index;

    BallPositionProtocol_Init();
    feed_ball_protocol_text("B,1,-125,86,42\r\n");
    assert(BallPositionProtocol_TakeSample(&sample) != 0U);
    assert(sample.valid == 1U);
    assert(sample.position_d10_mm == -125);
    assert(sample.quality == 86U);
    assert(sample.sequence == 42U);
    assert(BallPositionProtocol_TakeSample(&sample) == 0U);

    feed_ball_protocol_text("B,0,0,0,43\n");
    assert(BallPositionProtocol_TakeSample(&sample) != 0U);
    assert(sample.valid == 0U);
    assert(sample.sequence == 43U);

    feed_ball_protocol_text("B,1,250");
    assert(BallPositionProtocol_TakeSample(&sample) == 0U);
    feed_ball_protocol_text(",90,44\n");
    assert(BallPositionProtocol_TakeSample(&sample) != 0U);
    assert(sample.position_d10_mm == 250);
    assert(sample.quality == 90U);

    feed_ball_protocol_text("X,1,0,80,45\n");
    feed_ball_protocol_text("B,1,0,101,46\n");
    feed_ball_protocol_text("B,1,0,80,70000\n");
    feed_ball_protocol_text("B,999999999999,0,80,47\n");
    assert(g_ball_protocol_invalid_frame_count == 4U);

    for (index = 0U; index < 60U; index++) {
        BallPositionProtocol_FeedByte((uint8_t) 'X');
    }
    BallPositionProtocol_FeedByte((uint8_t) '\n');
    assert(g_ball_protocol_line_overflow_count == 1U);

    feed_ball_protocol_text("B,1,10,75,47\n");
    assert(BallPositionProtocol_TakeSample(&sample) != 0U);
    assert(sample.position_d10_mm == 10);
    assert(sample.sequence == 47U);
    assert(g_ball_protocol_valid_frame_count == 4U);
}

static void test_linkage_kinematics(void)
{
    static const float expected_servo_angles[13] = {
        97.8770f, 101.4523f, 105.1815f, 109.0738f, 113.1501f,
        117.4432f, 122.0000f, 126.8872f, 132.2031f, 138.1002f,
        144.8381f, 152.9286f, 163.7114f
    };
    float horn_angle_deg;
    float servo_angle_deg;
    float recovered_pitch_deg;
    int8_t pitch_deg;
    uint8_t index;

    assert_float_near_tolerance(
        LinkageKinematics_GetNominalServoCenterDeg(), 122.0f, 0.0002f);
    for (index = 0U; index < 13U; index++) {
        pitch_deg = (int8_t) index - 6;
        assert(LinkageKinematics_PipePitchToHornAngle(
                   (float) pitch_deg, &horn_angle_deg) != 0U);
        assert(LinkageKinematics_PipePitchToServoAngle(
                   (float) pitch_deg, &servo_angle_deg) != 0U);
        assert_float_near_tolerance(servo_angle_deg,
                                    expected_servo_angles[index],
                                    0.002f);
        assert(LinkageKinematics_ServoAngleToPipePitch(
                   servo_angle_deg, &recovered_pitch_deg) != 0U);
        assert_float_near_tolerance(recovered_pitch_deg,
                                    (float) pitch_deg,
                                    0.002f);
    }
    assert(LinkageKinematics_PipePitchToServoAngle(
               -8.0f, &servo_angle_deg) != 0U);
    assert_float_near_tolerance(servo_angle_deg, 91.2474f, 0.002f);
    assert(LinkageKinematics_PipePitchToServoAngle(
               6.8f, &servo_angle_deg) != 0U);
    assert_float_near_tolerance(servo_angle_deg, 178.7746f, 0.002f);
    assert(LinkageKinematics_PipePitchToServoAngle(-8.1f,
                                                    &servo_angle_deg) == 0U);
    assert(LinkageKinematics_PipePitchToServoAngle(6.9f,
                                                    &servo_angle_deg) == 0U);
    assert(LinkageKinematics_ServoAngleToPipePitch(90.0f,
                                                    &recovered_pitch_deg) == 0U);
}

static void test_pipe_attitude_control(void)
{
    PipeAttitudeControl control;
    PipeAttitudeMeasurement measurement;
    PipeAttitudeOutput output;
    float servo_angle;
    float model_pitch_deg;
    float raw_pitch_deg;
    float previous_raw_pitch_deg = 0.0f;
    float raw_pitch_rate_dps;
    float pitch_rad;
    uint32_t now_ms;
    uint16_t iteration;

    servo_angle = LinkageKinematics_GetNominalServoCenterDeg();
    PipeAttitudeControl_Init(&control);
    PipeAttitudeControl_Update(&control, 0U, 0, 0.0f, &output);
    assert(output.calibration_valid == 0U);
    assert(output.attitude_valid == 0U);

    assert(PipeAttitudeControl_StartCalibration(
               &control, 0U, servo_angle) != 0U);
    PipeAttitudeControl_Update(&control, 0U, 0, 0.0f, &output);
    assert(output.status_changed != 0U);
    assert(output.calibration_event == PIPE_CALIBRATION_EVENT_READY);
    assert(output.servo_override_active != 0U);

    now_ms = 0U;
    for (iteration = 0U; iteration < 1000U; iteration++) {
        now_ms += 20U;
        assert(LinkageKinematics_ServoAngleToPipePitch(
                   servo_angle, &model_pitch_deg) != 0U);
        /* 模拟远端 IMU 符号反转和柔性水管 0.42 倍角度响应。 */
        raw_pitch_deg = -0.42f * model_pitch_deg;
        raw_pitch_rate_dps =
            (raw_pitch_deg - previous_raw_pitch_deg) / 0.020f;
        pitch_rad = raw_pitch_deg * TEST_PI / 180.0f;
        measurement.valid = 1U;
        measurement.timestamp_ms = now_ms;
        measurement.accel_x_g = 0.0f;
        measurement.accel_y_g = sinf(pitch_rad);
        measurement.accel_z_g = cosf(pitch_rad);
        measurement.gyro_x_dps = 0.40f + raw_pitch_rate_dps;
        measurement.gyro_y_dps = 0.0f;
        measurement.gyro_z_dps = 0.0f;
        PipeAttitudeControl_Update(&control,
                                   now_ms,
                                   &measurement,
                                   0.0f,
                                   &output);
        if (output.servo_override_active != 0U) {
            servo_angle = output.servo_override_deg;
        }
        previous_raw_pitch_deg = raw_pitch_deg;
        if (output.calibration_valid != 0U) {
            break;
        }
    }

    assert(output.calibration_valid != 0U);
    assert(output.calibration_event == PIPE_CALIBRATION_EVENT_SUCCESS);
    assert(output.imu_pitch_sign == -1);
    assert_float_near_tolerance(output.gyro_x_bias_dps, 0.40f, 0.001f);
    assert(output.negative_pitch_deg < -3.2f);
    assert(output.center_pitch_deg > -0.1f);
    assert(output.center_pitch_deg < 0.1f);
    assert(output.positive_pitch_deg > 2.7f);
    assert_float_near_tolerance(output.model_scale, 0.42f, 0.03f);
    assert_float_near_tolerance(output.model_offset_deg, 0.0f, 0.08f);
    assert(test_abs(output.return_pitch_deg - output.center_pitch_deg) < 0.1f);

    now_ms += 20U;
    measurement.timestamp_ms = now_ms;
    measurement.accel_y_g = 0.0f;
    measurement.accel_z_g = 1.0f;
    measurement.gyro_x_dps = 0.40f;
    PipeAttitudeControl_Update(&control,
                               now_ms,
                               &measurement,
                               100.0f,
                               &output);
    assert(output.attitude_valid != 0U);
    assert(output.accel_trusted == 0U);

    assert(PipeAttitudeControl_StartCalibration(
               &control, now_ms, servo_angle) != 0U);
    PipeAttitudeControl_AbortCalibration(&control, now_ms);
    PipeAttitudeControl_Update(&control, now_ms, 0, 0.0f, &output);
    assert(output.calibration_event == PIPE_CALIBRATION_EVENT_ABORTED);
    assert(output.calibration_valid == 0U);
}

static void test_ball_balance_control(void)
{
    BallBalanceControl control;
    BallBalanceMeasurement measurement;
    BallBalanceAttitude attitude;
    BallBalanceOutput output;
    float neutral_angle_deg;
    uint32_t now_ms;
    uint8_t index;

    neutral_angle_deg = BallBalanceControl_GetNeutralAngleDeg();
    BallBalanceControl_Init(&control);
    BallBalanceControl_SetEnabled(&control, 1U, 0U);
    BallBalanceControl_Update(&control, 0U, 0, 0, &output);
    assert(output.enable_requested == 0U);
    assert_float_near_tolerance(output.command_angle_deg,
                                neutral_angle_deg,
                                0.0002f);

    BallBalanceControl_SetPipeCalibration(&control, 1U, 1.0f, 0.0f);
    BallBalanceControl_SetEnabled(&control, 1U, 0U);
    assert(BallBalanceControl_IsEnabled(&control) != 0U);

    attitude.valid = 1U;
    attitude.pitch_deg = 0.0f;
    attitude.pitch_rate_dps = 0.0f;
    now_ms = 0U;
    for (index = 0U; index < 30U; index++) {
        now_ms += 20U;
        attitude.timestamp_ms = now_ms;
        BallBalanceControl_Update(&control,
                                  now_ms,
                                  0,
                                  &attitude,
                                  &output);
    }
    assert(output.enabled != 0U);
    assert(output.enable_pending == 0U);
    assert_float_near_tolerance(output.command_angle_deg,
                                neutral_angle_deg,
                                0.001f);

    measurement.valid = 1U;
    measurement.position_d10_mm = 500;
    measurement.quality = 80U;
    measurement.sequence = 1U;
    now_ms += 20U;
    attitude.timestamp_ms = now_ms;
    BallBalanceControl_Update(&control,
                              now_ms,
                              &measurement,
                              &attitude,
                              &output);
    assert(output.measurement_accepted != 0U);
    assert(output.tracking_valid != 0U);
    assert(output.desired_pipe_pitch_deg > 0.0f);
    assert(output.desired_model_pitch_deg > 0.0f);
    assert(output.desired_angle_deg > neutral_angle_deg);
    assert(output.command_angle_deg > neutral_angle_deg);
    assert(output.command_angle_deg >= 90.0f);
    assert(output.command_angle_deg <= 180.0f);

    measurement.position_d10_mm = -500;
    now_ms += 20U;
    attitude.timestamp_ms = now_ms;
    BallBalanceControl_Update(&control,
                              now_ms,
                              &measurement,
                              &attitude,
                              &output);
    assert(output.measurement_accepted == 0U);
    assert(output.filtered_position_mm > 0.0f);

    BallBalanceControl_SetTargetMm(&control, 200.0f);
    now_ms += 20U;
    attitude.timestamp_ms = now_ms;
    BallBalanceControl_Update(&control,
                              now_ms,
                              0,
                              &attitude,
                              &output);
    assert_float_near(output.target_mm, 100.0f);

    now_ms += 220U;
    attitude.timestamp_ms = now_ms;
    BallBalanceControl_Update(&control,
                              now_ms,
                              0,
                              &attitude,
                              &output);
    assert(output.timed_out != 0U);
    assert(output.tracking_valid == 0U);
    assert_float_near(output.desired_pipe_pitch_deg, 0.0f);

    now_ms += 120U;
    BallBalanceControl_Update(&control, now_ms, 0, 0, &output);
    assert(output.enabled == 0U);
    assert(output.imu_fault != 0U);
    assert_float_near_tolerance(output.desired_angle_deg,
                                neutral_angle_deg,
                                0.001f);

    for (index = 0U; index < 30U; index++) {
        now_ms += 20U;
        attitude.timestamp_ms = now_ms;
        attitude.pitch_deg = 0.0f;
        attitude.pitch_rate_dps = 0.0f;
        BallBalanceControl_Update(&control,
                                  now_ms,
                                  0,
                                  &attitude,
                                  &output);
    }
    assert(output.enabled != 0U);
    assert(output.imu_fault == 0U);

    now_ms += 20U;
    attitude.timestamp_ms = now_ms;
    attitude.pitch_deg = 7.0f;
    BallBalanceControl_Update(&control,
                              now_ms,
                              0,
                              &attitude,
                              &output);
    assert(output.enabled == 0U);
    assert(output.imu_fault != 0U);

    BallBalanceControl_SetEnabled(&control, 0U, now_ms);
    BallBalanceControl_Update(&control, now_ms + 20U, 0, 0, &output);
    assert(output.enable_requested == 0U);
    assert(output.enabled == 0U);
}

static void test_flexible_pipe_mapping_limits(void)
{
    BallBalanceControl control;
    BallBalanceMeasurement measurement;
    BallBalanceAttitude attitude;
    BallBalanceOutput output;
    uint32_t now_ms = 0U;
    uint8_t index;

    BallBalanceControl_Init(&control);
    BallBalanceControl_SetPipeCalibration(&control, 1U, 0.42f, 0.38f);
    BallBalanceControl_Update(&control, now_ms, 0, 0, &output);
    assert(output.mapping_valid != 0U);
    assert_float_near_tolerance(
        output.minimum_pipe_pitch_deg, -2.98f, 0.01f);
    assert_float_near_tolerance(
        output.maximum_pipe_pitch_deg, 3.00f, 0.01f);

    BallBalanceControl_SetEnabled(&control, 1U, now_ms);
    attitude.valid = 1U;
    attitude.pitch_deg = 0.0f;
    attitude.pitch_rate_dps = 0.0f;
    for (index = 0U; index < 30U; index++) {
        now_ms += 20U;
        attitude.timestamp_ms = now_ms;
        BallBalanceControl_Update(&control,
                                  now_ms,
                                  0,
                                  &attitude,
                                  &output);
    }
    assert(output.enabled != 0U);

    measurement.valid = 1U;
    measurement.position_d10_mm = 1000;
    measurement.quality = 80U;
    measurement.sequence = 1U;
    now_ms += 20U;
    attitude.timestamp_ms = now_ms;
    BallBalanceControl_Update(&control,
                              now_ms,
                              &measurement,
                              &attitude,
                              &output);
    assert_float_near_tolerance(
        output.desired_pipe_pitch_deg, 3.00f, 0.01f);
    assert(output.mapping_valid != 0U);
    assert(output.mapping_fault == 0U);
    assert(output.desired_model_pitch_deg <= 6.8f);

    now_ms += 220U;
    measurement.position_d10_mm = -1000;
    measurement.sequence = 2U;
    attitude.timestamp_ms = now_ms;
    BallBalanceControl_Update(&control,
                              now_ms,
                              &measurement,
                              &attitude,
                              &output);
    assert_float_near_tolerance(
        output.desired_pipe_pitch_deg, -2.98f, 0.01f);
    assert(output.mapping_valid != 0U);
    assert(output.mapping_fault == 0U);
    assert(output.desired_model_pitch_deg >= -8.0f);
}

static void test_speed_pid(void)
{
    WheelSpeedPid pid;

    WheelSpeedPid_Init(&pid);
    WheelSpeedPid_SetTarget(&pid, 10.0f);
    assert(WheelSpeedPid_Step(&pid, 0.0f) > 0.0f);
    assert(WheelSpeedPid_Step(&pid, 10.0f) <= 0.50f);
    WheelSpeedPid_SetTarget(&pid, 0.0f);
    assert(WheelSpeedPid_Step(&pid, 10.0f) == 0.0f);
}

int main(void)
{
    test_infrared_eight_positions();
    test_infrared_forced_wide_slowdown();
    test_infrared_target_differential();
    test_infrared_loss_recovery();
    test_infrared_safe_loss_behavior();
    test_lap_stop_detector();
    test_ball_position_protocol();
    test_linkage_kinematics();
    test_pipe_attitude_control();
    test_ball_balance_control();
    test_flexible_pipe_mapping_limits();
    test_speed_pid();
    puts("基础控制、IMU 姿态与钢球平衡主机测试通过");
    return 0;
}
