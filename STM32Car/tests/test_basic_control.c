#include "ball_balance_control.h"
#include "ball_position_protocol.h"
#include "ball_shuttle_control.h"
#include "infrared_line_control.h"
#include "lap_stop_detector.h"
#include "linkage_kinematics.h"
#include "pipe_attitude_control.h"
#include "wheel_speed_pid.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>

#if defined(_WIN32)
/* 命令行测试失败时写入终端，避免 Windows CRT 弹出阻塞式对话框。 */
#undef assert
#define assert(expression)                                                   \
    do {                                                                     \
        if (!(expression)) {                                                 \
            fprintf(stderr,                                                  \
                    "assertion failed: %s (%s:%d)\n",                      \
                    #expression,                                             \
                    __FILE__,                                                \
                    __LINE__);                                               \
            exit(EXIT_FAILURE);                                              \
        }                                                                    \
    } while (0)
#endif

#define TEST_FLOAT_EPSILON       0.0001f /* 浮点断言允许的绝对误差。 */
#define TEST_TARGET_MAX_CM_S    60.0f    /* 测试使用的单轮目标速度上限。 */
#define TEST_PI                  3.14159265f /* 测试生成姿态样本使用的圆周率。 */

static float test_abs(float value)
{
    return (value < 0.0f) ? -value : value;
}

static float test_ball_accel_from_pipe_pitch(float pipe_pitch_deg)
{
    /* Independent bench convention: positive pipe pitch accelerates the
     * K230 coordinate in the negative direction. */
    return -100.0f * pipe_pitch_deg;
}

static void assert_float_near(float actual, float expected)
{
    if (test_abs(actual - expected) > TEST_FLOAT_EPSILON) {
        fprintf(stderr,
                "float mismatch: actual=%f expected=%f\n",
                (double) actual,
                (double) expected);
    }
    assert(test_abs(actual - expected) <= TEST_FLOAT_EPSILON);
}

static void assert_float_near_tolerance(float actual,
                                        float expected,
                                        float tolerance)
{
    if (test_abs(actual - expected) > tolerance) {
        fprintf(stderr,
                "float mismatch: actual=%f expected=%f tolerance=%f\n",
                (double) actual,
                (double) expected,
                (double) tolerance);
    }
    assert(test_abs(actual - expected) <= tolerance);
}

static void build_linear_mapping(float scale,
                                 float offset_deg,
                                 float mapping_pitch_deg[
                                     LINKAGE_KINEMATICS_CALIBRATION_POINT_COUNT])
{
    float model_pitch_deg;
    float servo_angle_deg;
    uint8_t index;

    for (index = 0U;
         index < LINKAGE_KINEMATICS_CALIBRATION_POINT_COUNT;
         index++) {
        assert(LinkageKinematics_GetCalibrationPoint(
                   index, &model_pitch_deg, &servo_angle_deg) != 0U);
        mapping_pitch_deg[index] = scale * model_pitch_deg + offset_deg;
    }
}

static void build_nonlinear_mapping(
    float mapping_pitch_deg[LINKAGE_KINEMATICS_CALIBRATION_POINT_COUNT])
{
    float model_pitch_deg;
    float servo_angle_deg;
    uint8_t index;

    for (index = 0U;
         index < LINKAGE_KINEMATICS_CALIBRATION_POINT_COUNT;
         index++) {
        assert(LinkageKinematics_GetCalibrationPoint(
                   index, &model_pitch_deg, &servo_angle_deg) != 0U);
        mapping_pitch_deg[index] =
            0.42f * model_pitch_deg +
            0.002f * model_pitch_deg *
            (model_pitch_deg + 10.0f) *
            (model_pitch_deg - 6.8f);
    }
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

    /* 上电位于 A 点横线时不得直接认为已经驶离起点。 */
    assert(LapStopDetector_Update(&detector, 0.0f, 1U, 0x0FU) == 0U);
    assert(detector.left_start_line == 0U);
    assert(detector.leave_clear_count == 0U);

    /* 驶离条件必须来自连续有效样本，无效 I2C 采样会清零确认计数。 */
    assert(LapStopDetector_Update(&detector, 0.0f, 1U, 0x03U) == 0U);
    assert(detector.leave_clear_count == 1U);
    assert(LapStopDetector_Update(&detector, 0.0f, 0U, 0x03U) == 0U);
    assert(detector.leave_clear_count == 0U);
    assert(LapStopDetector_Update(&detector, 0.0f, 1U, 0x03U) == 0U);
    assert(LapStopDetector_Update(&detector, 0.0f, 1U, 0x01U) == 0U);
    assert(detector.left_start_line != 0U);

    /* 未达到 400 cm 时，即使再次出现横线也不能触发返程完成。 */
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

    /* 4 路和 8 路黑线均满足门槛，连续两帧后确认返程通过 A 点。 */
    assert(LapStopDetector_Update(&detector, 0.0f, 1U, 0x0FU) == 0U);
    assert(LapStopDetector_Update(&detector, 0.0f, 1U, 0xFFU) != 0U);
    assert(detector.completed != 0U);
    assert(detector.full_black_count == 2U);
    assert(LapStopDetector_Update(&detector, 10.0f, 1U, 0x0FU) != 0U);
    assert_float_near(detector.distance_cm, 400.0f);

    LapStopDetector_Init(&detector);
    assert_float_near(detector.distance_cm, 0.0f);
    assert(detector.left_start_line == 0U);
    assert(detector.leave_clear_count == 0U);
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
        LinkageKinematics_GetNominalServoCenterDeg(),
        122.0f + LINKAGE_KINEMATICS_SERVO_LEVEL_TRIM_DEG,
        0.0002f);
    for (index = 0U; index < 13U; index++) {
        pitch_deg = (int8_t) index - 6;
        assert(LinkageKinematics_PipePitchToHornAngle(
                   (float) pitch_deg, &horn_angle_deg) != 0U);
        assert(LinkageKinematics_PipePitchToServoAngle(
                   (float) pitch_deg, &servo_angle_deg) != 0U);
        assert_float_near_tolerance(servo_angle_deg,
                                    expected_servo_angles[index] +
                                    LINKAGE_KINEMATICS_SERVO_LEVEL_TRIM_DEG,
                                    0.002f);
        assert(LinkageKinematics_ServoAngleToPipePitch(
                   servo_angle_deg, &recovered_pitch_deg) != 0U);
        assert_float_near_tolerance(recovered_pitch_deg,
                                    (float) pitch_deg,
                                    0.002f);
    }
    assert(LinkageKinematics_PipePitchToServoAngle(
               -10.0f, &servo_angle_deg) != 0U);
    assert_float_near_tolerance(
        servo_angle_deg,
        86.0168f + LINKAGE_KINEMATICS_SERVO_LEVEL_TRIM_DEG,
        0.002f);
    assert(LinkageKinematics_PipePitchToServoAngle(
               6.8f, &servo_angle_deg) != 0U);
    assert_float_near_tolerance(
        servo_angle_deg,
        178.7746f + LINKAGE_KINEMATICS_SERVO_LEVEL_TRIM_DEG,
        0.002f);
    assert(LinkageKinematics_PipePitchToServoAngle(-10.1f,
                                                    &servo_angle_deg) == 0U);
    assert(LinkageKinematics_PipePitchToServoAngle(6.9f,
                                                    &servo_angle_deg) == 0U);
    assert(LinkageKinematics_ServoAngleToPipePitch(
               90.0f + LINKAGE_KINEMATICS_SERVO_LEVEL_TRIM_DEG,
               &recovered_pitch_deg) != 0U);
    assert_float_near_tolerance(recovered_pitch_deg,
                                -8.4121f,
                                0.002f);
}

static void test_pipe_attitude_control(void)
{
    PipeAttitudeControl control;
    PipeAttitudeMeasurement measurement;
    PipeAttitudeOutput output;
    float raw_pitch_deg = 5.20f;
    float pitch_rad;
    uint32_t now_ms = 0U;
    uint16_t iteration;

    PipeAttitudeControl_Init(&control);
    PipeAttitudeControl_Update(&control, 0U, 0, 0.0f, &output);
    assert(output.calibration_valid == 0U);
    assert(output.attitude_valid == 0U);

    assert(PipeAttitudeControl_StartCalibration(
               &control, 0U, 2.0f) != 0U);
    PipeAttitudeControl_Update(&control, 0U, 0, 0.0f, &output);
    assert(output.status_changed != 0U);
    assert(output.calibration_event == PIPE_CALIBRATION_EVENT_READY);
    assert(output.servo_override_active != 0U);

    for (iteration = 0U; iteration < 400U; iteration++) {
        now_ms += 20U;
        pitch_rad = raw_pitch_deg * TEST_PI / 180.0f;
        measurement.valid = 1U;
        measurement.timestamp_ms = now_ms;
        measurement.accel_x_g = 0.0f;
        measurement.accel_y_g = sinf(pitch_rad);
        measurement.accel_z_g = cosf(pitch_rad);
        measurement.gyro_x_dps = 0.40f;
        measurement.gyro_y_dps = 0.0f;
        measurement.gyro_z_dps = 0.0f;
        PipeAttitudeControl_Update(&control,
                                   now_ms,
                                   &measurement,
                                   0.0f,
                                   &output);
        if (output.calibration_valid != 0U) {
            break;
        }
    }

    assert(output.calibration_valid != 0U);
    assert(output.calibration_event == PIPE_CALIBRATION_EVENT_SUCCESS);
    assert(output.imu_pitch_sign == -1);
    assert_float_near_tolerance(output.gyro_x_bias_dps, 0.40f, 0.001f);
    assert_float_near_tolerance(output.accel_pitch_zero_deg,
                                raw_pitch_deg,
                                0.01f);
    assert_float_near_tolerance(output.negative_pitch_deg, -5.5495f, 0.001f);
    assert_float_near_tolerance(output.center_pitch_deg, 0.0f, 0.001f);
    assert_float_near_tolerance(output.positive_pitch_deg, 5.5495f, 0.001f);
    assert_float_near_tolerance(output.model_scale, 1.0f, 0.001f);
    assert_float_near_tolerance(output.model_offset_deg, 0.0f, 0.001f);

    /* 原始IMU角减小1°，固定-1符号后应得到约+1°相对水管角。 */
    for (iteration = 0U; iteration < 80U; iteration++) {
        now_ms += 20U;
        pitch_rad = (raw_pitch_deg - 1.0f) * TEST_PI / 180.0f;
        measurement.timestamp_ms = now_ms;
        measurement.accel_y_g = sinf(pitch_rad);
        measurement.accel_z_g = cosf(pitch_rad);
        measurement.gyro_x_dps = 0.40f;
        PipeAttitudeControl_Update(&control,
                                   now_ms,
                                   &measurement,
                                   0.0f,
                                   &output);
    }
    assert(output.attitude_valid != 0U);
    assert_float_near_tolerance(output.pitch_deg, 1.0f, 0.08f);

    assert(PipeAttitudeControl_StartCalibration(
               &control, now_ms, 1.0f) != 0U);
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
    float mapping_pitch_deg[LINKAGE_KINEMATICS_CALIBRATION_POINT_COUNT];
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

    build_linear_mapping(1.0f, 0.0f, mapping_pitch_deg);
    BallBalanceControl_SetPipeCalibration(
        &control, 1U, 1.0f, 0.0f, mapping_pitch_deg);
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
                                0.03f);

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
    assert(output.trajectory_phase == BALL_BALANCE_PHASE_WAIT_PIPE);
    assert_float_near(output.desired_pipe_pitch_deg, 0.0f);

    /* A second stationary frame confirms stiction and starts the smooth
     * toward-target ramp. */
    measurement.sequence = 2U;
    now_ms += 20U;
    attitude.timestamp_ms = now_ms;
    BallBalanceControl_Update(&control,
                              now_ms,
                              &measurement,
                              &attitude,
                              &output);
    assert(output.desired_pipe_pitch_deg > 0.0f);
    assert(output.desired_model_pitch_deg > 0.0f);
    assert(output.desired_angle_deg > neutral_angle_deg);
    assert(output.command_angle_deg > neutral_angle_deg);
    assert(output.command_angle_deg >= -5.5495f);
    assert(output.command_angle_deg <= 5.5495f);

    /* 线性前馈之外，IMU姿态P/D应修正水管跟随误差并抑制角速度。 */
    attitude.pitch_deg = -0.5f;
    attitude.pitch_rate_dps = 6.0f;
    now_ms += 20U;
    attitude.timestamp_ms = now_ms;
    BallBalanceControl_Update(&control,
                              now_ms,
                              0,
                              &attitude,
                              &output);
    assert(output.pipe_p_term_deg > 0.0f);
    assert(output.pipe_d_term_deg < 0.0f);
    assert(test_abs(output.desired_angle_deg -
                    output.servo_feedforward_deg) > 0.01f);
    attitude.pitch_deg = 0.0f;
    attitude.pitch_rate_dps = 0.0f;

    /* 跳变帧使用新序号，进入跳变抑制逻辑而不是被同序号去重丢弃。 */
    measurement.sequence = 3U;
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

    /* Once the ball is moving, remove the acceleration pulse first.  The
     * trajectory must return the pipe to level before any brake pulse. */
    /* 持续跳变 400 ms 仍偏离才判定为真实新位置并重采样。 */
    measurement.sequence = 4U;
    now_ms += 400U;
    attitude.timestamp_ms = now_ms;
    BallBalanceControl_Update(&control,
                              now_ms,
                              &measurement,
                              &attitude,
                              &output);
    assert(output.measurement_accepted != 0U);
    /* 重采样时速度有意清零，避免假速度；位置为新位置并进入 MOVING。 */
    assert(output.velocity_mm_s == 0.0f);
    assert(output.predicted_position_mm < 0.0f);
    assert(output.trajectory_phase == BALL_BALANCE_PHASE_MOVING);
    /* 后续帧继续向负方向移动，产生真实负速度；水管先回平再刹车。 */
    measurement.sequence = 5U;
    measurement.position_d10_mm = -600;
    now_ms += 20U;
    attitude.timestamp_ms = now_ms;
    BallBalanceControl_Update(&control,
                              now_ms,
                              &measurement,
                              &attitude,
                              &output);
    assert(output.velocity_mm_s < 0.0f);
    assert(output.trajectory_phase == BALL_BALANCE_PHASE_MOVING);
    assert(output.braking_active != 0U);
    assert(output.d_term_deg < 0.0f);
    assert(output.desired_pipe_pitch_deg < 0.0f);

    BallBalanceControl_SetTargetMm(&control, 200.0f);
    now_ms += 20U;
    attitude.timestamp_ms = now_ms;
    BallBalanceControl_Update(&control,
                              now_ms,
                              0,
                              &attitude,
                              &output);
    assert_float_near(output.target_mm, 100.0f);

    now_ms += 360U;
    attitude.timestamp_ms = now_ms;
    BallBalanceControl_Update(&control,
                              now_ms,
                              0,
                              &attitude,
                              &output);
    assert(output.timed_out == 0U);
    assert(output.tracking_valid != 0U);

    now_ms += 650U;
    attitude.timestamp_ms = now_ms;
    BallBalanceControl_Update(&control,
                              now_ms,
                              0,
                              &attitude,
                              &output);
    assert(output.timed_out != 0U);
    assert(output.tracking_valid == 0U);
    /* 视觉超时后水管回中，允许 IMU 内环修正的微小残留。 */
    assert_float_near_tolerance(output.desired_pipe_pitch_deg, 0.0f, 0.3f);

    now_ms += 120U;
    BallBalanceControl_Update(&control, now_ms, 0, 0, &output);
    /* 去抖窗口内（<150 ms）姿态不新鲜先保持闭环让内环自纠，不立即判故障。 */
    assert(output.enabled != 0U);
    assert(output.imu_fault == 0U);
    assert_float_near_tolerance(output.desired_angle_deg,
                                neutral_angle_deg,
                                0.03f);
    /* 持续无姿态数据超过 IMU 故障去抖时长（150 ms）后关闭平衡并判故障。 */
    now_ms += 150U;
    BallBalanceControl_Update(&control, now_ms, 0, 0, &output);
    assert(output.enabled == 0U);
    assert(output.imu_fault != 0U);

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
    attitude.pitch_deg = 9.0f;
    attitude.pitch_rate_dps = 0.0f;
    BallBalanceControl_Update(&control,
                              now_ms,
                              0,
                              &attitude,
                              &output);
    /* 去抖窗口内（<150 ms）姿态超限先保持闭环让内环自纠，不立即判故障。 */
    assert(output.enabled != 0U);
    assert(output.imu_fault == 0U);
    now_ms += 160U;
    attitude.timestamp_ms = now_ms;
    BallBalanceControl_Update(&control,
                              now_ms,
                              0,
                              &attitude,
                              &output);
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
    float mapping_pitch_deg[LINKAGE_KINEMATICS_CALIBRATION_POINT_COUNT];
    uint32_t now_ms = 0U;
    uint8_t index;

    BallBalanceControl_Init(&control);
    build_linear_mapping(0.42f, 0.38f, mapping_pitch_deg);
    BallBalanceControl_SetPipeCalibration(
        &control, 1U, 0.42f, 0.38f, mapping_pitch_deg);
    BallBalanceControl_Update(&control, now_ms, 0, 0, &output);
    assert(output.mapping_valid != 0U);
    assert_float_near_tolerance(
        output.minimum_pipe_pitch_deg, -5.5495f, 0.001f);
    assert_float_near_tolerance(
        output.maximum_pipe_pitch_deg, 5.5495f, 0.001f);

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
    assert(output.trajectory_phase == BALL_BALANCE_PHASE_WAIT_PIPE);
    measurement.sequence = 2U;
    now_ms += 20U;
    attitude.timestamp_ms = now_ms;
    BallBalanceControl_Update(&control,
                              now_ms,
                              &measurement,
                              &attitude,
                              &output);
    assert_float_near_tolerance(
        output.desired_pipe_pitch_deg, 2.15f, 0.01f);
    assert(output.mapping_valid != 0U);
    assert(output.mapping_fault == 0U);
    assert(output.desired_model_pitch_deg <= 6.8f);

    now_ms += 220U;
    measurement.position_d10_mm = -1000;
    measurement.sequence = 3U;
    attitude.timestamp_ms = now_ms;
    BallBalanceControl_Update(&control,
                              now_ms,
                              &measurement,
                              &attitude,
                              &output);
    assert(output.mapping_valid != 0U);
    assert(output.mapping_fault == 0U);
    /* 跳变帧先被抑制，400 ms 重采样锁定新位置后产生负俯仰。 */
    measurement.sequence = 4U;
    now_ms += 400U;
    attitude.timestamp_ms = now_ms;
    BallBalanceControl_Update(&control,
                              now_ms,
                              &measurement,
                              &attitude,
                              &output);
    assert(output.measurement_accepted != 0U);
    assert(output.desired_pipe_pitch_deg < 0.0f);
    assert(output.trajectory_phase == BALL_BALANCE_PHASE_MOVING);
    assert(output.desired_model_pitch_deg >= -10.0f);
}

static void test_ball_balance_friction_compensation(void)
{
    BallBalanceControl control;
    BallBalanceMeasurement measurement;
    BallBalanceAttitude attitude;
    BallBalanceOutput output;
    float mapping_pitch_deg[LINKAGE_KINEMATICS_CALIBRATION_POINT_COUNT];
    uint32_t now_ms = 0U;
    uint8_t index;

    build_linear_mapping(1.0f, 0.0f, mapping_pitch_deg);
    BallBalanceControl_Init(&control);
    BallBalanceControl_SetPipeCalibration(
        &control, 1U, 1.0f, 0.0f, mapping_pitch_deg);
    BallBalanceControl_SetEnabled(&control, 1U, now_ms);

    attitude.valid = 1U;
    attitude.pitch_deg = 0.0f;
    attitude.pitch_rate_dps = 0.0f;
    for (index = 0U; index < 30U; index++) {
        now_ms += 20U;
        attitude.timestamp_ms = now_ms;
        BallBalanceControl_Update(&control, now_ms, 0, &attitude, &output);
    }
    assert(output.enabled != 0U);

    BallBalanceControl_SetPidGains(&control, 0.0f, 0.020f, 0.0f);
    measurement.valid = 1U;
    measurement.position_d10_mm = 150;
    measurement.quality = 80U;
    for (index = 1U; index <= 20U; index++) {
        measurement.sequence = index;
        now_ms += 100U;
        attitude.timestamp_ms = now_ms;
        BallBalanceControl_Update(&control,
                                  now_ms,
                                  &measurement,
                                  &attitude,
                                  &output);
    }
    /* 距离目标15毫米且静止时启用Ki，逐步建立正向水管角补偿。 */
    assert(output.integral_active != 0U);
    assert(output.integral_mm_s > 0.0f);
    assert(output.i_term_deg > 0.0f);
    assert(output.i_term_deg <= 0.80f);
    assert(output.desired_pipe_pitch_deg > 0.0f);
    assert_float_near_tolerance(output.position_kp_deg_per_mm,
                                0.0f,
                                0.0001f);
    assert_float_near_tolerance(output.position_ki_deg_per_mm_s,
                                0.020f,
                                0.0001f);
    assert_float_near_tolerance(output.position_kd_deg_per_mm_s,
                                0.0f,
                                0.0001f);

    /* 回到目标死区（±4毫米）内后立即禁用并清除积分。 */
    measurement.position_d10_mm = 30;
    for (index = 21U; index <= 26U; index++) {
        measurement.sequence = index;
        now_ms += 100U;
        attitude.timestamp_ms = now_ms;
        BallBalanceControl_Update(&control,
                                  now_ms,
                                  &measurement,
                                  &attitude,
                                  &output);
    }
    assert(test_abs(output.error_mm) <= 10.0f);
    assert(output.integral_active == 0U);
    assert_float_near_tolerance(output.integral_mm_s, 0.0f, 0.001f);
    assert_float_near_tolerance(output.i_term_deg, 0.0f, 0.001f);

    /* 离开20毫米积分区后也保持积分清零，避免远距离控制饱和。 */
    measurement.position_d10_mm = 1000;
    measurement.sequence = 27U;
    now_ms += 100U;
    attitude.timestamp_ms = now_ms;
    BallBalanceControl_Update(&control,
                              now_ms,
                              &measurement,
                              &attitude,
                              &output);
    assert(output.integral_active == 0U);
    assert_float_near_tolerance(output.integral_mm_s, 0.0f, 0.001f);
    assert_float_near_tolerance(output.i_term_deg, 0.0f, 0.001f);

    BallBalanceControl_Init(&control);
    BallBalanceControl_SetPipeCalibration(
        &control, 1U, 1.0f, 0.0f, mapping_pitch_deg);
    BallBalanceControl_SetEnabled(&control, 1U, now_ms);
    for (index = 0U; index < 30U; index++) {
        now_ms += 20U;
        attitude.timestamp_ms = now_ms;
        BallBalanceControl_Update(&control, now_ms, 0, &attitude, &output);
    }
    assert(output.enabled != 0U);

    BallBalanceControl_SetPidGains(&control, 0.025f, 0.015f, 0.040f);
    /* 小球静止在目标正向3毫米时应进入HOLD，且P/I均不产生追逐动作。 */
    measurement.position_d10_mm = 30;
    measurement.quality = 80U;
    for (index = 1U; index <= 35U; index++) {
        measurement.sequence = index;
        now_ms += 20U;
        attitude.timestamp_ms = now_ms;
        BallBalanceControl_Update(&control,
                                  now_ms,
                                  &measurement,
                                  &attitude,
                                  &output);
    }
    assert(output.trajectory_phase == BALL_BALANCE_PHASE_HOLD);
    assert(output.integral_active == 0U);
    assert_float_near_tolerance(output.p_term_deg, 0.0f, 0.001f);
    assert_float_near_tolerance(output.i_term_deg, 0.0f, 0.001f);
    assert_float_near_tolerance(
        output.desired_pipe_pitch_deg, 0.0f, 0.001f);
}

static void test_ball_balance_motion_classification(void)
{
    BallBalanceControl control;
    BallBalanceMeasurement measurement;
    BallBalanceAttitude attitude;
    BallBalanceOutput output;
    float mapping_pitch_deg[LINKAGE_KINEMATICS_CALIBRATION_POINT_COUNT];
    float fifth_pitch_deg = 0.0f;
    float initial_brake_abs;
    float initial_pitch_abs;
    uint32_t now_ms = 0U;
    uint8_t index;

    build_linear_mapping(1.0f, 0.0f, mapping_pitch_deg);
    BallBalanceControl_Init(&control);
    BallBalanceControl_SetPipeCalibration(
        &control, 1U, 1.0f, 0.0f, mapping_pitch_deg);
    BallBalanceControl_SetEnabled(&control, 1U, now_ms);
    attitude.valid = 1U;
    attitude.pitch_deg = 0.0f;
    attitude.pitch_rate_dps = 0.0f;
    for (index = 0U; index < 12U; index++) {
        now_ms += 20U;
        attitude.timestamp_ms = now_ms;
        BallBalanceControl_Update(&control, now_ms, 0, &attitude, &output);
    }
    assert(output.enabled != 0U);

    /* A stationary ball receives one continuous ramp.  Every failed stage
     * raises the same-direction target without returning the pipe to level. */
    measurement.valid = 1U;
    measurement.position_d10_mm = -500;
    measurement.quality = 80U;
    measurement.sequence = 1U;
    now_ms += 20U;
    attitude.timestamp_ms = now_ms;
    BallBalanceControl_Update(
        &control, now_ms, &measurement, &attitude, &output);
    measurement.sequence = 2U;
    now_ms += 100U;
    attitude.timestamp_ms = now_ms;
    BallBalanceControl_Update(
        &control, now_ms, &measurement, &attitude, &output);
    assert(output.trajectory_phase == BALL_BALANCE_PHASE_BREAKAWAY);
    assert(output.desired_pipe_pitch_deg < 0.0f);
    initial_pitch_abs = test_abs(output.desired_pipe_pitch_deg);
    /* A short YOLO-validity gap must hold the breakaway angle rather than
     * level the pipe and restart the two-frame stationary classification. */
    now_ms += 700U;
    attitude.timestamp_ms = now_ms;
    BallBalanceControl_Update(
        &control, now_ms, 0, &attitude, &output);
    assert(output.timed_out == 0U);
    assert(output.trajectory_phase == BALL_BALANCE_PHASE_BREAKAWAY);
    assert_float_near_tolerance(
        test_abs(output.desired_pipe_pitch_deg), initial_pitch_abs, 0.001f);
    for (index = 1U; index <= 7U; index++) {
        if (index == 1U) {
            /* A tiny negative-side outward jitter must not be mistaken for
             * a real high-speed side turnaround. */
            measurement.position_d10_mm = -501;
        }
        measurement.sequence++;
        now_ms += 600U;
        attitude.timestamp_ms = now_ms;
        BallBalanceControl_Update(
            &control, now_ms, &measurement, &attitude, &output);
        assert(output.trajectory_phase == BALL_BALANCE_PHASE_BREAKAWAY);
        assert(output.kick_attempt_count == index);
        assert(output.desired_pipe_pitch_deg < 0.0f);
        if (index == 5U) {
            fifth_pitch_deg = test_abs(output.desired_pipe_pitch_deg);
        }
    }
    assert(test_abs(output.desired_pipe_pitch_deg) > initial_pitch_abs);
    assert(test_abs(output.desired_pipe_pitch_deg) > fifth_pitch_deg);
    assert(output.kick_attempt_count == 7U);
    assert(output.error_mm * test_ball_accel_from_pipe_pitch(
               output.desired_pipe_pitch_deg) < 0.0f);

    /* Starting at zero and being pushed rapidly outward must immediately
     * command a restoring pitch instead of waiting for the ball to stop. */
    BallBalanceControl_Init(&control);
    BallBalanceControl_SetPipeCalibration(
        &control, 1U, 1.0f, 0.0f, mapping_pitch_deg);
    BallBalanceControl_SetEnabled(&control, 1U, now_ms);
    for (index = 0U; index < 12U; index++) {
        now_ms += 20U;
        attitude.timestamp_ms = now_ms;
        BallBalanceControl_Update(&control, now_ms, 0, &attitude, &output);
    }
    measurement.position_d10_mm = 0;
    measurement.sequence = 1U;
    now_ms += 20U;
    attitude.timestamp_ms = now_ms;
    BallBalanceControl_Update(
        &control, now_ms, &measurement, &attitude, &output);
    measurement.sequence = 2U;
    now_ms += 100U;
    attitude.timestamp_ms = now_ms;
    BallBalanceControl_Update(
        &control, now_ms, &measurement, &attitude, &output);
    assert(output.trajectory_phase == BALL_BALANCE_PHASE_HOLD);
    /* Even a large apparent displacement cannot move the pipe during the
     * 500 ms zero-stop freeze. */
    measurement.position_d10_mm = 300;
    measurement.sequence = 3U;
    now_ms += 100U;
    attitude.timestamp_ms = now_ms;
    BallBalanceControl_Update(
        &control, now_ms, &measurement, &attitude, &output);
    assert(output.trajectory_phase == BALL_BALANCE_PHASE_HOLD);
    /* 冻结期水管保持在进入 HOLD 时的姿态附近，允许微小残留。 */
    assert_float_near_tolerance(output.desired_pipe_pitch_deg, 0.0f, 0.3f);
    measurement.sequence = 4U;
    now_ms += 200U;
    attitude.timestamp_ms = now_ms;
    BallBalanceControl_Update(
        &control, now_ms, &measurement, &attitude, &output);
    assert(output.trajectory_phase == BALL_BALANCE_PHASE_HOLD);
    measurement.sequence = 5U;
    now_ms += 200U;
    attitude.timestamp_ms = now_ms;
    BallBalanceControl_Update(
        &control, now_ms, &measurement, &attitude, &output);
    assert(output.trajectory_phase == BALL_BALANCE_PHASE_HOLD);
    /* 冻结结束后球保持 +30 mm 静止，控制器破静摩擦推动，水管回正。 */
    measurement.sequence = 6U;
    now_ms += 100U;
    attitude.timestamp_ms = now_ms;
    BallBalanceControl_Update(
        &control, now_ms, &measurement, &attitude, &output);
    assert(output.desired_pipe_pitch_deg > 0.0f);
    assert(output.error_mm * test_ball_accel_from_pipe_pitch(
               output.desired_pipe_pitch_deg) < 0.0f);
    /* 球随后真正向外移动，产生正速度；第二帧运动确认后进入 MOVING。 */
    measurement.sequence = 7U;
    measurement.position_d10_mm = 400;
    now_ms += 100U;
    attitude.timestamp_ms = now_ms;
    BallBalanceControl_Update(
        &control, now_ms, &measurement, &attitude, &output);
    assert(output.velocity_mm_s > 0.0f);
    assert(output.desired_pipe_pitch_deg > 0.0f);
    assert(output.velocity_mm_s * test_ball_accel_from_pipe_pitch(
               output.desired_pipe_pitch_deg) < 0.0f);
    measurement.sequence = 8U;
    measurement.position_d10_mm = 500;
    now_ms += 100U;
    attitude.timestamp_ms = now_ms;
    BallBalanceControl_Update(
        &control, now_ms, &measurement, &attitude, &output);
    assert(output.trajectory_phase == BALL_BALANCE_PHASE_MOVING);
    /* A confirmed high-speed outward run receives one bounded 500 ms freeze
     * when its speed fades at the side turning point. */
    BallBalanceControl_Init(&control);
    BallBalanceControl_SetPipeCalibration(
        &control, 1U, 1.0f, 0.0f, mapping_pitch_deg);
    BallBalanceControl_SetEnabled(&control, 1U, now_ms);
    for (index = 0U; index < 30U; index++) {
        now_ms += 20U;
        attitude.timestamp_ms = now_ms;
        BallBalanceControl_Update(&control, now_ms, 0, &attitude, &output);
    }
    assert(output.enabled != 0U);
    assert(output.pipe_settled != 0U);
    /* 球静止在目标0位置两帧，进入零点保持冻结。 */
    measurement.position_d10_mm = 0;
    measurement.sequence = 1U;
    now_ms += 20U;
    attitude.timestamp_ms = now_ms;
    BallBalanceControl_Update(
        &control, now_ms, &measurement, &attitude, &output);
    measurement.sequence = 2U;
    now_ms += 20U;
    attitude.timestamp_ms = now_ms;
    BallBalanceControl_Update(
        &control, now_ms, &measurement, &attitude, &output);
    assert(output.trajectory_phase == BALL_BALANCE_PHASE_HOLD);
    /* 球被推向外侧 +30 mm，冻结期水管保持不动（跳变被抑制）。 */
    measurement.position_d10_mm = 300;
    measurement.sequence = 3U;
    now_ms += 20U;
    attitude.timestamp_ms = now_ms;
    BallBalanceControl_Update(
        &control, now_ms, &measurement, &attitude, &output);
    measurement.sequence = 4U;
    now_ms += 20U;
    attitude.timestamp_ms = now_ms;
    BallBalanceControl_Update(
        &control, now_ms, &measurement, &attitude, &output);
    assert(output.trajectory_phase == BALL_BALANCE_PHASE_HOLD);
    /* 持续跳变 400 ms 仍偏离，重采样锁定新位置 +30 mm。 */
    measurement.sequence = 5U;
    now_ms += 400U;
    attitude.timestamp_ms = now_ms;
    BallBalanceControl_Update(
        &control, now_ms, &measurement, &attitude, &output);
    assert(output.measurement_accepted != 0U);
    /* 球以每帧 +3 mm 连续外移（增量小于跳变阈值），进入 MOVING 并
     * 确认向外运动。 */
    for (index = 6U; index <= 30U; index++) {
        measurement.sequence = index;
        measurement.position_d10_mm = (int32_t) (300 + (index - 5U) * 30);
        now_ms += 20U;
        attitude.timestamp_ms = now_ms;
        BallBalanceControl_Update(
            &control, now_ms, &measurement, &attitude, &output);
    }
    assert(control.outward_motion_seen != 0U);
    assert(output.trajectory_phase == BALL_BALANCE_PHASE_MOVING);
    /* 球在端部静止，速度衰减触发一次 500 ms 冻结。 */
    for (index = 31U;
         (index <= 50U) && (control.turnaround_freeze_active == 0U);
         index++) {
        measurement.sequence = index;
        now_ms += 100U;
        attitude.timestamp_ms = now_ms;
        BallBalanceControl_Update(
            &control, now_ms, &measurement, &attitude, &output);
    }
    assert(output.velocity_mm_s > 0.0f);
    assert(output.velocity_mm_s <= 12.0f);
    assert(control.turnaround_freeze_active != 0U);
    assert(output.trajectory_phase == BALL_BALANCE_PHASE_MOVING);
    assert_float_near_tolerance(
        output.desired_pipe_pitch_deg,
        output.command_pipe_pitch_deg,
        0.001f);
    assert_float_near_tolerance(output.pipe_command_rate_dps, 0.0f, 0.001f);
    assert(output.kick_attempt_count == 0U);
    for (index = 0U; index < 2U; index++) {
        measurement.sequence++;
        now_ms += 250U;
        attitude.timestamp_ms = now_ms;
        BallBalanceControl_Update(
            &control, now_ms, &measurement, &attitude, &output);
    }
    assert(control.turnaround_freeze_active == 0U);

    /* A K230 frame may jump completely across the +/-10 mm window.  Inward
     * motion must already be braking by speed alone; the explicit crossing
     * event then clears stale phase modifiers on the same accepted frame. */
    BallBalanceControl_Init(&control);
    BallBalanceControl_SetPipeCalibration(
        &control, 1U, 1.0f, 0.0f, mapping_pitch_deg);
    BallBalanceControl_SetEnabled(&control, 1U, now_ms);
    for (index = 0U; index < 12U; index++) {
        now_ms += 20U;
        attitude.timestamp_ms = now_ms;
        BallBalanceControl_Update(&control, now_ms, 0, &attitude, &output);
    }
    measurement.position_d10_mm = 300;
    measurement.sequence = 1U;
    now_ms += 20U;
    attitude.timestamp_ms = now_ms;
    BallBalanceControl_Update(
        &control, now_ms, &measurement, &attitude, &output);
    measurement.position_d10_mm = 300;
    measurement.sequence = 2U;
    now_ms += 100U;
    attitude.timestamp_ms = now_ms;
    BallBalanceControl_Update(
        &control, now_ms, &measurement, &attitude, &output);
    assert(output.trajectory_phase == BALL_BALANCE_PHASE_BREAKAWAY);
    measurement.position_d10_mm = 200;
    measurement.sequence = 3U;
    now_ms += 100U;
    attitude.timestamp_ms = now_ms;
    BallBalanceControl_Update(
        &control, now_ms, &measurement, &attitude, &output);
    assert(output.trajectory_phase == BALL_BALANCE_PHASE_BREAKAWAY);
    measurement.position_d10_mm = 100;
    measurement.sequence = 4U;
    now_ms += 100U;
    attitude.timestamp_ms = now_ms;
    BallBalanceControl_Update(
        &control, now_ms, &measurement, &attitude, &output);
    assert(output.trajectory_phase == BALL_BALANCE_PHASE_MOVING);
    assert(output.desired_pipe_pitch_deg < 0.0f);
    /* 回程进入目标 15 mm 内时启用回程制动 P，与运动方向反向制动。 */
    assert(output.p_term_deg < 0.0f);
    assert(output.velocity_mm_s * test_ball_accel_from_pipe_pitch(
               output.desired_pipe_pitch_deg) < 0.0f);
    measurement.position_d10_mm = -400;
    measurement.sequence = 5U;
    now_ms += 400U;
    attitude.timestamp_ms = now_ms;
    BallBalanceControl_Update(
        &control, now_ms, &measurement, &attitude, &output);
    assert(output.measurement_accepted != 0U);
    /* 跳变帧先被抑制，400 ms 重采样锁定新位置后触发跨零事件。 */
    assert(output.zero_crossed != 0U);
    assert(output.desired_pipe_pitch_deg < 0.0f);
    assert(output.kick_attempt_count == 0U);
    assert(output.turnaround_freeze_active == 0U);
    /* 重采样帧速度清零，下一帧球继续向负方向移动后验证速度-加速度反向。 */
    measurement.position_d10_mm = -500;
    measurement.sequence = 6U;
    now_ms += 100U;
    attitude.timestamp_ms = now_ms;
    BallBalanceControl_Update(
        &control, now_ms, &measurement, &attitude, &output);
    assert(output.velocity_mm_s * test_ball_accel_from_pipe_pitch(
               output.desired_pipe_pitch_deg) < 0.0f);

    /* With the normal nonzero Kp, inward motion still uses pure velocity
     * damping: pitch opposes velocity and tapers only as speed falls. */
    BallBalanceControl_Init(&control);
    BallBalanceControl_SetPipeCalibration(
        &control, 1U, 1.0f, 0.0f, mapping_pitch_deg);
    BallBalanceControl_SetEnabled(&control, 1U, now_ms);
    for (index = 0U; index < 12U; index++) {
        now_ms += 20U;
        attitude.timestamp_ms = now_ms;
        BallBalanceControl_Update(&control, now_ms, 0, &attitude, &output);
    }
    measurement.position_d10_mm = 500;
    measurement.sequence = 1U;
    now_ms += 20U;
    attitude.timestamp_ms = now_ms;
    BallBalanceControl_Update(
        &control, now_ms, &measurement, &attitude, &output);
    measurement.position_d10_mm = 500;
    measurement.sequence = 2U;
    now_ms += 100U;
    attitude.timestamp_ms = now_ms;
    BallBalanceControl_Update(
        &control, now_ms, &measurement, &attitude, &output);
    assert(output.trajectory_phase == BALL_BALANCE_PHASE_BREAKAWAY);
    measurement.position_d10_mm = 400;
    measurement.sequence = 3U;
    now_ms += 100U;
    attitude.timestamp_ms = now_ms;
    BallBalanceControl_Update(
        &control, now_ms, &measurement, &attitude, &output);
    assert(output.trajectory_phase == BALL_BALANCE_PHASE_BREAKAWAY);
    measurement.position_d10_mm = 300;
    measurement.sequence = 4U;
    now_ms += 100U;
    attitude.timestamp_ms = now_ms;
    BallBalanceControl_Update(
        &control, now_ms, &measurement, &attitude, &output);
    assert(output.trajectory_phase == BALL_BALANCE_PHASE_MOVING);
    assert_float_near_tolerance(output.p_term_deg, 0.0f, 0.001f);
    initial_brake_abs = test_abs(output.desired_pipe_pitch_deg);
    /* Keep the ball far from zero while the filtered inward speed drops; the
     * requested brake angle must decrease solely because of that speed. */
    measurement.position_d10_mm = 350;
    measurement.sequence = 5U;
    now_ms += 100U;
    attitude.timestamp_ms = now_ms;
    BallBalanceControl_Update(
        &control, now_ms, &measurement, &attitude, &output);
    assert(output.trajectory_phase == BALL_BALANCE_PHASE_MOVING);
    assert_float_near_tolerance(output.p_term_deg, 0.0f, 0.001f);
    assert(test_abs(output.desired_pipe_pitch_deg) < initial_brake_abs);
    /* 球小幅递增外移（增量小于跳变阈值）产生正速度。 */
    measurement.position_d10_mm = 400;
    measurement.sequence = 6U;
    now_ms += 100U;
    attitude.timestamp_ms = now_ms;
    BallBalanceControl_Update(
        &control, now_ms, &measurement, &attitude, &output);
    measurement.position_d10_mm = 450;
    measurement.sequence = 7U;
    now_ms += 100U;
    attitude.timestamp_ms = now_ms;
    BallBalanceControl_Update(
        &control, now_ms, &measurement, &attitude, &output);
    assert(output.velocity_mm_s > 0.0f);
    assert(output.trajectory_phase == BALL_BALANCE_PHASE_MOVING);
    assert(output.desired_pipe_pitch_deg > 0.0f);
    assert(output.braking_active != 0U);
}

static void test_ball_balance_endpoint_integral(void)
{
    BallBalanceControl control;
    BallBalanceMeasurement measurement;
    BallBalanceAttitude attitude;
    BallBalanceOutput output;
    float mapping_pitch_deg[LINKAGE_KINEMATICS_CALIBRATION_POINT_COUNT];
    uint32_t now_ms = 0U;
    uint8_t index;

    build_linear_mapping(1.0f, 0.0f, mapping_pitch_deg);
    BallBalanceControl_Init(&control);
    BallBalanceControl_SetPipeCalibration(
        &control, 1U, 1.0f, 0.0f, mapping_pitch_deg);
    BallBalanceControl_SetEnabled(&control, 1U, now_ms);

    attitude.valid = 1U;
    attitude.pitch_deg = 0.0f;
    attitude.pitch_rate_dps = 0.0f;
    for (index = 0U; index < 30U; index++) {
        now_ms += 20U;
        attitude.timestamp_ms = now_ms;
        BallBalanceControl_Update(&control, now_ms, 0, &attitude, &output);
    }
    assert(output.enabled != 0U);

    BallBalanceControl_SetTargetMm(&control, -50.0f);
    BallBalanceControl_SetEndpointHoldEnabled(&control, 1U);
    measurement.valid = 1U;
    measurement.position_d10_mm = -480;
    measurement.quality = 80U;
    for (index = 1U; index <= 12U; index++) {
        measurement.sequence = index;
        now_ms += 100U;
        attitude.timestamp_ms = now_ms;
        BallBalanceControl_SetPidGains(
            &control,
            BALL_BALANCE_DEFAULT_KP_DEG_PER_MM,
            BALL_BALANCE_DEFAULT_KI_DEG_PER_MM_S,
            BALL_BALANCE_DEFAULT_KD_DEG_PER_MM_S);
        BallBalanceControl_Update(&control,
                                  now_ms,
                                  &measurement,
                                  &attitude,
                                  &output);
    }

    /* 第三题端点误差在死区（2毫米）内时不得启用积分。 */
    assert(output.endpoint_hold_enabled != 0U);
    assert(output.integral_active == 0U);
    assert_float_near_tolerance(output.i_term_deg, 0.0f, 0.001f);

    measurement.position_d10_mm = -350;
    for (index = 13U; index <= 24U; index++) {
        measurement.sequence = index;
        now_ms += 100U;
        attitude.timestamp_ms = now_ms;
        BallBalanceControl_Update(&control,
                                  now_ms,
                                  &measurement,
                                  &attitude,
                                  &output);
    }
    assert(output.integral_active != 0U);
    assert(output.i_term_deg > 0.01f);
    assert(output.desired_pipe_pitch_deg > 0.0f);

    BallBalanceControl_SetEndpointHoldEnabled(&control, 0U);
    measurement.sequence++;
    now_ms += 100U;
    attitude.timestamp_ms = now_ms;
    BallBalanceControl_Update(&control,
                              now_ms,
                              &measurement,
                              &attitude,
                              &output);
    assert(output.endpoint_hold_enabled == 0U);
    assert(output.integral_active != 0U);
    assert(output.i_term_deg > 0.0f);
    assert(output.i_term_deg < 0.05f);
    assert(output.desired_pipe_pitch_deg > 0.0f);
}

static void test_ball_shuttle_control(void)
{
    BallShuttleControl control;
    BallShuttleConfig config;
    BallShuttleEvent event;

    BallShuttleControl_Init(&control);
    assert(control.state == BALL_SHUTTLE_STATE_IDLE);
    assert(BallShuttleControl_IsRunning(&control) == 0U);
    assert(BallShuttleControl_IsHolding(&control) == 0U);

    config.angle_magnitude_deg[0] = 3.0f;
    config.angle_magnitude_deg[1] = 4.0f;
    config.angle_magnitude_deg[2] = 2.0f;
    config.duration_ms[0] = 100U;
    config.duration_ms[1] = 200U;
    config.duration_ms[2] = 150U;
    assert(BallShuttleControl_SetConfig(&control, &config) != 0U);

    config.duration_ms[0] = 6000U;
    assert(BallShuttleControl_SetConfig(&control, &config) == 0U);
    config.duration_ms[0] = 100U;
    config.duration_ms[1] = 200U;
    config.duration_ms[2] = 150U;
    assert(BallShuttleControl_SetConfig(&control, &config) != 0U);
    config.angle_magnitude_deg[2] = 6.0f;
    assert(BallShuttleControl_SetConfig(&control, &config) == 0U);
    config.angle_magnitude_deg[2] = 2.0f;
    assert(BallShuttleControl_SetConfig(&control, &config) != 0U);
    config.duration_ms[1] = 4800U;
    assert(BallShuttleControl_SetConfig(&control, &config) == 0U);

    config.duration_ms[1] = 200U;
    assert(BallShuttleControl_SetConfig(&control, &config) != 0U);
    assert(BallShuttleControl_Start(&control, 0U) != 0U);
    assert(BallShuttleControl_IsRunning(&control) != 0U);
    assert_float_near_tolerance(
        BallShuttleControl_GetTargetPipePitchDeg(&control), -3.0f, 0.001f);
    assert(BallShuttleControl_SetConfig(&control, &config) == 0U);

    event = BallShuttleControl_Update(&control, 50U);
    assert(event == BALL_SHUTTLE_EVENT_NONE);
    assert(control.state == BALL_SHUTTLE_STATE_PHASE_1_NEGATIVE);

    event = BallShuttleControl_Update(&control, 100U);
    assert(event == BALL_SHUTTLE_EVENT_PHASE_2);
    assert(control.state == BALL_SHUTTLE_STATE_PHASE_2_POSITIVE);
    assert_float_near_tolerance(
        BallShuttleControl_GetTargetPipePitchDeg(&control), 4.0f, 0.001f);

    event = BallShuttleControl_Update(&control, 200U);
    assert(event == BALL_SHUTTLE_EVENT_NONE);
    assert(control.state == BALL_SHUTTLE_STATE_PHASE_2_POSITIVE);

    event = BallShuttleControl_Update(&control, 300U);
    assert(event == BALL_SHUTTLE_EVENT_PHASE_3);
    assert(control.state == BALL_SHUTTLE_STATE_PHASE_3_NEGATIVE);
    assert_float_near_tolerance(
        BallShuttleControl_GetTargetPipePitchDeg(&control), -2.0f, 0.001f);

    event = BallShuttleControl_Update(&control, 400U);
    assert(event == BALL_SHUTTLE_EVENT_NONE);
    assert(control.state == BALL_SHUTTLE_STATE_PHASE_3_NEGATIVE);

    event = BallShuttleControl_Update(&control, 450U);
    assert(event == BALL_SHUTTLE_EVENT_SUCCEEDED);
    assert(BallShuttleControl_IsHolding(&control) != 0U);
    assert_float_near_tolerance(
        BallShuttleControl_GetTargetPipePitchDeg(&control), 0.0f, 0.001f);
    assert(control.elapsed_ms == 450U);

    event = BallShuttleControl_Update(&control, 5000U);
    assert(event == BALL_SHUTTLE_EVENT_NONE);
    assert(BallShuttleControl_IsHolding(&control) != 0U);

    BallShuttleControl_HoldLevel(&control, 5000U);
    assert(control.state == BALL_SHUTTLE_STATE_HOLDING_LEVEL);
    assert_float_near_tolerance(
        BallShuttleControl_GetTargetPipePitchDeg(&control), 0.0f, 0.001f);
    event = BallShuttleControl_Update(&control, 5100U);
    assert(event == BALL_SHUTTLE_EVENT_NONE);

    BallShuttleControl_Stop(&control);
    assert(control.state == BALL_SHUTTLE_STATE_IDLE);
    assert(BallShuttleControl_Start(&control, 5200U) != 0U);
    BallShuttleControl_Stop(&control);
    assert(BallShuttleControl_IsRunning(&control) == 0U);
}

static void test_extended_downward_pipe_limit(void)
{
    BallBalanceControl control;
    BallBalanceMeasurement measurement;
    BallBalanceAttitude attitude;
    BallBalanceOutput output;
    float mapping_pitch_deg[LINKAGE_KINEMATICS_CALIBRATION_POINT_COUNT];
    uint32_t now_ms = 0U;
    uint8_t index;

    BallBalanceControl_Init(&control);
    build_linear_mapping(0.55f, 0.0f, mapping_pitch_deg);
    BallBalanceControl_SetPipeCalibration(
        &control, 1U, 0.55f, 0.0f, mapping_pitch_deg);
    BallBalanceControl_Update(&control, now_ms, 0, 0, &output);
    assert(output.mapping_valid != 0U);
    assert_float_near_tolerance(
        output.minimum_pipe_pitch_deg, -5.5495f, 0.001f);

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
    measurement.position_d10_mm = -1300;
    measurement.quality = 80U;
    measurement.sequence = 1U;
    now_ms += 20U;
    attitude.timestamp_ms = now_ms;
    BallBalanceControl_Update(&control,
                              now_ms,
                              &measurement,
                              &attitude,
                              &output);
    assert(output.trajectory_phase == BALL_BALANCE_PHASE_WAIT_PIPE);
    measurement.sequence = 2U;
    now_ms += 20U;
    attitude.timestamp_ms = now_ms;
    BallBalanceControl_Update(&control,
                              now_ms,
                              &measurement,
                              &attitude,
                              &output);
    assert_float_near_tolerance(
        output.desired_pipe_pitch_deg, -2.57f, 0.01f);
    assert(output.desired_pipe_pitch_deg < 0.0f);
    assert(output.mapping_valid != 0U);
    assert(output.mapping_fault == 0U);
}

static void test_nonlinear_mapping_interpolation(void)
{
    BallBalanceControl control;
    BallBalanceMeasurement measurement;
    BallBalanceAttitude attitude;
    BallBalanceOutput output;
    float mapping_pitch_deg[LINKAGE_KINEMATICS_CALIBRATION_POINT_COUNT];
    uint32_t now_ms = 0U;
    uint8_t index;

    BallBalanceControl_Init(&control);
    BallBalanceControl_SetPidGains(&control, 0.040f, 0.0f, 0.030f);
    build_nonlinear_mapping(mapping_pitch_deg);
    BallBalanceControl_SetPipeCalibration(
        &control, 1U, 0.42f, 0.0f, mapping_pitch_deg);
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
    measurement.position_d10_mm = 473;
    measurement.quality = 80U;
    measurement.sequence = 1U;
    now_ms += 20U;
    attitude.timestamp_ms = now_ms;
    BallBalanceControl_Update(&control,
                              now_ms,
                              &measurement,
                              &attitude,
                              &output);
    measurement.sequence = 2U;
    now_ms += 20U;
    attitude.timestamp_ms = now_ms;
    BallBalanceControl_Update(&control,
                              now_ms,
                              &measurement,
                              &attitude,
                              &output);
    for (index = 0U; index < 7U; index++) {
        now_ms += 20U;
        attitude.timestamp_ms = now_ms;
        BallBalanceControl_Update(&control,
                                  now_ms,
                                  0,
                                  &attitude,
                                  &output);
    }

    assert(output.command_pipe_pitch_deg > 0.0f);
    assert(output.command_pipe_pitch_deg < output.desired_pipe_pitch_deg);
    assert(test_abs(output.pipe_command_rate_dps) <= 12.01f);
    assert(output.feedforward_model_pitch_deg > 0.0f);
    assert(output.servo_feedforward_deg >
           BallBalanceControl_GetNeutralAngleDeg());
    assert(output.mapping_fault == 0U);
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

static void test_ball_balance_transport_mode(void)
{
    BallBalanceControl control;
    BallBalanceMeasurement measurement;
    BallBalanceAttitude attitude;
    BallBalanceOutput output;
    float mapping_pitch_deg[LINKAGE_KINEMATICS_CALIBRATION_POINT_COUNT];
    uint32_t now_ms = 0U;
    uint8_t index;

    build_linear_mapping(1.0f, 0.0f, mapping_pitch_deg);
    BallBalanceControl_Init(&control);
    BallBalanceControl_SetPipeCalibration(
        &control, 1U, 1.0f, 0.0f, mapping_pitch_deg);
    BallBalanceControl_SetTransportMode(&control, 1U);
    BallBalanceControl_SetEnabled(&control, 1U, now_ms);

    attitude.valid = 1U;
    attitude.pitch_deg = 0.0f;
    attitude.pitch_rate_dps = 0.0f;
    for (index = 0U; index < 30U; index++) {
        now_ms += 20U;
        attitude.timestamp_ms = now_ms;
        BallBalanceControl_Update(&control, now_ms, 0, &attitude, &output);
    }
    assert(output.enabled != 0U);
    assert(output.transport_mode_enabled != 0U);

    BallBalanceControl_SetTransportInputs(&control, 8.0f, 0.80f, 1U);
    measurement.valid = 1U;
    measurement.position_d10_mm = 0;
    measurement.quality = 80U;
    measurement.sequence = 1U;
    now_ms += 100U;
    attitude.timestamp_ms = now_ms;
    BallBalanceControl_Update(
        &control, now_ms, &measurement, &attitude, &output);
    measurement.sequence = 2U;
    now_ms += 100U;
    attitude.timestamp_ms = now_ms;
    BallBalanceControl_Update(
        &control, now_ms, &measurement, &attitude, &output);
    assert(output.transport_feedforward_pitch_deg < -0.45f);
    assert(output.transport_feedforward_pitch_deg > -0.75f);
    assert_float_near_tolerance(
        output.transport_feedback_pitch_deg, 0.0f, 0.01f);
    assert(output.desired_pipe_pitch_deg < 0.0f);

    BallBalanceControl_SetTransportInputs(&control, 0.0f, 0.0f, 1U);
    BallBalanceControl_SetTransportWallAcceleration(
        &control, 5.0f, 1U);
    measurement.sequence++;
    now_ms += 100U;
    attitude.timestamp_ms = now_ms;
    BallBalanceControl_Update(
        &control, now_ms, &measurement, &attitude, &output);
    assert_float_near_tolerance(
        output.transport_wall_feedforward_pitch_deg,
        -0.292025f, 0.01f);
    assert(output.transport_feedforward_pitch_deg < -0.28f);
    assert(output.transport_feedforward_pitch_deg > -0.31f);

    BallBalanceControl_SetTransportInputs(&control, 0.0f, 0.0f, 1U);
    BallBalanceControl_SetTransportWallAcceleration(
        &control, 0.0f, 0U);
    measurement.position_d10_mm = 300;
    measurement.sequence++;
    now_ms += 100U;
    attitude.timestamp_ms = now_ms;
    BallBalanceControl_Update(
        &control, now_ms, &measurement, &attitude, &output);
    /* K230 约 5~6 FPS 时，行驶中相邻帧 30 mm 位移必须立即进入反馈，
     * 不能沿用停车模式的 15 mm spike 门槛冻结 400 ms。 */
    assert(output.measurement_accepted != 0U);
    assert(control.spike_reject_active == 0U);
    assert(output.filtered_position_mm > 0.0f);
    for (index = 0U; index < 8U; index++) {
        measurement.sequence++;
        now_ms += 100U;
        attitude.timestamp_ms = now_ms;
        BallBalanceControl_Update(
            &control, now_ms, &measurement, &attitude, &output);
    }
    assert(output.transport_recovery_active != 0U);
    assert(output.transport_feedback_pitch_deg >= 0.64f);
    assert(output.transport_feedback_pitch_deg <= 1.20f);
    assert(output.desired_pipe_pitch_deg > 0.0f);
    assert(output.integral_active == 0U);

    /* 匀速时扩大中心死区，避免小位置抖动反复驱动水管；速度阻尼不在此清零。 */
    measurement.position_d10_mm = 40;
    for (index = 0U; index < 8U; index++) {
        measurement.sequence++;
        now_ms += 100U;
        attitude.timestamp_ms = now_ms;
        BallBalanceControl_Update(
            &control, now_ms, &measurement, &attitude, &output);
    }
    assert_float_near_tolerance(
        output.transport_position_deadband_mm, 5.0f, 0.001f);
    assert(output.transport_recovery_active == 0U);
    assert_float_near_tolerance(output.p_term_deg, 0.0f, 0.001f);

    /* 同样的 4 mm 偏差在起步/停车阶段仍超过 2 mm 死区并产生位置修正。 */
    BallBalanceControl_SetTransportInputs(&control, 8.0f, 0.0f, 1U);
    measurement.sequence++;
    now_ms += 100U;
    attitude.timestamp_ms = now_ms;
    BallBalanceControl_Update(
        &control, now_ms, &measurement, &attitude, &output);
    assert_float_near_tolerance(
        output.transport_position_deadband_mm, 2.0f, 0.001f);
    assert(output.p_term_deg > 0.0f);

    /* 负方向越界使用完全对称的回收角，持续位置也会越过运输跳变保护重采样。 */
    BallBalanceControl_SetTransportInputs(&control, 0.0f, 0.0f, 1U);
    measurement.position_d10_mm = -300;
    for (index = 0U; index < 5U; index++) {
        measurement.sequence++;
        now_ms += 100U;
        attitude.timestamp_ms = now_ms;
        BallBalanceControl_Update(
            &control, now_ms, &measurement, &attitude, &output);
    }
    assert(output.transport_recovery_active != 0U);
    assert(output.transport_feedback_pitch_deg <= -0.64f);
    assert(output.transport_feedback_pitch_deg >= -1.20f);
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
    test_ball_balance_friction_compensation();
    test_ball_balance_motion_classification();
    test_ball_balance_endpoint_integral();
    test_ball_shuttle_control();
    test_flexible_pipe_mapping_limits();
    test_extended_downward_pipe_limit();
    test_nonlinear_mapping_interpolation();
    test_ball_balance_transport_mode();
    test_speed_pid();
    puts("基础控制、IMU 姿态与钢球平衡主机测试通过");
    return 0;
}
