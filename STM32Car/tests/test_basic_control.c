#include "infrared_line_control.h"
#include "lap_stop_detector.h"
#include "wheel_speed_pid.h"
#include <assert.h>
#include <stdio.h>

#define TEST_FLOAT_EPSILON 0.0001f
#define TEST_TARGET_MAX_CM_S 60.0f

static float test_abs(float value)
{
    return (value < 0.0f) ? -value : value;
}

static void assert_float_near(float actual, float expected)
{
    assert(test_abs(actual - expected) <= TEST_FLOAT_EPSILON);
}

static void assert_targets_in_range(const InfraredLineOutput *output)
{
    assert(output->left_target_cm_s >= 0.0f);
    assert(output->left_target_cm_s <= TEST_TARGET_MAX_CM_S);
    assert(output->right_target_cm_s >= 0.0f);
    assert(output->right_target_cm_s <= TEST_TARGET_MAX_CM_S);
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
        InfraredLineControl_Update(&control, 0U, masks[index], &output);
        assert(output.command_valid != 0U);
        assert(output.active_count == 1U);
        assert_float_near(output.raw_error, expected_errors[index]);
        assert_float_near(output.filtered_error, expected_errors[index]);
        assert_targets_in_range(&output);
    }

    InfraredLineControl_Init(&control);
    InfraredLineControl_Update(&control, 0U, 0x18U, &output);
    assert(output.command_valid != 0U);
    assert(output.active_count == 2U);
    assert_float_near(output.raw_error, 0.0f);
    assert_float_near(output.filtered_error, 0.0f);
    assert_float_near(output.left_target_cm_s, output.right_target_cm_s);

    InfraredLineControl_Init(&control);
    InfraredLineControl_Update(&control, 0U, 0x03U, &output);
    assert_float_near(output.raw_error, -3.6f);

    InfraredLineControl_Init(&control);
    InfraredLineControl_Update(&control, 0U, 0xC0U, &output);
    assert_float_near(output.raw_error, 3.6f);

    InfraredLineControl_Init(&control);
    InfraredLineControl_Update(&control, 0U, 0xFFU, &output);
    assert(output.command_valid != 0U);
    assert(output.wide_line != 0U);
    assert_float_near(output.filtered_error, 0.0f);
}

static void test_infrared_target_differential(void)
{
    InfraredLineControl control;
    InfraredLineOutput output;

    InfraredLineControl_Init(&control);
    InfraredLineControl_Update(&control, 0U, 0x01U, &output);
    assert(output.left_target_cm_s < output.right_target_cm_s);
    assert_targets_in_range(&output);

    InfraredLineControl_Init(&control);
    InfraredLineControl_Update(&control, 0U, 0x08U, &output);
    assert(output.left_target_cm_s < output.right_target_cm_s);
    assert(output.left_target_cm_s != output.right_target_cm_s);
    assert_targets_in_range(&output);

    InfraredLineControl_Init(&control);
    InfraredLineControl_Update(&control, 0U, 0x10U, &output);
    assert(output.left_target_cm_s > output.right_target_cm_s);
    assert(output.left_target_cm_s != output.right_target_cm_s);
    assert_targets_in_range(&output);

    InfraredLineControl_Init(&control);
    InfraredLineControl_Update(&control, 0U, 0x80U, &output);
    assert(output.left_target_cm_s > output.right_target_cm_s);
    assert_targets_in_range(&output);
}

static void test_infrared_loss_recovery(void)
{
    InfraredLineControl control;
    InfraredLineOutput output;

    InfraredLineControl_Init(&control);
    InfraredLineControl_Update(&control, 0U, 0x08U, &output);
    assert(output.left_target_cm_s < output.right_target_cm_s);

    InfraredLineControl_Update(&control, 200U, 0x00U, &output);
    assert(output.command_valid != 0U);
    assert(output.line_lost != 0U);
    assert_float_near(output.raw_error, -4.2f);
    assert(output.correction_cm_s < 0.0f);
    assert(output.left_target_cm_s < output.right_target_cm_s);
    assert_targets_in_range(&output);

    InfraredLineControl_Update(&control, 220U, 0x00U, &output);
    assert(output.command_valid != 0U);
    assert(output.line_lost != 0U);
    assert_float_near(output.raw_error, -4.2f);
    assert(output.correction_cm_s < 0.0f);
    assert(output.left_target_cm_s < output.right_target_cm_s);
    assert_targets_in_range(&output);

    InfraredLineControl_Update(&control, 500U, 0x00U, &output);
    assert(output.command_valid != 0U);
    InfraredLineControl_Update(&control, 501U, 0x00U, &output);
    assert(output.command_valid == 0U);

    InfraredLineControl_Init(&control);
    InfraredLineControl_Update(&control, 0U, 0x10U, &output);
    InfraredLineControl_Update(&control, 100U, 0x00U, &output);
    assert(output.command_valid != 0U);
    assert_float_near(output.raw_error, 4.2f);
    assert(output.correction_cm_s > 0.0f);
    assert(output.left_target_cm_s > output.right_target_cm_s);

    InfraredLineControl_Update(&control, 120U, 0x10U, &output);
    assert(output.command_valid != 0U);
    assert(output.line_lost == 0U);
    assert_float_near(output.raw_error, 0.6f);
}

static void test_infrared_safe_loss_behavior(void)
{
    InfraredLineControl control;
    InfraredLineOutput output;

    InfraredLineControl_Init(&control);
    InfraredLineControl_Update(&control, 0U, 0x00U, &output);
    assert(output.command_valid == 0U);
    assert(output.line_lost != 0U);

    InfraredLineControl_Init(&control);
    InfraredLineControl_Update(&control, 0U, 0x18U, &output);
    assert(output.command_valid != 0U);
    InfraredLineControl_Update(&control, 20U, 0x00U, &output);
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

    assert(LapStopDetector_Update(&detector, 1.0f, 1U, 0x18U) == 0U);
    assert(detector.armed != 0U);
    assert(detector.full_black_count == 0U);

    assert(LapStopDetector_Update(&detector, 0.0f, 1U, 0xFFU) == 0U);
    assert(detector.full_black_count == 1U);
    assert(LapStopDetector_Update(&detector, 0.0f, 1U, 0x18U) == 0U);
    assert(detector.full_black_count == 0U);

    assert(LapStopDetector_Update(&detector, 0.0f, 1U, 0xFFU) == 0U);
    assert(LapStopDetector_Update(&detector, 0.0f, 0U, 0xFFU) == 0U);
    assert(detector.full_black_count == 0U);

    assert(LapStopDetector_Update(&detector, 0.0f, 1U, 0xFFU) == 0U);
    assert(LapStopDetector_Update(&detector, 0.0f, 1U, 0xFFU) == 0U);
    assert(LapStopDetector_Update(&detector, 0.0f, 1U, 0xFFU) != 0U);
    assert(detector.completed != 0U);
    assert(detector.full_black_count == 3U);
    assert(LapStopDetector_Update(&detector, 10.0f, 1U, 0x18U) != 0U);
    assert_float_near(detector.distance_cm, 400.0f);

    LapStopDetector_Init(&detector);
    assert_float_near(detector.distance_cm, 0.0f);
    assert(detector.armed == 0U);
    assert(detector.full_black_count == 0U);
    assert(detector.completed == 0U);
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
    test_infrared_target_differential();
    test_infrared_loss_recovery();
    test_infrared_safe_loss_behavior();
    test_lap_stop_detector();
    test_speed_pid();
    puts("基础巡线主机测试通过");
    return 0;
}
