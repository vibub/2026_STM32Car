#include "infrared_line_control.h"
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

static void test_infrared_physical_positions(void)
{
    static const uint8_t masks[4] = {0x01U, 0x02U, 0x04U, 0x08U};
    static const float expected_errors[4] = {-3.0f, -0.4f, 0.4f, 3.0f};
    InfraredLineControl control;
    InfraredLineOutput output;
    uint8_t index;

    for (index = 0U; index < 4U; index++) {
        InfraredLineControl_Init(&control);
        InfraredLineControl_Update(&control, 0U, masks[index], &output);
        assert(output.command_valid != 0U);
        assert(output.active_count == 1U);
        assert_float_near(output.raw_error, expected_errors[index]);
        assert_float_near(output.filtered_error, expected_errors[index]);
        assert_targets_in_range(&output);
    }

    InfraredLineControl_Init(&control);
    InfraredLineControl_Update(&control, 0U, 0x03U, &output);
    assert_float_near(output.raw_error, -1.7f);

    InfraredLineControl_Init(&control);
    InfraredLineControl_Update(&control, 0U, 0x0CU, &output);
    assert_float_near(output.raw_error, 1.7f);

    InfraredLineControl_Init(&control);
    InfraredLineControl_Update(&control, 0U, 0x06U, &output);
    assert(output.command_valid != 0U);
    assert(output.active_count == 2U);
    assert_float_near(output.filtered_error, 0.0f);
    assert_float_near(output.left_target_cm_s, output.right_target_cm_s);

    InfraredLineControl_Init(&control);
    InfraredLineControl_Update(&control, 0U, 0x0FU, &output);
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
    InfraredLineControl_Update(&control, 0U, 0x02U, &output);
    assert(output.left_target_cm_s < output.right_target_cm_s);
    assert(output.left_target_cm_s != output.right_target_cm_s);
    assert_targets_in_range(&output);

    InfraredLineControl_Init(&control);
    InfraredLineControl_Update(&control, 0U, 0x04U, &output);
    assert(output.left_target_cm_s > output.right_target_cm_s);
    assert(output.left_target_cm_s != output.right_target_cm_s);
    assert_targets_in_range(&output);

    InfraredLineControl_Init(&control);
    InfraredLineControl_Update(&control, 0U, 0x08U, &output);
    assert(output.left_target_cm_s > output.right_target_cm_s);
    assert_targets_in_range(&output);
}

static void test_infrared_gap_bridge_and_search(void)
{
    InfraredLineControl control;
    InfraredLineOutput output;

    InfraredLineControl_Init(&control);
    InfraredLineControl_Update(&control, 0U, 0x02U, &output);
    assert(output.left_target_cm_s < output.right_target_cm_s);

    InfraredLineControl_Update(&control, 200U, 0x00U, &output);
    assert(output.command_valid != 0U);
    assert(output.line_lost != 0U);
    assert_float_near(output.raw_error, -1.7f);
    assert(output.correction_cm_s < 0.0f);
    assert(output.left_target_cm_s < output.right_target_cm_s);
    assert_targets_in_range(&output);

    InfraredLineControl_Update(&control, 220U, 0x00U, &output);
    assert(output.command_valid != 0U);
    assert(output.line_lost != 0U);
    assert(output.filtered_error < 0.0f);
    assert(output.correction_cm_s < 0.0f);
    assert(output.left_target_cm_s < output.right_target_cm_s);
    assert_targets_in_range(&output);

    InfraredLineControl_Update(&control, 500U, 0x00U, &output);
    assert(output.command_valid != 0U);
    InfraredLineControl_Update(&control, 501U, 0x00U, &output);
    assert(output.command_valid == 0U);

    InfraredLineControl_Init(&control);
    InfraredLineControl_Update(&control, 0U, 0x04U, &output);
    InfraredLineControl_Update(&control, 100U, 0x00U, &output);
    assert(output.command_valid != 0U);
    assert_float_near(output.raw_error, 1.7f);
    assert(output.correction_cm_s > 0.0f);
    assert(output.left_target_cm_s > output.right_target_cm_s);

    InfraredLineControl_Update(&control, 120U, 0x04U, &output);
    assert(output.command_valid != 0U);
    assert(output.line_lost == 0U);
    assert_float_near(output.raw_error, 0.4f);
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
    InfraredLineControl_Update(&control, 0U, 0x06U, &output);
    assert(output.command_valid != 0U);
    InfraredLineControl_Update(&control, 20U, 0x00U, &output);
    assert(output.command_valid == 0U);
    assert(output.line_lost != 0U);
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
    test_infrared_physical_positions();
    test_infrared_target_differential();
    test_infrared_gap_bridge_and_search();
    test_infrared_safe_loss_behavior();
    test_speed_pid();
    puts("基础巡线主机测试通过");
    return 0;
}
