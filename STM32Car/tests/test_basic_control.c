#include "infrared_line_control.h"
#include "wheel_speed_pid.h"
#include <assert.h>
#include <stdio.h>

static void test_infrared_line_control(void)
{
    InfraredLineControl control;
    InfraredLineOutput output;

    InfraredLineControl_Init(&control);
    InfraredLineControl_Update(&control, 0U, 0x06U, &output);
    assert(output.command_valid != 0U);
    assert(output.active_count == 2U);
    assert(output.filtered_error == 0.0f);
    assert(output.left_target_cm_s == output.right_target_cm_s);

    InfraredLineControl_Init(&control);
    InfraredLineControl_Update(&control, 0U, 0x01U, &output);
    assert(output.filtered_error < 0.0f);
    assert(output.left_target_cm_s < output.right_target_cm_s);

    InfraredLineControl_Init(&control);
    InfraredLineControl_Update(&control, 0U, 0x08U, &output);
    assert(output.filtered_error > 0.0f);
    assert(output.left_target_cm_s > output.right_target_cm_s);
    InfraredLineControl_Update(&control, 40U, 0x00U, &output);
    assert(output.command_valid != 0U);
    assert(output.line_lost != 0U);
    InfraredLineControl_Update(&control, 81U, 0x00U, &output);
    assert(output.command_valid == 0U);

    InfraredLineControl_Init(&control);
    InfraredLineControl_Update(&control, 0U, 0x0FU, &output);
    assert(output.command_valid != 0U);
    assert(output.wide_line != 0U);
    assert(output.filtered_error == 0.0f);
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
    test_infrared_line_control();
    test_speed_pid();
    puts("基础巡线主机测试通过");
    return 0;
}
