#include "chassis_accel_estimator.h"
#include "chassis_motion_profile.h"
#include "infrared_line_control.h"

#include <assert.h>
#include <stdio.h>

#define TEST_EPSILON 0.0001f /* 浮点测试允许的绝对误差。 */

static float test_abs(float value)
{
    return (value < 0.0f) ? -value : value;
}

static void assert_near(float actual, float expected, float tolerance)
{
    assert(test_abs(actual - expected) <= tolerance);
}

static ChassisMotionConfig build_motion_config(float completion_distance_cm)
{
    ChassisMotionConfig config;

    config.target_speed_cm_s = 40.0f;          /* 测试使用的巡航目标速度。 */
    config.maximum_accel_cm_s2 = 25.0f;        /* 测试使用的最大正向加速度。 */
    config.maximum_decel_cm_s2 = 30.0f;        /* 测试使用的最大减速度。 */
    config.maximum_jerk_cm_s3 = 100.0f;        /* 测试使用的最大加速度变化率。 */
    config.completion_distance_cm = completion_distance_cm;
    return config;
}

static void test_motion_profile_smooth_start(void)
{
    ChassisMotionProfile profile;
    ChassisMotionOutput output;
    ChassisMotionConfig config = build_motion_config(0.0f);
    float previous_speed = 0.0f;
    float previous_accel = 0.0f;
    uint32_t now_ms = 0U;
    uint16_t index;

    ChassisMotionProfile_Init(&profile);
    assert(profile.state == CHASSIS_MOTION_STATE_IDLE);
    assert(ChassisMotionProfile_Start(&profile, now_ms, 0.0f,
                                      &config) != 0U);

    for (index = 0U; index < 250U; index++) {
        now_ms += 20U;
        ChassisMotionProfile_Update(&profile, now_ms, 0.0f, &output);
        assert(output.command_speed_cm_s + TEST_EPSILON >= previous_speed);
        assert(output.command_speed_cm_s <= config.target_speed_cm_s +
               TEST_EPSILON);
        assert(output.command_accel_cm_s2 <=
               config.maximum_accel_cm_s2 + TEST_EPSILON);
        assert(test_abs(output.command_accel_cm_s2 - previous_accel) <=
               config.maximum_jerk_cm_s3 * 0.020f + TEST_EPSILON);
        previous_speed = output.command_speed_cm_s;
        previous_accel = output.command_accel_cm_s2;
    }

    assert_near(output.command_speed_cm_s, 40.0f, 0.01f);
    assert(output.state == CHASSIS_MOTION_STATE_CRUISING);
}

static void test_motion_profile_target_reduction(void)
{
    ChassisMotionProfile profile;
    ChassisMotionOutput output;
    ChassisMotionConfig config = build_motion_config(0.0f);
    uint32_t now_ms = 0U;
    uint16_t index;

    assert(ChassisMotionProfile_Start(&profile, now_ms, 40.0f,
                                      &config) != 0U);
    ChassisMotionProfile_SetTargetSpeed(&profile, 20.0f);
    for (index = 0U; index < 200U; index++) {
        now_ms += 20U;
        ChassisMotionProfile_Update(&profile, now_ms, 0.0f, &output);
        assert(output.command_speed_cm_s >= 0.0f);
        assert(output.command_accel_cm_s2 >=
               -config.maximum_decel_cm_s2 - TEST_EPSILON);
    }

    assert_near(output.command_speed_cm_s, 20.0f, 0.01f);
    assert(output.state == CHASSIS_MOTION_STATE_CRUISING);
}

static void test_motion_profile_pass_event(void)
{
    ChassisMotionProfile profile;
    ChassisMotionOutput output;
    ChassisMotionConfig config = build_motion_config(150.0f);
    uint32_t now_ms = 0U;
    uint16_t index;

    assert(ChassisMotionProfile_Start(&profile, now_ms, 0.0f,
                                      &config) != 0U);
    for (index = 0U; index < 149U; index++) {
        now_ms += 20U;
        ChassisMotionProfile_Update(&profile, now_ms, 1.0f, &output);
        assert(output.distance_reached == 0U);
    }

    now_ms += 20U;
    ChassisMotionProfile_Update(&profile, now_ms, 1.0f, &output);
    assert(output.distance_reached != 0U);
    assert(output.state == CHASSIS_MOTION_STATE_PASSED);
    assert_near(output.travelled_distance_cm, 150.0f, 0.001f);

    now_ms += 20U;
    ChassisMotionProfile_Update(&profile, now_ms, 2.0f, &output);
    assert(output.state == CHASSIS_MOTION_STATE_PASSED);
    assert(output.command_speed_cm_s > 0.0f);
}

static void test_motion_profile_soft_stop(void)
{
    ChassisMotionProfile profile;
    ChassisMotionOutput output;
    ChassisMotionConfig config = build_motion_config(0.0f);
    float previous_accel = 0.0f;
    uint32_t now_ms = 0U;
    uint16_t index;

    config.target_speed_cm_s = 26.0f;
    config.maximum_accel_cm_s2 = 8.0f;
    config.maximum_decel_cm_s2 = 8.0f;
    config.maximum_jerk_cm_s3 = 24.0f;
    assert(ChassisMotionProfile_Start(&profile, now_ms, 26.0f,
                                      &config) != 0U);
    ChassisMotionProfile_Update(&profile, now_ms, 0.0f, &output);
    ChassisMotionProfile_RequestStop(&profile);

    for (index = 0U; index < 500U; index++) {
        now_ms += 20U;
        ChassisMotionProfile_Update(&profile, now_ms, 0.0f, &output);
        assert(output.command_speed_cm_s >= 0.0f);
        assert(output.command_accel_cm_s2 >=
               -config.maximum_decel_cm_s2 - TEST_EPSILON);
        assert(test_abs(output.command_accel_cm_s2 - previous_accel) <=
               config.maximum_jerk_cm_s3 * 0.020f + TEST_EPSILON);
        previous_accel = output.command_accel_cm_s2;
        if (output.stopped != 0U) {
            break;
        }
    }

    assert(output.stop_requested != 0U);
    assert(output.stopped != 0U);
    assert(output.state == CHASSIS_MOTION_STATE_STOPPED);
    assert_near(output.command_speed_cm_s, 0.0f, TEST_EPSILON);
    assert_near(output.command_accel_cm_s2, 0.0f, TEST_EPSILON);
}

static uint32_t simulate_pass_time_ms(const ChassisMotionConfig *config)
{
    ChassisMotionProfile profile;
    ChassisMotionOutput output;
    uint32_t now_ms = 0U;
    uint16_t index;

    assert(ChassisMotionProfile_Start(&profile, now_ms, 0.0f,
                                      config) != 0U);
    ChassisMotionProfile_Update(&profile, now_ms, 0.0f, &output);
    for (index = 0U; index < 2000U; index++) {
        now_ms += 20U;
        ChassisMotionProfile_Update(
            &profile,
            now_ms,
            output.command_speed_cm_s * 0.020f,
            &output);
        if (output.distance_reached != 0U) {
            return now_ms;
        }
    }
    assert(0);
    return 0U;
}

static void test_ab_and_abcd_time_budget(void)
{
    ChassisMotionConfig ab_config;
    ChassisMotionConfig abcd_config;

    ab_config.target_speed_cm_s = 35.0f;
    ab_config.maximum_accel_cm_s2 = 10.0f;
    ab_config.maximum_decel_cm_s2 = 8.0f;
    ab_config.maximum_jerk_cm_s3 = 30.0f;
    ab_config.completion_distance_cm = 150.0f;
    assert(simulate_pass_time_ms(&ab_config) <= 8000U);

    abcd_config.target_speed_cm_s = 26.0f;
    abcd_config.maximum_accel_cm_s2 = 8.0f;
    abcd_config.maximum_decel_cm_s2 = 8.0f;
    abcd_config.maximum_jerk_cm_s3 = 24.0f;
    abcd_config.completion_distance_cm = 614.2f;
    assert(simulate_pass_time_ms(&abcd_config) <= 30000U);
}

static void test_motion_profile_rejects_invalid_config(void)
{
    ChassisMotionProfile profile;
    ChassisMotionConfig config = build_motion_config(0.0f);

    config.maximum_jerk_cm_s3 = 0.0f;
    assert(ChassisMotionProfile_Start(&profile, 0U, 0.0f,
                                      &config) == 0U);
}

static void test_chassis_accel_estimator(void)
{
    ChassisAccelEstimator estimator;
    ChassisAccelOutput output;
    uint32_t now_ms = 0U;
    uint16_t index;

    ChassisAccelEstimator_Init(&estimator);
    ChassisAccelEstimator_Update(&estimator, now_ms, 0.0f, 0.0f, &output);
    assert(output.valid == 0U);

    now_ms += 20U;
    ChassisAccelEstimator_Update(&estimator, now_ms, 10.0f, 10.0f, &output);
    assert(output.valid != 0U);
    assert(output.filtered_speed_cm_s > 0.0f);
    assert(output.accel_cm_s2 > 0.0f);
    assert(output.raw_accel_cm_s2 <= 250.0f + TEST_EPSILON);

    for (index = 0U; index < 100U; index++) {
        now_ms += 20U;
        ChassisAccelEstimator_Update(&estimator, now_ms,
                                     10.0f, 10.0f, &output);
    }
    assert_near(output.filtered_speed_cm_s, 10.0f, 0.01f);
    assert_near(output.accel_cm_s2, 0.0f, 0.05f);

    now_ms += 20U;
    ChassisAccelEstimator_Update(&estimator, now_ms,
                                 1000.0f, 1000.0f, &output);
    assert(output.raw_accel_cm_s2 <= 250.0f + TEST_EPSILON);

    now_ms += 200U;
    ChassisAccelEstimator_Update(&estimator, now_ms,
                                 20.0f, 20.0f, &output);
    assert(output.valid == 0U);
    assert_near(output.filtered_speed_cm_s, 20.0f, 0.001f);
    assert_near(output.accel_cm_s2, 0.0f, 0.001f);
}

static void test_runtime_line_base_speed(void)
{
    InfraredLineControl control;
    InfraredLineOutput output;
    float average_target;

    InfraredLineControl_Init(&control);
    InfraredLineControl_SetBaseSpeed(&control, 40.0f);
    InfraredLineControl_Update(&control, 0U, 0x18U, 0U, &output);
    InfraredLineControl_Update(&control, 20U, 0x01U, 0U, &output);
    average_target = 0.5f *
                     (output.left_target_cm_s + output.right_target_cm_s);
    /* 大偏差降速先平滑建立减速度，不再从40 cm/s直接跳到缩放速度。 */
    assert_near(average_target, 39.96f, 0.01f);

    InfraredLineControl_Init(&control);
    InfraredLineControl_SetBaseSpeed(&control, 12.0f);
    InfraredLineControl_Update(&control, 0U, 0x18U, 0U, &output);
    assert_near(output.left_target_cm_s, 12.0f, 0.001f);
    assert_near(output.right_target_cm_s, 12.0f, 0.001f);

    InfraredLineControl_Init(&control);
    InfraredLineControl_SetBaseSpeed(&control, 1.0f);
    InfraredLineControl_Update(&control, 20U, 0x01U, 0U, &output);
    assert(test_abs(output.correction_cm_s) <= 1.0f + TEST_EPSILON);
    assert(output.left_target_cm_s <= 2.0f + TEST_EPSILON);
    assert(output.right_target_cm_s <= 2.0f + TEST_EPSILON);

    InfraredLineControl_Init(&control);
    InfraredLineControl_SetBaseSpeed(&control, -5.0f);
    InfraredLineControl_Update(&control, 40U, 0x18U, 0U, &output);
    assert_near(output.left_target_cm_s, 0.0f, 0.001f);
    assert_near(output.right_target_cm_s, 0.0f, 0.001f);
}

static void test_dynamic_line_speed_uses_task_motion_limits(void)
{
    InfraredLineControl control;
    InfraredLineOutput output;
    float previous_speed = 0.0f;
    float previous_accel = 0.0f;
    float average_target;
    float speed_delta;
    uint32_t now_ms = 0U;
    uint16_t index;

    InfraredLineControl_Init(&control);
    InfraredLineControl_SetMotionLimits(&control,
                                        8.0f,  /* 第五题最大加速度。 */
                                        8.0f,  /* 第五题最大减速度。 */
                                        24.0f  /* 第五题最大jerk。 */);
    InfraredLineControl_SetBaseSpeed(&control, 0.0f);
    InfraredLineControl_Update(&control, now_ms, 0x18U, 0U, &output);
    InfraredLineControl_SetBaseSpeed(&control, 26.0f);

    for (index = 0U; index < 250U; index++) {
        now_ms += 20U;
        InfraredLineControl_Update(&control, now_ms, 0x18U, 0U, &output);
        average_target = 0.5f *
                         (output.left_target_cm_s +
                          output.right_target_cm_s);
        speed_delta = average_target - previous_speed;
        assert(speed_delta <= 8.0f * 0.020f + TEST_EPSILON);
        assert(speed_delta >= -8.0f * 0.020f - TEST_EPSILON);
        assert(test_abs(control.applied_base_accel_cm_s2 -
                        previous_accel) <=
               24.0f * 0.020f + TEST_EPSILON);
        previous_speed = average_target;
        previous_accel = control.applied_base_accel_cm_s2;
    }
    assert_near(previous_speed, 26.0f, 0.01f);

    /* 第五题检测到宽线以及离开宽线时，减速和恢复都不能发生速度阶跃。 */
    for (index = 0U; index < 80U; index++) {
        now_ms += 20U;
        InfraredLineControl_Update(&control, now_ms, 0x18U, 1U, &output);
        average_target = 0.5f *
                         (output.left_target_cm_s +
                          output.right_target_cm_s);
        speed_delta = average_target - previous_speed;
        assert(speed_delta <= 8.0f * 0.020f + TEST_EPSILON);
        assert(speed_delta >= -8.0f * 0.020f - TEST_EPSILON);
        assert(test_abs(control.applied_base_accel_cm_s2 -
                        previous_accel) <=
               24.0f * 0.020f + TEST_EPSILON);
        previous_speed = average_target;
        previous_accel = control.applied_base_accel_cm_s2;
    }

    for (index = 0U; index < 180U; index++) {
        now_ms += 20U;
        InfraredLineControl_Update(&control, now_ms, 0x18U, 0U, &output);
        average_target = 0.5f *
                         (output.left_target_cm_s +
                          output.right_target_cm_s);
        speed_delta = average_target - previous_speed;
        assert(speed_delta <= 8.0f * 0.020f + TEST_EPSILON);
        assert(speed_delta >= -8.0f * 0.020f - TEST_EPSILON);
        assert(test_abs(control.applied_base_accel_cm_s2 -
                        previous_accel) <=
               24.0f * 0.020f + TEST_EPSILON);
        previous_speed = average_target;
        previous_accel = control.applied_base_accel_cm_s2;
    }
    assert_near(previous_speed, 26.0f, 0.01f);
}

int main(void)
{
    test_motion_profile_smooth_start();
    test_motion_profile_target_reduction();
    test_motion_profile_pass_event();
    test_motion_profile_soft_stop();
    test_ab_and_abcd_time_budget();
    test_motion_profile_rejects_invalid_config();
    test_chassis_accel_estimator();
    test_runtime_line_base_speed();
    test_dynamic_line_speed_uses_task_motion_limits();
    puts("平滑底盘速度轨迹与加速度估计测试通过");
    return 0;
}
