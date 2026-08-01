#include "dynamic_run_task.h"

#include <assert.h>
#include <stdio.h>

#define TEST_EPSILON 0.001f /* 浮点统计测试允许的绝对误差。 */

static float test_abs(float value)
{
    return (value < 0.0f) ? -value : value;
}

static DynamicRunTaskConfig build_config(void)
{
    DynamicRunTaskConfig config;

    config.prepare_timeout_ms = 10000U;       /* 测试使用的准备超时时间。 */
    config.ready_hold_ms = 300U;              /* 测试使用的中心稳定时间。 */
    config.ready_sample_gap_ms = 200U;        /* 测试使用的视觉样本间隔上限。 */
    config.ab_timeout_ms = 8000U;             /* 测试使用的第四题限时。 */
    config.abcd_timeout_ms = 30000U;          /* 测试使用的第五题限时。 */
    config.ready_position_limit_mm = 5.0f;    /* 测试使用的起步位置门槛。 */
    config.ready_speed_limit_mm_s = 15.0f;    /* 测试使用的起步速度门槛。 */
    config.warning_limit_mm = 10.0f;          /* 测试使用的超差门槛。 */
    config.warning_clear_mm = 8.0f;           /* 测试使用的警告清除门槛。 */
    config.warning_clear_samples = 3U;        /* 测试使用的警告清除样本数。 */
    config.overrange_confirm_samples = 2U;    /* 测试使用的超差锁存样本数。 */
    config.imu_fault_debounce_ms = 300U;      /* 测试使用的 IMU 故障去抖时长。 */
    config.vision_fault_debounce_ms = 500U;   /* 测试使用的视觉失联去抖时长。 */
    return config;
}

static DynamicRunTaskInput build_ready_input(uint32_t now_ms)
{
    DynamicRunTaskInput input = {0};

    input.now_ms = now_ms;
    input.balance_enabled = 1U;
    input.tracking_valid = 1U;
    input.attitude_valid = 1U;
    input.measurement_accepted = 1U;
    input.evaluate_completion = 1U;
    input.error_mm = 2.0f;
    input.velocity_mm_s = 5.0f;
    return input;
}

static void start_running_task(DynamicRunTask *task, DynamicRunMode mode)
{
    DynamicRunTaskInput input;
    DynamicRunTaskActions actions;
    uint32_t now_ms;

    assert(DynamicRunTask_Start(task, mode, 0U) != 0U);
    for (now_ms = 0U; now_ms <= 300U; now_ms += 100U) {
        input = build_ready_input(now_ms);
        DynamicRunTask_Update(task, &input, &actions);
    }
    assert(actions.start_motion != 0U);
    DynamicRunTask_ConfirmMotionStarted(task, 300U);
    assert(task->state == DYNAMIC_RUN_STATE_RUNNING);
}

static void test_rejects_invalid_config(void)
{
    DynamicRunTask task;
    DynamicRunTaskConfig config = build_config();

    config.warning_clear_mm = 10.0f;
    assert(DynamicRunTask_Init(&task, &config) == 0U);
    assert(DynamicRunTask_Init(0, &config) == 0U);
}

static void test_ready_gate_requires_continuous_samples(void)
{
    DynamicRunTask task;
    DynamicRunTaskConfig config = build_config();
    DynamicRunTaskInput input;
    DynamicRunTaskActions actions;

    assert(DynamicRunTask_Init(&task, &config) != 0U);
    assert(DynamicRunTask_Start(&task, DYNAMIC_RUN_MODE_AB, 0U) != 0U);

    input = build_ready_input(0U);
    DynamicRunTask_Update(&task, &input, &actions);
    assert(actions.start_motion == 0U);
    assert(task.ready_window_active != 0U);

    input = build_ready_input(250U);
    input.measurement_accepted = 0U;
    DynamicRunTask_Update(&task, &input, &actions);
    assert(task.ready_window_active == 0U);

    input = build_ready_input(300U);
    DynamicRunTask_Update(&task, &input, &actions);
    input = build_ready_input(400U);
    DynamicRunTask_Update(&task, &input, &actions);
    input = build_ready_input(500U);
    input.error_mm = 5.1f;
    DynamicRunTask_Update(&task, &input, &actions);
    assert(task.ready_window_active == 0U);

    input = build_ready_input(600U);
    DynamicRunTask_Update(&task, &input, &actions);
    input = build_ready_input(700U);
    DynamicRunTask_Update(&task, &input, &actions);
    input = build_ready_input(800U);
    DynamicRunTask_Update(&task, &input, &actions);
    input = build_ready_input(900U);
    DynamicRunTask_Update(&task, &input, &actions);
    assert(actions.start_motion != 0U);
}

static void test_prepare_timeout_reports_hard_failure(void)
{
    DynamicRunTask task;
    DynamicRunTaskConfig config = build_config();
    DynamicRunTaskInput input = {0};
    DynamicRunTaskActions actions;

    assert(DynamicRunTask_Init(&task, &config) != 0U);
    assert(DynamicRunTask_Start(&task, DYNAMIC_RUN_MODE_ABCD, 100U) != 0U);
    input.now_ms = 10100U;
    DynamicRunTask_Update(&task, &input, &actions);
    assert(actions.hard_stop != 0U);
    assert(actions.result_ready != 0U);
    assert(task.state == DYNAMIC_RUN_STATE_FAILED);
    assert(task.failure == DYNAMIC_RUN_FAILURE_PREPARE_TIMEOUT);
}

static void test_pass_records_metrics_and_soft_stops(void)
{
    DynamicRunTask task;
    DynamicRunTaskConfig config = build_config();
    DynamicRunTaskInput input;
    DynamicRunTaskActions actions;
    float rms_error_mm;

    assert(DynamicRunTask_Init(&task, &config) != 0U);
    start_running_task(&task, DYNAMIC_RUN_MODE_AB);

    input = build_ready_input(400U);
    input.error_mm = 3.0f;
    input.travelled_distance_cm = 30.0f;
    DynamicRunTask_Update(&task, &input, &actions);

    input = build_ready_input(500U);
    input.error_mm = 11.0f;
    input.travelled_distance_cm = 60.0f;
    DynamicRunTask_Update(&task, &input, &actions);
    assert(actions.warning_changed != 0U);
    assert(task.overrange_latched == 0U);

    input = build_ready_input(600U);
    input.error_mm = 12.0f;
    input.travelled_distance_cm = 90.0f;
    DynamicRunTask_Update(&task, &input, &actions);
    assert(task.overrange_latched != 0U);
    assert(task.overrange_event_count == 1U);

    input = build_ready_input(700U);
    input.measurement_accepted = 0U;
    input.travelled_distance_cm = 120.0f;
    DynamicRunTask_Update(&task, &input, &actions);

    input = build_ready_input(800U);
    input.error_mm = 4.0f;
    input.route_passed = 1U;
    input.travelled_distance_cm = 150.0f;
    DynamicRunTask_Update(&task, &input, &actions);
    assert(actions.request_soft_stop != 0U);
    assert(actions.result_ready != 0U);
    assert(actions.warning_changed != 0U);
    assert(task.state == DYNAMIC_RUN_STATE_BRAKING);
    assert(task.result == DYNAMIC_RUN_RESULT_PASSED_OVERRANGE);
    assert(task.elapsed_ms == 500U);
    assert(task.travelled_distance_cm == 150.0f);
    assert(task.maximum_abs_error_mm == 12.0f);
    assert(task.overrange_duration_ms == 300U);

    rms_error_mm = DynamicRunTask_GetRmsErrorMm(&task);
    assert(test_abs(rms_error_mm - 8.5147f) <= 0.001f);

    input = build_ready_input(900U);
    input.measurement_accepted = 0U;
    input.motion_stopped = 1U;
    DynamicRunTask_Update(&task, &input, &actions);
    assert(actions.enter_stop_mode != 0U);
    assert(task.state == DYNAMIC_RUN_STATE_COMPLETE);
}

static void test_single_overrange_sample_does_not_fail_result(void)
{
    DynamicRunTask task;
    DynamicRunTaskConfig config = build_config();
    DynamicRunTaskInput input;
    DynamicRunTaskActions actions;

    assert(DynamicRunTask_Init(&task, &config) != 0U);
    start_running_task(&task, DYNAMIC_RUN_MODE_AB);

    input = build_ready_input(400U);
    input.error_mm = 11.0f;
    DynamicRunTask_Update(&task, &input, &actions);
    input = build_ready_input(500U);
    input.error_mm = 9.0f;
    DynamicRunTask_Update(&task, &input, &actions);
    input = build_ready_input(600U);
    input.error_mm = 2.0f;
    input.route_passed = 1U;
    DynamicRunTask_Update(&task, &input, &actions);

    assert(task.overrange_latched == 0U);
    assert(task.result == DYNAMIC_RUN_RESULT_PASSED);
}

static void test_pass_has_priority_at_time_limit(void)
{
    DynamicRunTask task;
    DynamicRunTaskConfig config = build_config();
    DynamicRunTaskInput input;
    DynamicRunTaskActions actions;

    assert(DynamicRunTask_Init(&task, &config) != 0U);
    start_running_task(&task, DYNAMIC_RUN_MODE_AB);

    /* 底盘采样前只更新故障和统计，不得抢先把边界时刻判为超时。 */
    input = build_ready_input(8300U);
    input.evaluate_completion = 0U;
    DynamicRunTask_Update(&task, &input, &actions);
    assert(task.state == DYNAMIC_RUN_STATE_RUNNING);

    /* 同一时刻底盘采样确认通过时，通过事件优先于 8 s 超时。 */
    input = build_ready_input(8300U);
    input.measurement_accepted = 0U;
    input.route_passed = 1U;
    input.travelled_distance_cm = 150.0f;
    DynamicRunTask_Update(&task, &input, &actions);
    assert(task.elapsed_ms == 8000U);
    assert(task.result == DYNAMIC_RUN_RESULT_PASSED);
    assert(task.failure == DYNAMIC_RUN_FAILURE_NONE);
}

static void test_timeout_requests_soft_stop(void)
{
    DynamicRunTask task;
    DynamicRunTaskConfig config = build_config();
    DynamicRunTaskInput input;
    DynamicRunTaskActions actions;

    assert(DynamicRunTask_Init(&task, &config) != 0U);
    start_running_task(&task, DYNAMIC_RUN_MODE_AB);

    input = build_ready_input(8300U);
    input.travelled_distance_cm = 145.0f;
    DynamicRunTask_Update(&task, &input, &actions);
    assert(actions.request_soft_stop != 0U);
    assert(actions.hard_stop == 0U);
    assert(task.state == DYNAMIC_RUN_STATE_BRAKING);
    assert(task.result == DYNAMIC_RUN_RESULT_TIMEOUT);
    assert(task.failure == DYNAMIC_RUN_FAILURE_RUN_TIMEOUT);
}

static void test_braking_fault_preserves_frozen_time(void)
{
    DynamicRunTask task;
    DynamicRunTaskConfig config = build_config();
    DynamicRunTaskInput input;
    DynamicRunTaskActions actions;
    uint32_t frozen_elapsed_ms;

    assert(DynamicRunTask_Init(&task, &config) != 0U);
    start_running_task(&task, DYNAMIC_RUN_MODE_AB);

    input = build_ready_input(500U);
    input.route_passed = 1U;
    input.travelled_distance_cm = 150.0f;
    DynamicRunTask_Update(&task, &input, &actions);
    frozen_elapsed_ms = task.elapsed_ms;
    assert(task.state == DYNAMIC_RUN_STATE_BRAKING);

    input = build_ready_input(900U);
    input.measurement_accepted = 0U;
    input.tracking_valid = 0U;
    input.timed_out = 1U;
    DynamicRunTask_Update(&task, &input, &actions);
    /* 视觉故障去抖窗口内先保持柔停，不立即硬停车。 */
    assert(actions.hard_stop == 0U);
    assert(task.state == DYNAMIC_RUN_STATE_BRAKING);

    /* 持续超过 500 ms 去抖时长后判视觉故障并冻结成绩时间。 */
    input = build_ready_input(1500U);
    input.measurement_accepted = 0U;
    input.tracking_valid = 0U;
    input.timed_out = 1U;
    DynamicRunTask_Update(&task, &input, &actions);
    assert(actions.hard_stop != 0U);
    assert(task.state == DYNAMIC_RUN_STATE_FAILED);
    assert(task.elapsed_ms == frozen_elapsed_ms);
}

static void test_running_vision_fault_debounced(void)
{
    DynamicRunTask task;
    DynamicRunTaskConfig config = build_config();
    DynamicRunTaskInput input;
    DynamicRunTaskActions actions;

    assert(DynamicRunTask_Init(&task, &config) != 0U);
    start_running_task(&task, DYNAMIC_RUN_MODE_ABCD);

    /* 视觉短暂失联（K230 偶发丢帧）先保持运行，去抖窗口内不判失败。 */
    input = build_ready_input(400U);
    input.tracking_valid = 0U;
    input.timed_out = 1U;
    DynamicRunTask_Update(&task, &input, &actions);
    assert(actions.hard_stop == 0U);
    assert(task.state == DYNAMIC_RUN_STATE_RUNNING);

    /* 恢复视觉后计时清零。 */
    input = build_ready_input(700U);
    DynamicRunTask_Update(&task, &input, &actions);
    assert(actions.hard_stop == 0U);
    assert(task.state == DYNAMIC_RUN_STATE_RUNNING);

    /* 再次失联并持续超过 500 ms 去抖时长才判视觉故障停车。 */
    input = build_ready_input(800U);
    input.tracking_valid = 0U;
    input.timed_out = 1U;
    DynamicRunTask_Update(&task, &input, &actions);
    assert(actions.hard_stop == 0U);
    input = build_ready_input(1400U);
    input.tracking_valid = 0U;
    input.timed_out = 1U;
    DynamicRunTask_Update(&task, &input, &actions);
    assert(actions.hard_stop != 0U);
    assert(task.state == DYNAMIC_RUN_STATE_FAILED);
    assert(task.result == DYNAMIC_RUN_RESULT_FAILED);
    assert(task.failure == DYNAMIC_RUN_FAILURE_VISION);
    assert(DynamicRunTask_IsActive(&task) == 0U);
}

static void test_abcd_braking_ignores_post_finish_ball_loss(void)
{
    DynamicRunTask task;
    DynamicRunTaskConfig config = build_config();
    DynamicRunTaskInput input;
    DynamicRunTaskActions actions;

    assert(DynamicRunTask_Init(&task, &config) != 0U);
    start_running_task(&task, DYNAMIC_RUN_MODE_ABCD);

    input = build_ready_input(1000U);
    input.route_passed = 1U;
    input.travelled_distance_cm = 614.2f;
    DynamicRunTask_Update(&task, &input, &actions);
    assert(task.state == DYNAMIC_RUN_STATE_BRAKING);
    assert(actions.request_soft_stop != 0U);

    input = build_ready_input(1800U);
    input.measurement_accepted = 0U;
    input.balance_enabled = 0U;
    input.tracking_valid = 0U;
    input.timed_out = 1U;
    input.attitude_valid = 0U;
    input.imu_fault = 1U;
    input.motion_stopped = 0U;
    DynamicRunTask_Update(&task, &input, &actions);
    assert(actions.hard_stop == 0U);
    assert(task.state == DYNAMIC_RUN_STATE_BRAKING);

    input.now_ms = 2000U;
    input.motion_stopped = 1U;
    DynamicRunTask_Update(&task, &input, &actions);
    assert(actions.enter_stop_mode != 0U);
    assert(task.state == DYNAMIC_RUN_STATE_COMPLETE);
}

int main(void)
{
    test_rejects_invalid_config();
    test_ready_gate_requires_continuous_samples();
    test_prepare_timeout_reports_hard_failure();
    test_pass_records_metrics_and_soft_stops();
    test_single_overrange_sample_does_not_fail_result();
    test_pass_has_priority_at_time_limit();
    test_timeout_requests_soft_stop();
    test_braking_fault_preserves_frozen_time();
    test_running_vision_fault_debounced();
    test_abcd_braking_ignores_post_finish_ball_loss();
    puts("第四、五题动态任务状态机测试通过");
    return 0;
}
