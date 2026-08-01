#include "dynamic_run_task.h"

#include <math.h>
#include <string.h>

static float DynamicRunTask_Abs(float value)
{
    return (value < 0.0f) ? -value : value;
}

static uint8_t DynamicRunTask_ConfigIsValid(
    const DynamicRunTaskConfig *config)
{
    if (config == 0) {
        return 0U;
    }
    return (uint8_t) (
        (config->prepare_timeout_ms > 0U) &&
        (config->ready_hold_ms > 0U) &&
        (config->ready_sample_gap_ms > 0U) &&
        (config->ab_timeout_ms > 0U) &&
        (config->abcd_timeout_ms > 0U) &&
        (config->ready_position_limit_mm > 0.0f) &&
        (config->ready_speed_limit_mm_s > 0.0f) &&
        (config->warning_limit_mm > 0.0f) &&
        (config->warning_clear_mm >= 0.0f) &&
        (config->warning_clear_mm < config->warning_limit_mm) &&
        (config->warning_clear_samples > 0U) &&
        (config->overrange_confirm_samples > 0U) &&
        (config->imu_fault_debounce_ms > 0U) &&
        (config->vision_fault_debounce_ms > 0U));
}

static void DynamicRunTask_ResetRuntime(DynamicRunTask *task)
{
    DynamicRunTaskConfig config;

    config = task->config;
    memset(task, 0, sizeof(*task));
    task->config = config;
    task->mode = DYNAMIC_RUN_MODE_NONE;
    task->state = DYNAMIC_RUN_STATE_IDLE;
    task->result = DYNAMIC_RUN_RESULT_NONE;
    task->failure = DYNAMIC_RUN_FAILURE_NONE;
}

static void DynamicRunTask_ResetReadyWindow(DynamicRunTask *task)
{
    task->ready_window_active = 0U;
    task->ready_sample_count = 0U;
    task->ready_started_ms = 0U;
    task->last_ready_sample_ms = 0U;
    task->ready_stable_elapsed_ms = 0U;
}

static void DynamicRunTask_ClearWarning(DynamicRunTask *task,
                                        DynamicRunTaskActions *actions)
{
    if (task->warning_active != 0U) {
        task->warning_active = 0U;
        if (actions != 0) {
            actions->warning_changed = 1U;
        }
    }
    task->warning_clear_count = 0U;
    task->overrange_consecutive_count = 0U;
    task->overrange_episode_active = 0U;
    task->overlimit_time_active = 0U;
}

static void DynamicRunTask_SetFailure(DynamicRunTask *task,
                                      DynamicRunFailure failure,
                                      uint32_t now_ms,
                                      DynamicRunTaskActions *actions)
{
    if (task->state == DYNAMIC_RUN_STATE_RUNNING) {
        task->elapsed_ms = (uint32_t) (now_ms - task->run_started_ms);
    }
    task->failure = failure;
    task->result = DYNAMIC_RUN_RESULT_FAILED;
    task->state = DYNAMIC_RUN_STATE_FAILED;
    DynamicRunTask_ClearWarning(task, actions);
    if (actions != 0) {
        actions->hard_stop = 1U;
        actions->result_ready = 1U;
    }
}

static uint32_t DynamicRunTask_GetTimeoutMs(const DynamicRunTask *task)
{
    return (task->mode == DYNAMIC_RUN_MODE_AB) ?
           task->config.ab_timeout_ms : task->config.abcd_timeout_ms;
}

static void DynamicRunTask_UpdateReadyWindow(
    DynamicRunTask *task,
    const DynamicRunTaskInput *input,
    DynamicRunTaskActions *actions)
{
    uint8_t stable_sample;

    /* 只有连续到达的新视觉样本才能完成 300 ms 起步确认。 */
    if (input->measurement_accepted == 0U) {
        if ((task->ready_window_active != 0U) &&
            ((uint32_t) (input->now_ms - task->last_ready_sample_ms) >
             task->config.ready_sample_gap_ms)) {
            DynamicRunTask_ResetReadyWindow(task);
        } else if (task->ready_window_active != 0U) {
            task->ready_stable_elapsed_ms =
                (uint32_t) (input->now_ms - task->ready_started_ms);
        }
        return;
    }

    stable_sample = (uint8_t) (
        (input->balance_enabled != 0U) &&
        (input->tracking_valid != 0U) &&
        (input->timed_out == 0U) &&
        (input->attitude_valid != 0U) &&
        (DynamicRunTask_Abs(input->error_mm) <=
         task->config.ready_position_limit_mm) &&
        (DynamicRunTask_Abs(input->velocity_mm_s) <=
         task->config.ready_speed_limit_mm_s));
    if (stable_sample == 0U) {
        DynamicRunTask_ResetReadyWindow(task);
        return;
    }

    if ((task->ready_window_active == 0U) ||
        ((uint32_t) (input->now_ms - task->last_ready_sample_ms) >
         task->config.ready_sample_gap_ms)) {
        task->ready_window_active = 1U;
        task->ready_sample_count = 1U;
        task->ready_started_ms = input->now_ms;
    } else if (task->ready_sample_count < 0xFFU) {
        task->ready_sample_count++;
    }
    task->last_ready_sample_ms = input->now_ms;
    task->ready_stable_elapsed_ms =
        (uint32_t) (input->now_ms - task->ready_started_ms);

    if ((task->motion_start_requested == 0U) &&
        (task->ready_sample_count >= 2U) &&
        (task->ready_stable_elapsed_ms >= task->config.ready_hold_ms)) {
        task->motion_start_requested = 1U;
        actions->start_motion = 1U;
    }
}

static void DynamicRunTask_UpdateMetrics(
    DynamicRunTask *task,
    const DynamicRunTaskInput *input,
    DynamicRunTaskActions *actions)
{
    float abs_error_mm;

    task->travelled_distance_cm = input->travelled_distance_cm;
    if (input->measurement_accepted == 0U) {
        return;
    }

    abs_error_mm = DynamicRunTask_Abs(input->error_mm);
    task->accepted_sample_count++;
    task->squared_error_sum_mm2 += input->error_mm * input->error_mm;
    if (abs_error_mm > task->maximum_abs_error_mm) {
        task->maximum_abs_error_mm = abs_error_mm;
    }

    /* 红圈单帧即提示，最终超差结果则按配置连续样本数去抖。 */
    if (abs_error_mm >= task->config.warning_limit_mm) {
        task->overlimit_time_active = 1U;
        task->warning_clear_count = 0U;
        if (task->warning_active == 0U) {
            task->warning_active = 1U;
            actions->warning_changed = 1U;
        }
        if (task->overrange_consecutive_count < 0xFFU) {
            task->overrange_consecutive_count++;
        }
        if ((task->overrange_consecutive_count >=
             task->config.overrange_confirm_samples) &&
            (task->overrange_episode_active == 0U)) {
            task->overrange_episode_active = 1U;
            task->overrange_latched = 1U;
            task->overrange_event_count++;
        }
        return;
    }

    task->overlimit_time_active = 0U;
    task->overrange_consecutive_count = 0U;
    if ((task->warning_active != 0U) &&
        (abs_error_mm <= task->config.warning_clear_mm)) {
        if (task->warning_clear_count <
            task->config.warning_clear_samples) {
            task->warning_clear_count++;
        }
        if (task->warning_clear_count >=
            task->config.warning_clear_samples) {
            task->warning_active = 0U;
            task->warning_clear_count = 0U;
            task->overrange_episode_active = 0U;
            actions->warning_changed = 1U;
        }
    } else if (abs_error_mm > task->config.warning_clear_mm) {
        task->warning_clear_count = 0U;
    }
}

static uint8_t DynamicRunTask_VisionUnsafeDebounced(
    DynamicRunTask *task,
    const DynamicRunTaskInput *input)
{
    uint8_t vision_lost = (uint8_t) ((input->tracking_valid == 0U) ||
                                     (input->timed_out != 0U));

    /* 视觉短暂失联（K230 偶发丢帧、处理卡顿）先保持运行让控制器用
     * 预测值维持，持续超过配置时长仍未恢复才判视觉故障停车。 */
    if (vision_lost != 0U) {
        if (task->vision_unsafe_active == 0U) {
            task->vision_unsafe_active = 1U;
            task->vision_unsafe_since_ms = input->now_ms;
        }
        return (uint8_t) ((uint32_t) (input->now_ms -
                                      task->vision_unsafe_since_ms) >=
                          task->config.vision_fault_debounce_ms);
    }
    task->vision_unsafe_active = 0U;
    task->vision_unsafe_since_ms = 0U;
    return 0U;
}

static uint8_t DynamicRunTask_ImuUnsafeDebounced(
    DynamicRunTask *task,
    const DynamicRunTaskInput *input)
{
    /* IMU 已由平衡控制器去抖后报告硬故障，必须立即判失败停车，
     * 不允许带着已关闭的平衡继续行驶。 */
    if (input->imu_fault != 0U) {
        task->imu_unsafe_active = 0U;
        task->imu_unsafe_since_ms = 0U;
        return 1U;
    }

    /* 姿态数据短暂不新鲜（如起步变速造成 100 ms 超时）先保持运行让内环
     * 自纠，持续超过配置时长仍未恢复才判 IMU 故障，容忍瞬态。 */
    if (input->attitude_valid == 0U) {
        if (task->imu_unsafe_active == 0U) {
            task->imu_unsafe_active = 1U;
            task->imu_unsafe_since_ms = input->now_ms;
        }
        return (uint8_t) ((uint32_t) (input->now_ms -
                                      task->imu_unsafe_since_ms) >=
                          task->config.imu_fault_debounce_ms);
    }
    task->imu_unsafe_active = 0U;
    task->imu_unsafe_since_ms = 0U;
    return 0U;
}

static uint8_t DynamicRunTask_CheckDrivingFaults(
    DynamicRunTask *task,
    const DynamicRunTaskInput *input,
    DynamicRunTaskActions *actions)
{
    if (DynamicRunTask_ImuUnsafeDebounced(task, input) != 0U) {
        DynamicRunTask_SetFailure(task, DYNAMIC_RUN_FAILURE_IMU,
                                  input->now_ms, actions);
        return 1U;
    }
    if (input->mapping_fault != 0U) {
        DynamicRunTask_SetFailure(task, DYNAMIC_RUN_FAILURE_MAPPING,
                                  input->now_ms, actions);
        return 1U;
    }
    if (DynamicRunTask_VisionUnsafeDebounced(task, input) != 0U) {
        DynamicRunTask_SetFailure(task, DYNAMIC_RUN_FAILURE_VISION,
                                  input->now_ms, actions);
        return 1U;
    }
    if (input->balance_enabled == 0U) {
        DynamicRunTask_SetFailure(task,
                                  DYNAMIC_RUN_FAILURE_BALANCE_DISABLED,
                                  input->now_ms, actions);
        return 1U;
    }
    return 0U;
}

uint8_t DynamicRunTask_Init(DynamicRunTask *task,
                            const DynamicRunTaskConfig *config)
{
    if ((task == 0) || (DynamicRunTask_ConfigIsValid(config) == 0U)) {
        return 0U;
    }

    memset(task, 0, sizeof(*task));
    task->config = *config;
    task->mode = DYNAMIC_RUN_MODE_NONE;
    task->state = DYNAMIC_RUN_STATE_IDLE;
    return 1U;
}

uint8_t DynamicRunTask_Start(DynamicRunTask *task,
                             DynamicRunMode mode,
                             uint32_t now_ms)
{
    if ((task == 0) ||
        ((mode != DYNAMIC_RUN_MODE_AB) &&
         (mode != DYNAMIC_RUN_MODE_ABCD)) ||
        (DynamicRunTask_IsActive(task) != 0U)) {
        return 0U;
    }

    DynamicRunTask_ResetRuntime(task);
    task->mode = mode;
    task->state = DYNAMIC_RUN_STATE_PREPARING;
    task->prepare_started_ms = now_ms;
    task->last_update_ms = now_ms;
    task->update_time_ready = 1U;
    return 1U;
}

void DynamicRunTask_ConfirmMotionStarted(DynamicRunTask *task,
                                         uint32_t now_ms)
{
    if ((task == 0) ||
        (task->state != DYNAMIC_RUN_STATE_PREPARING) ||
        (task->motion_start_requested == 0U)) {
        return;
    }

    task->state = DYNAMIC_RUN_STATE_RUNNING;
    task->result = DYNAMIC_RUN_RESULT_NONE;
    task->failure = DYNAMIC_RUN_FAILURE_NONE;
    task->run_started_ms = now_ms;
    task->elapsed_ms = 0U;
    task->last_update_ms = now_ms;
    task->update_time_ready = 1U;
    task->travelled_distance_cm = 0.0f;
    task->maximum_abs_error_mm = 0.0f;
    task->squared_error_sum_mm2 = 0.0f;
    task->accepted_sample_count = 0U;
    task->overrange_event_count = 0U;
    task->overrange_duration_ms = 0U;
    task->warning_active = 0U;
    task->warning_clear_count = 0U;
    task->overrange_consecutive_count = 0U;
    task->overrange_episode_active = 0U;
    task->overrange_latched = 0U;
    task->overlimit_time_active = 0U;
}

void DynamicRunTask_Update(DynamicRunTask *task,
                           const DynamicRunTaskInput *input,
                           DynamicRunTaskActions *actions)
{
    uint32_t update_elapsed_ms;
    uint32_t run_timeout_ms;

    if (actions != 0) {
        memset(actions, 0, sizeof(*actions));
    }
    if ((task == 0) || (input == 0) || (actions == 0)) {
        return;
    }

    if (task->update_time_ready != 0U) {
        update_elapsed_ms = (uint32_t) (input->now_ms - task->last_update_ms);
        if ((task->state == DYNAMIC_RUN_STATE_RUNNING) &&
            (task->overlimit_time_active != 0U)) {
            task->overrange_duration_ms += update_elapsed_ms;
        }
    }
    task->last_update_ms = input->now_ms;
    task->update_time_ready = 1U;

    if (task->state == DYNAMIC_RUN_STATE_PREPARING) {
        if (DynamicRunTask_ImuUnsafeDebounced(task, input) != 0U) {
            DynamicRunTask_SetFailure(task, DYNAMIC_RUN_FAILURE_IMU,
                                      input->now_ms, actions);
            return;
        }
        if (input->mapping_fault != 0U) {
            DynamicRunTask_SetFailure(task, DYNAMIC_RUN_FAILURE_MAPPING,
                                      input->now_ms, actions);
            return;
        }
        if ((uint32_t) (input->now_ms - task->prepare_started_ms) >=
            task->config.prepare_timeout_ms) {
            DynamicRunTask_SetFailure(
                task, DYNAMIC_RUN_FAILURE_PREPARE_TIMEOUT,
                input->now_ms, actions);
            return;
        }
        DynamicRunTask_UpdateReadyWindow(task, input, actions);
        return;
    }

    if (task->state == DYNAMIC_RUN_STATE_RUNNING) {
        task->elapsed_ms = (uint32_t) (input->now_ms - task->run_started_ms);
        DynamicRunTask_UpdateMetrics(task, input, actions);
        if (DynamicRunTask_CheckDrivingFaults(task, input, actions) != 0U) {
            return;
        }
        /* 通过和超时必须在本周期底盘采样完成后统一判断。 */
        if (input->evaluate_completion == 0U) {
            return;
        }

        /* 同一控制周期既到达时间边界又通过时，优先认定通过。 */
        if (input->route_passed != 0U) {
            task->result = (task->overrange_latched != 0U) ?
                           DYNAMIC_RUN_RESULT_PASSED_OVERRANGE :
                           DYNAMIC_RUN_RESULT_PASSED;
            task->failure = DYNAMIC_RUN_FAILURE_NONE;
            task->state = DYNAMIC_RUN_STATE_BRAKING;
            DynamicRunTask_ClearWarning(task, actions);
            actions->request_soft_stop = 1U;
            actions->result_ready = 1U;
            return;
        }

        run_timeout_ms = DynamicRunTask_GetTimeoutMs(task);
        if (task->elapsed_ms >= run_timeout_ms) {
            task->result = DYNAMIC_RUN_RESULT_TIMEOUT;
            task->failure = DYNAMIC_RUN_FAILURE_RUN_TIMEOUT;
            task->state = DYNAMIC_RUN_STATE_BRAKING;
            DynamicRunTask_ClearWarning(task, actions);
            actions->request_soft_stop = 1U;
            actions->result_ready = 1U;
        }
        return;
    }

    if (task->state == DYNAMIC_RUN_STATE_BRAKING) {
        if (DynamicRunTask_CheckDrivingFaults(task, input, actions) != 0U) {
            return;
        }
        if ((input->evaluate_completion != 0U) &&
            (input->motion_stopped != 0U)) {
            task->state = DYNAMIC_RUN_STATE_COMPLETE;
            actions->enter_stop_mode = 1U;
        }
    }
}

void DynamicRunTask_Fail(DynamicRunTask *task,
                         DynamicRunFailure failure,
                         uint32_t now_ms)
{
    if ((task == 0) || (DynamicRunTask_IsActive(task) == 0U)) {
        return;
    }
    DynamicRunTask_SetFailure(task, failure, now_ms, 0);
}

void DynamicRunTask_Cancel(DynamicRunTask *task)
{
    if (task != 0) {
        DynamicRunTask_ResetRuntime(task);
    }
}

uint8_t DynamicRunTask_IsActive(const DynamicRunTask *task)
{
    if (task == 0) {
        return 0U;
    }
    return (uint8_t) (
        (task->state == DYNAMIC_RUN_STATE_PREPARING) ||
        (task->state == DYNAMIC_RUN_STATE_RUNNING) ||
        (task->state == DYNAMIC_RUN_STATE_BRAKING));
}

uint8_t DynamicRunTask_IsDriving(const DynamicRunTask *task)
{
    if (task == 0) {
        return 0U;
    }
    return (uint8_t) (
        (task->state == DYNAMIC_RUN_STATE_RUNNING) ||
        (task->state == DYNAMIC_RUN_STATE_BRAKING));
}

float DynamicRunTask_GetRmsErrorMm(const DynamicRunTask *task)
{
    if ((task == 0) || (task->accepted_sample_count == 0U)) {
        return 0.0f;
    }
    return sqrtf(task->squared_error_sum_mm2 /
                 (float) task->accepted_sample_count);
}
