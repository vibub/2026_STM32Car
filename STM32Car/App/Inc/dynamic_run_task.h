#ifndef DYNAMIC_RUN_TASK_H
#define DYNAMIC_RUN_TASK_H

#include <stdint.h>

typedef enum {
    DYNAMIC_RUN_MODE_NONE = 0,
    DYNAMIC_RUN_MODE_AB,
    DYNAMIC_RUN_MODE_ABCD
} DynamicRunMode;

typedef enum {
    DYNAMIC_RUN_STATE_IDLE = 0,
    DYNAMIC_RUN_STATE_PREPARING,
    DYNAMIC_RUN_STATE_RUNNING,
    DYNAMIC_RUN_STATE_BRAKING,
    DYNAMIC_RUN_STATE_COMPLETE,
    DYNAMIC_RUN_STATE_FAILED
} DynamicRunState;

typedef enum {
    DYNAMIC_RUN_RESULT_NONE = 0,
    DYNAMIC_RUN_RESULT_PASSED,
    DYNAMIC_RUN_RESULT_PASSED_OVERRANGE,
    DYNAMIC_RUN_RESULT_TIMEOUT,
    DYNAMIC_RUN_RESULT_FAILED
} DynamicRunResult;

typedef enum {
    DYNAMIC_RUN_FAILURE_NONE = 0,
    DYNAMIC_RUN_FAILURE_IMU,
    DYNAMIC_RUN_FAILURE_MAPPING,
    DYNAMIC_RUN_FAILURE_PREPARE_TIMEOUT,
    DYNAMIC_RUN_FAILURE_MOTION_START,
    DYNAMIC_RUN_FAILURE_BALANCE_DISABLED,
    DYNAMIC_RUN_FAILURE_VISION,
    DYNAMIC_RUN_FAILURE_LINE_SENSOR,
    DYNAMIC_RUN_FAILURE_LINE_LOST,
    DYNAMIC_RUN_FAILURE_MOTOR,
    DYNAMIC_RUN_FAILURE_RUN_TIMEOUT
} DynamicRunFailure;

typedef struct {
    uint32_t prepare_timeout_ms;          /* 等待中心稳定的最长时间，单位为毫秒。 */
    uint32_t ready_hold_ms;               /* 中心条件需要连续保持的时间，单位为毫秒。 */
    uint32_t ready_sample_gap_ms;         /* 准备阶段相邻视觉样本允许的最大间隔。 */
    uint32_t ab_timeout_ms;               /* 第四题从起步到通过 B 的时间限制。 */
    uint32_t abcd_timeout_ms;             /* 第五题从起步到通过 A 的时间限制。 */
    float ready_position_limit_mm;        /* 允许起步的钢球位置误差绝对值上限。 */
    float ready_speed_limit_mm_s;         /* 允许起步的钢球速度绝对值上限。 */
    float warning_limit_mm;               /* 触发串口屏红圈的误差绝对值门槛。 */
    float warning_clear_mm;               /* 清除红圈所需回到的误差绝对值。 */
    uint8_t warning_clear_samples;        /* 清除红圈需要连续满足的视觉样本数。 */
    uint8_t overrange_confirm_samples;    /* 锁存超差结果需要连续越界的样本数。 */
} DynamicRunTaskConfig;

typedef struct {
    uint32_t now_ms;                      /* 当前系统时间，单位为毫秒。 */
    uint8_t balance_enabled;              /* 钢球平衡控制器是否已经实际启用。 */
    uint8_t tracking_valid;               /* K230 钢球跟踪是否有效。 */
    uint8_t timed_out;                    /* K230 有效数据是否已经超时。 */
    uint8_t attitude_valid;               /* 水管 IMU 姿态是否有效。 */
    uint8_t imu_fault;                    /* 水管 IMU 是否报告故障。 */
    uint8_t mapping_fault;                /* 九点实测映射是否报告故障。 */
    uint8_t measurement_accepted;         /* 本次是否收到控制器接受的新视觉样本。 */
    uint8_t evaluate_completion;          /* 本次是否在底盘采样后判断通过和超时。 */
    uint8_t route_passed;                 /* 本次是否检测到 B 点或返程 A 点。 */
    uint8_t motion_stopped;               /* 平滑速度规划是否已经柔和停稳。 */
    float error_mm;                       /* 钢球相对中心目标的位置误差。 */
    float velocity_mm_s;                  /* 钢球沿水管方向的估计速度。 */
    float travelled_distance_cm;          /* 本次任务的底盘累计行驶里程。 */
} DynamicRunTaskInput;

typedef struct {
    uint8_t start_motion;                 /* 请求 App 启动对应的底盘速度规划。 */
    uint8_t request_soft_stop;            /* 请求 App 进入受限减速度柔和停车。 */
    uint8_t hard_stop;                    /* 请求 App 立即执行安全停车。 */
    uint8_t enter_stop_mode;              /* 柔和停车完成后请求进入停车模式。 */
    uint8_t warning_changed;              /* 红圈警告状态在本次更新中发生变化。 */
    uint8_t result_ready;                 /* 通过、超差或超时结果已经冻结。 */
} DynamicRunTaskActions;

typedef struct {
    DynamicRunTaskConfig config;
    DynamicRunMode mode;
    DynamicRunState state;
    DynamicRunResult result;
    DynamicRunFailure failure;
    uint8_t ready_window_active;
    uint8_t ready_sample_count;
    uint8_t motion_start_requested;
    uint8_t warning_active;
    uint8_t warning_clear_count;
    uint8_t overrange_consecutive_count;
    uint8_t overrange_episode_active;
    uint8_t overrange_latched;
    uint8_t overlimit_time_active;
    uint8_t update_time_ready;
    uint32_t prepare_started_ms;
    uint32_t ready_started_ms;
    uint32_t last_ready_sample_ms;
    uint32_t ready_stable_elapsed_ms;
    uint32_t run_started_ms;
    uint32_t elapsed_ms;
    uint32_t last_update_ms;
    uint32_t accepted_sample_count;
    uint32_t overrange_event_count;
    uint32_t overrange_duration_ms;
    float travelled_distance_cm;
    float maximum_abs_error_mm;
    float squared_error_sum_mm2;
} DynamicRunTask;

uint8_t DynamicRunTask_Init(DynamicRunTask *task,
                            const DynamicRunTaskConfig *config);
uint8_t DynamicRunTask_Start(DynamicRunTask *task,
                             DynamicRunMode mode,
                             uint32_t now_ms);
void DynamicRunTask_ConfirmMotionStarted(DynamicRunTask *task,
                                         uint32_t now_ms);
void DynamicRunTask_Update(DynamicRunTask *task,
                           const DynamicRunTaskInput *input,
                           DynamicRunTaskActions *actions);
void DynamicRunTask_Fail(DynamicRunTask *task,
                         DynamicRunFailure failure,
                         uint32_t now_ms);
void DynamicRunTask_Cancel(DynamicRunTask *task);
uint8_t DynamicRunTask_IsActive(const DynamicRunTask *task);
uint8_t DynamicRunTask_IsDriving(const DynamicRunTask *task);
float DynamicRunTask_GetRmsErrorMm(const DynamicRunTask *task);

#endif /* DYNAMIC_RUN_TASK_H */
