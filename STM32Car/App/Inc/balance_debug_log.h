#ifndef BALANCE_DEBUG_LOG_H
#define BALANCE_DEBUG_LOG_H

#include <stdint.h>

#include "app_config.h"
#include "ball_balance_control.h"
#include "pipe_attitude_control.h"

#define BALANCE_DEBUG_LOG_MAGIC          0x424C4F47UL /* 日志文件标识“BLOG”。 */
#define BALANCE_DEBUG_LOG_FORMAT_VERSION          1UL /* 二进制结构版本号。 */
#define BALANCE_DEBUG_LOG_PERIOD_MS               20U /* 同步采样周期，单位为毫秒。 */
#define BALANCE_DEBUG_LOG_CAPACITY               500U /* 20 ms 周期下保存10秒。 */

#define BALANCE_DEBUG_LOG_COMMAND_NONE             0U /* 不改变当前采集状态。 */
#define BALANCE_DEBUG_LOG_COMMAND_START            1U /* 清空计数并开始新一轮采集。 */
#define BALANCE_DEBUG_LOG_COMMAND_STOP             2U /* 立即冻结当前已采集数据。 */

#define BALANCE_DEBUG_LOG_FLAG_ENABLE_REQUESTED    (1UL << 0)  /* 已请求启用平衡。 */
#define BALANCE_DEBUG_LOG_FLAG_ENABLED             (1UL << 1)  /* 平衡闭环正在输出。 */
#define BALANCE_DEBUG_LOG_FLAG_ENABLE_PENDING      (1UL << 2)  /* 正在等待启用条件。 */
#define BALANCE_DEBUG_LOG_FLAG_BALL_SAMPLE_VALID   (1UL << 3)  /* K230 最新原始帧有效。 */
#define BALANCE_DEBUG_LOG_FLAG_TRACKING_VALID      (1UL << 4)  /* 位置跟踪当前有效。 */
#define BALANCE_DEBUG_LOG_FLAG_TIMED_OUT           (1UL << 5)  /* 视觉位置已经超时。 */
#define BALANCE_DEBUG_LOG_FLAG_MEASUREMENT_ACCEPTED (1UL << 6) /* 本轮接受了位置测量。 */
#define BALANCE_DEBUG_LOG_FLAG_ATTITUDE_VALID      (1UL << 7)  /* IMU 姿态当前有效。 */
#define BALANCE_DEBUG_LOG_FLAG_IMU_FAULT           (1UL << 8)  /* 姿态链路存在故障。 */
#define BALANCE_DEBUG_LOG_FLAG_MAPPING_VALID       (1UL << 9)  /* 九点映射当前有效。 */
#define BALANCE_DEBUG_LOG_FLAG_MAPPING_FAULT       (1UL << 10) /* 九点映射存在故障。 */
#define BALANCE_DEBUG_LOG_FLAG_INTEGRAL_ACTIVE     (1UL << 11) /* 位置积分项正在累积。 */
#define BALANCE_DEBUG_LOG_FLAG_BRAKING_ACTIVE      (1UL << 12) /* 预测制动逻辑正在工作。 */
#define BALANCE_DEBUG_LOG_FLAG_ZERO_CROSSED        (1UL << 13) /* 小球已经越过目标点。 */
#define BALANCE_DEBUG_LOG_FLAG_TURNAROUND_FREEZE   (1UL << 14) /* 换向冻结逻辑正在工作。 */
#define BALANCE_DEBUG_LOG_FLAG_PIPE_SETTLED        (1UL << 15) /* 水管姿态已经稳定。 */
#define BALANCE_DEBUG_LOG_FLAG_ENDPOINT_HOLD       (1UL << 16) /* 端点保持逻辑已启用。 */
#define BALANCE_DEBUG_LOG_FLAG_TRANSPORT_MODE      (1UL << 17) /* 底盘运输前馈已启用。 */
#define BALANCE_DEBUG_LOG_FLAG_ACCEL_TRUSTED       (1UL << 18) /* 当前信任加速度计姿态。 */
#define BALANCE_DEBUG_LOG_FLAG_SERVO_OVERRIDE      (1UL << 19) /* 标定逻辑正在接管舵机。 */
#define BALANCE_DEBUG_LOG_FLAG_CALIBRATION_VALID   (1UL << 20) /* 水管九点标定有效。 */

typedef enum {
    BALANCE_DEBUG_LOG_STATE_IDLE = 0,
    BALANCE_DEBUG_LOG_STATE_RECORDING,
    BALANCE_DEBUG_LOG_STATE_FULL,
    BALANCE_DEBUG_LOG_STATE_STOPPED
} BalanceDebugLogState;

typedef struct {
    uint32_t now_ms;
    uint32_t imu_timestamp_ms;
    int32_t raw_position_d10_mm;
    uint16_t ball_sequence;
    uint8_t ball_quality;
    uint8_t ball_sample_valid;
    float imu_accel_y_g;
    float imu_accel_z_g;
    float imu_gyro_x_dps;
    float actual_servo_angle_deg;
    uint32_t servo_pulse_us;
} BalanceDebugLogInput;

/* 每个元素是一帧同一时刻的完整控制快照，禁止拆成不同步的平行数组。 */
typedef struct {
    uint32_t timestamp_ms;
    uint32_t imu_timestamp_ms;
    int32_t raw_position_d10_mm;
    uint32_t ball_sequence;
    uint32_t ball_quality;
    uint32_t servo_pulse_us;
    uint32_t flags;
    uint32_t state_word;
    float target_mm;
    float raw_position_mm;
    float filtered_position_mm;
    float predicted_position_mm;
    float predicted_accel_mm_s2;
    float velocity_mm_s;
    float integral_mm_s;
    float error_mm;
    float p_term_deg;
    float i_term_deg;
    float d_term_deg;
    float brake_term_deg;
    float desired_pipe_pitch_deg;
    float command_pipe_pitch_deg;
    float pipe_command_rate_dps;
    float measured_pipe_pitch_deg;
    float measured_pipe_rate_dps;
    float pipe_pitch_error_deg;
    float pipe_p_term_deg;
    float pipe_d_term_deg;
    float minimum_pipe_pitch_deg;
    float maximum_pipe_pitch_deg;
    float feedforward_model_pitch_deg;
    float desired_model_pitch_deg;
    float servo_feedforward_deg;
    float desired_angle_deg;
    float command_angle_deg;
    float actual_servo_angle_deg;
    float raw_accel_pitch_deg;
    float accel_pitch_deg;
    float raw_pitch_deg;
    float raw_pitch_rate_dps;
    float imu_accel_y_g;
    float imu_accel_z_g;
    float imu_gyro_x_dps;
    float transport_command_accel_cm_s2;
    float transport_imu_disturbance_deg;
    float transport_feedforward_pitch_deg;
    float transport_feedback_pitch_deg;
    float phase_pitch_deg;
    float stop_distance_mm;
} BalanceDebugLogSample;

/* 连续导出该结构即可，文件头包含解析所需的版本、长度和有效点数。 */
typedef struct {
    uint32_t magic;
    uint32_t format_version;
    uint32_t sample_size;
    uint32_t capacity;
    uint32_t count;
    uint32_t state;
    uint32_t period_ms;
    uint32_t buffer_size;
    BalanceDebugLogSample samples[BALANCE_DEBUG_LOG_CAPACITY];
} BalanceDebugLogBuffer;

#if APP_ENABLE_BALANCE_LOG
extern volatile uint32_t g_balance_debug_log_command;
extern BalanceDebugLogBuffer g_balance_debug_log;
#endif

void BalanceDebugLog_Init(void);
void BalanceDebugLog_Update(const BalanceDebugLogInput *input,
                            const BallBalanceOutput *balance_output,
                            const PipeAttitudeOutput *attitude_output);

#endif /* BALANCE_DEBUG_LOG_H */
