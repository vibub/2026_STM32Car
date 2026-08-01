#include "app_main.h"
#include "app_config.h"
#include "balance_debug_log.h"
#include "ball_balance_control.h"
#include "ball_position_protocol.h"
#include "ball_shuttle_control.h"
#include "board_time.h"
#include "chassis_accel_estimator.h"
#include "chassis_motion_profile.h"
#include "dynamic_run_task.h"
#include "encoder_driver.h"
#include "host_calibration_protocol.h"
#include "host_link_uart.h"
#include "imu_heading.h"
#include "infrared_line_control.h"
#include "infrared_line_sensor.h"
#include "lap_stop_detector.h"
#include "k230_uart.h"
#include "motor_driver.h"
#include "oled_display.h"
#include "pipe_attitude_control.h"
#include "protocol_tjc.h"
#include "servo_driver.h"
#include "tjc_uart.h"
#include "wheel_speed_pid.h"

#include "stm32h7xx_hal.h"

#include <string.h>

#define APP_BALL_POSITION_KP_MAX          0.200f /* 上位机允许的位置Kp安全上限。 */
#define APP_BALL_POSITION_KI_MAX          0.100f /* 上位机允许的位置Ki安全上限。 */
#define APP_BALL_POSITION_KD_MAX          0.200f /* 上位机允许的位置Kd安全上限。 */
#define APP_BALL_PIPE_KP_MAX              2.000f /* 上位机允许的姿态Kp安全上限。 */
#define APP_BALL_PIPE_KD_MAX_S            0.300f /* 上位机允许的姿态Kd安全上限，单位为秒。 */

/* Debug variables for IAR Watch. PID gain variables intentionally drive control. */
volatile uint8_t g_app_init_step;
volatile uint8_t g_app_line_command_valid;
volatile uint8_t g_app_motor_fault;
volatile uint8_t g_app_line_sensor_mask;
volatile uint8_t g_app_line_active_count;
volatile uint8_t g_app_line_lost;
volatile float g_app_left_target_cm_s;
volatile float g_app_right_target_cm_s;
volatile float g_app_left_speed_cm_s;
volatile float g_app_right_speed_cm_s;
volatile float g_app_left_pwm_ratio;
volatile float g_app_right_pwm_ratio;
volatile float g_app_line_error;
volatile float g_app_line_correction_cm_s;
volatile uint8_t g_app_line_sensor_raw;
volatile uint32_t g_app_line_sensor_i2c_error_count;
volatile uint8_t g_app_line_sensor_stale_count;
volatile uint8_t g_app_line_sensor_fault;
volatile float g_app_lap_distance_cm;
volatile uint8_t g_app_lap_left_start_line;
volatile uint8_t g_app_lap_stop_armed;
volatile uint8_t g_app_lap_full_black_count;
volatile uint8_t g_app_lap_completed;
volatile uint8_t g_app_ball_balance_enabled;
volatile uint8_t g_app_ball_enable_pending;
volatile uint8_t g_app_ball_imu_fault;
volatile uint8_t g_app_ball_mapping_fault;
volatile uint8_t g_app_ball_sample_valid;
volatile int32_t g_app_ball_position_d10_mm;
volatile uint8_t g_app_ball_quality;
volatile uint16_t g_app_ball_sequence;
volatile uint8_t g_app_ball_tracking_valid;
volatile uint8_t g_app_ball_timed_out;
volatile uint8_t g_app_ball_measurement_accepted;
volatile float g_app_ball_target_mm;
volatile float g_app_ball_filtered_position_mm;
volatile float g_app_ball_predicted_position_mm;
volatile float g_app_ball_predicted_accel_mm_s2;
volatile float g_app_ball_velocity_mm_s;
volatile float g_app_ball_error_mm;
volatile float g_app_ball_p_term_deg;
volatile float g_app_ball_i_term_deg;
volatile float g_app_ball_d_term_deg;
volatile float g_app_ball_brake_term_deg;
volatile uint8_t g_app_ball_transport_mode_enabled;
volatile uint8_t g_app_ball_transport_recovery_active;
volatile float g_app_ball_transport_command_accel_cm_s2;
volatile float g_app_ball_transport_position_deadband_mm;
volatile float g_app_ball_transport_imu_disturbance_deg;
volatile float g_app_ball_transport_feedforward_pitch_deg;
volatile float g_app_ball_transport_feedback_pitch_deg;
volatile float g_app_ball_integral_mm_s;
volatile uint8_t g_app_ball_integral_active;
volatile uint8_t g_app_ball_braking_active;
volatile uint8_t g_app_ball_zero_crossed;
volatile uint8_t g_app_ball_turnaround_freeze_active;
volatile uint8_t g_app_ball_pipe_settled;
volatile uint8_t g_app_ball_moving_sample_count;
volatile int8_t g_app_ball_moving_direction;
volatile uint8_t g_app_ball_kick_attempt_count;
volatile uint8_t g_app_ball_trajectory_phase;
volatile float g_app_ball_phase_pitch_deg;
volatile float g_app_ball_stop_distance_mm;
volatile float g_app_ball_kp = BALL_BALANCE_DEFAULT_KP_DEG_PER_MM;
volatile float g_app_ball_ki = BALL_BALANCE_DEFAULT_KI_DEG_PER_MM_S;
volatile float g_app_ball_kd = BALL_BALANCE_DEFAULT_KD_DEG_PER_MM_S;
volatile float g_app_ball_pipe_kp = BALL_BALANCE_DEFAULT_PIPE_KP;
volatile float g_app_ball_pipe_kd_s = BALL_BALANCE_DEFAULT_PIPE_KD_S;
volatile float g_app_ball_desired_pipe_pitch_deg;
volatile float g_app_ball_command_pipe_pitch_deg;
volatile float g_app_ball_pipe_command_rate_dps;
volatile float g_app_ball_measured_pipe_pitch_deg;
volatile float g_app_ball_measured_pipe_rate_dps;
volatile float g_app_ball_pipe_error_deg;
volatile float g_app_ball_pipe_p_term_deg;
volatile float g_app_ball_pipe_d_term_deg;
volatile float g_app_ball_model_scale;
volatile float g_app_ball_model_offset_deg;
volatile float g_app_ball_minimum_pipe_pitch_deg;
volatile float g_app_ball_maximum_pipe_pitch_deg;
volatile float g_app_ball_feedforward_model_pitch_deg;
volatile float g_app_ball_desired_model_pitch_deg;
volatile float g_app_ball_servo_feedforward_deg;
volatile float g_app_ball_desired_angle_deg;
volatile float g_app_ball_command_angle_deg;
volatile uint32_t g_app_ball_feedforward_pwm_us;
volatile uint32_t g_app_ball_desired_pwm_us;
volatile uint32_t g_app_ball_command_pwm_us;
volatile uint32_t g_app_ball_actual_pwm_us;
volatile uint8_t g_app_ball_endpoint_hold_enabled;
volatile uint8_t g_app_ball_shuttle_state;
volatile uint8_t g_app_ball_shuttle_start_reject;
volatile uint32_t g_app_ball_shuttle_elapsed_ms;
volatile float g_app_ball_shuttle_target_pipe_pitch_deg;
volatile float g_app_ball_shuttle_angle_magnitude_deg[
    BALL_SHUTTLE_PHASE_COUNT];
volatile uint32_t g_app_ball_shuttle_duration_ms[
    BALL_SHUTTLE_PHASE_COUNT];
volatile uint8_t g_app_pipe_calibration_state;
volatile uint8_t g_app_pipe_calibration_active;
volatile uint8_t g_app_pipe_calibration_valid;
volatile uint8_t g_app_pipe_zero_adjust_active;
volatile uint32_t g_app_pipe_zero_pwm_us;
volatile uint8_t g_app_pipe_attitude_valid;
volatile uint8_t g_app_pipe_accel_trusted;
volatile int8_t g_app_pipe_imu_pitch_sign;
volatile uint8_t g_app_pipe_calibration_point_index;
volatile uint16_t g_app_pipe_calibration_sample_count;
volatile float g_app_pipe_gyro_x_bias_dps;
volatile float g_app_pipe_accel_zero_deg;
volatile float g_app_pipe_raw_accel_pitch_deg;
volatile float g_app_pipe_raw_pitch_deg;
volatile float g_app_pipe_raw_pitch_rate_dps;
volatile float g_app_pipe_accel_pitch_deg;
volatile float g_app_pipe_pitch_deg;
volatile float g_app_pipe_pitch_rate_dps;
volatile float g_app_pipe_negative_pitch_deg;
volatile float g_app_pipe_center_pitch_deg;
volatile float g_app_pipe_positive_pitch_deg;
volatile float g_app_pipe_return_pitch_deg;
volatile float g_app_pipe_model_scale;
volatile float g_app_pipe_model_offset_deg;
volatile float g_app_pipe_mapping_pitch_deg[
    LINKAGE_KINEMATICS_CALIBRATION_POINT_COUNT];
volatile float g_app_pipe_calibration_target_pitch_deg;
volatile float g_app_pipe_servo_target_deg;
volatile float g_app_pipe_servo_override_deg;
volatile float g_app_chassis_speed_filtered_cm_s;
volatile float g_app_chassis_accel_raw_cm_s2;
volatile float g_app_chassis_accel_cm_s2;
volatile uint8_t g_app_motion_profile_state;
volatile uint8_t g_app_motion_distance_reached;
volatile float g_app_motion_target_speed_cm_s;
volatile float g_app_motion_command_speed_cm_s;
volatile float g_app_motion_command_accel_cm_s2;
volatile float g_app_motion_travelled_distance_cm;
volatile uint8_t g_app_dynamic_run_mode;
volatile uint8_t g_app_dynamic_run_state;
volatile uint8_t g_app_dynamic_run_result;
volatile uint8_t g_app_dynamic_run_failure;
volatile uint8_t g_app_dynamic_start_reject;
volatile uint8_t g_app_dynamic_overrange_latched;
volatile uint32_t g_app_dynamic_ready_elapsed_ms;
volatile uint32_t g_app_dynamic_elapsed_ms;
volatile uint32_t g_app_dynamic_sample_count;
volatile uint32_t g_app_dynamic_overrange_duration_ms;
volatile float g_app_dynamic_distance_cm;
volatile float g_app_dynamic_max_abs_error_mm;
volatile float g_app_dynamic_rms_error_mm;
volatile uint8_t g_app_ball_limit_warning;
volatile uint32_t g_app_ball_limit_warning_count;
volatile uint32_t g_app_k230_overflow_count;
volatile uint32_t g_app_k230_error_count;
volatile uint32_t g_app_k230_tx_frame_count;
volatile uint32_t g_app_k230_tx_drop_count;
volatile uint8_t g_app_lateral_valid;
volatile uint8_t g_app_turn_active;
volatile uint8_t g_app_lateral_bias_ready;
volatile int8_t g_app_lateral_imu_sign;
volatile float g_app_lateral_accel_x_bias_g;
volatile float g_app_lateral_yaw_rate_dps;
volatile float g_app_lateral_imu_accel_cm_s2;
volatile float g_app_lateral_kinematic_accel_cm_s2;
volatile float g_app_lateral_fused_accel_cm_s2;
volatile float g_app_abcd_wall_coupling_gain =
    APP_ABCD_WALL_COUPLING_GAIN;
volatile float g_app_abcd_wall_accel_cm_s2;
volatile float g_app_abcd_wall_pitch_deg;

static uint32_t s_last_control_ms;
static uint32_t s_oled_timer_start_ms;
static uint32_t s_oled_timer_elapsed_s;
static uint32_t s_oled_timer_last_display_s;
static uint8_t s_oled_timer_running;
static InfraredLineControl s_line_control;
static LapStopDetector s_lap_stop_detector;
static ChassisMotionProfile s_motion_profile;
static ChassisMotionOutput s_motion_output;
static ChassisAccelEstimator s_chassis_accel_estimator;
static ChassisAccelOutput s_chassis_accel_output;
static ChassisLateralEstimator s_chassis_lateral_estimator;
static ChassisLateralOutput s_chassis_lateral_output;
static BallBalanceControl s_ball_balance_control;
static BallBalanceOutput s_ball_balance_output;
static BallShuttleControl s_ball_shuttle_control;
static PipeAttitudeControl s_pipe_attitude_control;
static PipeAttitudeOutput s_pipe_attitude_output;
static uint8_t s_pipe_zero_adjust_active;
static uint32_t s_pipe_zero_pwm_us;
static WheelSpeedPid s_left_speed_pid;
static WheelSpeedPid s_right_speed_pid;
static uint8_t s_current_mode;
static uint8_t s_line_enabled;
static uint8_t s_last_line_sensor_mask;
static uint8_t s_has_line_sensor_sample;
static uint8_t s_line_sensor_failure_count;
static uint32_t s_last_k230_overflow_count;
static uint32_t s_last_k230_error_count;
static uint32_t s_pipe_attitude_timestamp_ms;
#if APP_ENABLE_BALANCE_LOG
static float s_balance_log_imu_accel_y_g;
static float s_balance_log_imu_accel_z_g;
static float s_balance_log_imu_gyro_x_dps;
#endif
static float s_chassis_accel_cm_s2;
static float s_latest_imu_accel_x_g;
static uint8_t s_lap_stop_enabled;
static uint8_t s_last_ball_active;
static uint8_t s_last_ball_imu_fault;
static uint32_t s_last_ball_sample_ms;
static uint8_t s_tjc_status_pending;
static Tjc_StatusText s_pending_tjc_status;
static uint8_t s_tjc_dynamic_target_pending;
static float s_pending_tjc_dynamic_target_mm;
static uint8_t s_tjc_shuttle_phase_pending;
static uint8_t s_pending_tjc_shuttle_phase;
static float s_pending_tjc_shuttle_target_deg;
static uint8_t s_host_calibration_enabled = APP_HOST_CALIBRATION_DEFAULT;

volatile uint8_t g_app_host_calibration_enabled;

static DynamicRunTask s_dynamic_run_task;
static uint8_t s_dynamic_route_passed;
static uint8_t s_ball_limit_warning;
static uint8_t s_k230_telemetry_session_active;
static uint32_t s_k230_telemetry_last_sample_ms;

static void App_DynamicRunTask(uint32_t now_ms,
                               uint8_t evaluate_completion,
                               uint8_t include_measurement);
static void App_DynamicRunFail(uint32_t now_ms,
                               DynamicRunFailure failure);
static uint8_t App_DynamicRunIsLapMode(void);

#define APP_LINE_SENSOR_FAILURE_LIMIT          3U /* 红外 I2C 连续读取失败后停车的次数门槛。 */
#define APP_DYNAMIC_TARGET_SAMPLE_MAX_AGE_MS 300U /* 第六题目标捕获允许的滤波位置最大年龄。 */
#define APP_BALL_SHUTTLE_CENTER_LIMIT_D10_MM  50  /* 第三题启动时最新单帧必须位于O点正负5毫米。 */
#define APP_BALL_SHUTTLE_SAMPLE_MAX_AGE_MS   200U /* 第三题启动允许的K230最新样本最大年龄。 */
#define APP_BALL_SHUTTLE_MIN_QUALITY          50U /* 第三题启动单帧所需的最低识别质量。 */

#define APP_K230_TELEMETRY_FLAG_LATERAL_VALID  (1U << 0)
#define APP_K230_TELEMETRY_FLAG_TURN_ACTIVE    (1U << 1)
#define APP_K230_TELEMETRY_FLAG_BIAS_READY     (1U << 2)
#define APP_K230_TELEMETRY_FLAG_IMU_SIGN_READY (1U << 3)
#define APP_K230_TELEMETRY_FLAG_BALL_VALID     (1U << 4)
#define APP_K230_TELEMETRY_FLAG_OVERRANGE      (1U << 5)
#define APP_K230_TELEMETRY_FLAG_ACCEL_TRUSTED  (1U << 6)
#define APP_K230_TELEMETRY_FLAG_WALL_ACTIVE    (1U << 7)

#define APP_SHUTTLE_NVM_ADDRESS       0x080E0000UL
#define APP_SHUTTLE_NVM_END_ADDRESS   0x08100000UL
#define APP_SHUTTLE_NVM_RECORD_SIZE   32U
#define APP_SHUTTLE_NVM_MAGIC         0x53485554UL
#define APP_SHUTTLE_NVM_VERSION       1UL
#define APP_SHUTTLE_NVM_VALID_MARKER  0xA55A3CC3UL

typedef struct {
    uint32_t magic;
    uint32_t version;
    uint32_t sequence;
    uint32_t duration_ms[BALL_SHUTTLE_PHASE_COUNT];
    uint32_t crc32;
    uint32_t valid_marker;
} AppShuttleNvmRecord;

static uint32_t App_ShuttleNvmCrc32(const AppShuttleNvmRecord *record)
{
    const uint8_t *bytes = (const uint8_t *) record;
    uint32_t crc = 0xFFFFFFFFUL;
    uint32_t byte_index;
    uint8_t bit_index;

    for (byte_index = 0U; byte_index < 24U; byte_index++) {
        crc ^= bytes[byte_index];
        for (bit_index = 0U; bit_index < 8U; bit_index++) {
            crc = ((crc & 1U) != 0U) ?
                  ((crc >> 1U) ^ 0xEDB88320UL) : (crc >> 1U);
        }
    }
    return ~crc;
}

static uint8_t App_ShuttleTimesAreValid(const uint32_t *duration_ms)
{
    uint32_t total_ms = 0U;
    uint8_t index;

    if (duration_ms == 0) {
        return 0U;
    }
    for (index = 0U; index < BALL_SHUTTLE_PHASE_COUNT; index++) {
        if ((duration_ms[index] < BALL_SHUTTLE_MIN_PHASE_TIME_MS) ||
            (duration_ms[index] > BALL_SHUTTLE_MAX_PHASE_TIME_MS)) {
            return 0U;
        }
        total_ms += duration_ms[index];
    }
    return (uint8_t) (total_ms <= BALL_SHUTTLE_MAX_TOTAL_TIME_MS);
}

static uint8_t App_ShuttleNvmRecordIsValid(
    const AppShuttleNvmRecord *record)
{
    return (uint8_t) ((record != 0) &&
                      (record->magic == APP_SHUTTLE_NVM_MAGIC) &&
                      (record->version == APP_SHUTTLE_NVM_VERSION) &&
                      (record->valid_marker ==
                       APP_SHUTTLE_NVM_VALID_MARKER) &&
                      (App_ShuttleTimesAreValid(record->duration_ms) != 0U) &&
                      (record->crc32 == App_ShuttleNvmCrc32(record)));
}

static uint8_t App_ShuttleNvmSlotIsErased(uint32_t address)
{
    const uint32_t *words = (const uint32_t *) address;
    uint8_t index;

    for (index = 0U; index < 8U; index++) {
        if (words[index] != 0xFFFFFFFFUL) {
            return 0U;
        }
    }
    return 1U;
}

static uint8_t App_ShuttleNvmFindLatest(AppShuttleNvmRecord *latest,
                                        uint32_t *next_address)
{
    uint32_t address;
    uint8_t found = 0U;

    if (next_address != 0) {
        *next_address = APP_SHUTTLE_NVM_END_ADDRESS;
    }
    for (address = APP_SHUTTLE_NVM_ADDRESS;
         address < APP_SHUTTLE_NVM_END_ADDRESS;
         address += APP_SHUTTLE_NVM_RECORD_SIZE) {
        const AppShuttleNvmRecord *candidate =
            (const AppShuttleNvmRecord *) address;

        if (App_ShuttleNvmSlotIsErased(address) != 0U) {
            if ((next_address != 0) &&
                (*next_address == APP_SHUTTLE_NVM_END_ADDRESS)) {
                *next_address = address;
            }
            continue;
        }
        if ((App_ShuttleNvmRecordIsValid(candidate) != 0U) &&
            ((found == 0U) ||
             ((int32_t) (candidate->sequence - latest->sequence) > 0))) {
            if (latest != 0) {
                *latest = *candidate;
            }
            found = 1U;
        }
    }
    return found;
}

static uint8_t App_ShuttleNvmEraseSector(void)
{
    FLASH_EraseInitTypeDef erase;
    uint32_t sector_error = 0U;
    HAL_StatusTypeDef status;

    erase.TypeErase = FLASH_TYPEERASE_SECTORS;
    erase.Banks = FLASH_BANK_1;
    erase.Sector = FLASH_SECTOR_7;
    erase.NbSectors = 1U;
    erase.VoltageRange = FLASH_VOLTAGE_RANGE_3;
    status = HAL_FLASHEx_Erase(&erase, &sector_error);
    return (uint8_t) ((status == HAL_OK) &&
                      (sector_error == 0xFFFFFFFFUL));
}

static uint8_t App_ShuttleNvmSave(const uint32_t *duration_ms)
{
    AppShuttleNvmRecord latest;
    AppShuttleNvmRecord record;
    uint32_t address;
    uint8_t found;
    uint8_t index;
    HAL_StatusTypeDef status;

    if (App_ShuttleTimesAreValid(duration_ms) == 0U) {
        return 0U;
    }
    found = App_ShuttleNvmFindLatest(&latest, &address);
    if ((found != 0U) &&
        (memcmp(latest.duration_ms, duration_ms,
                sizeof(latest.duration_ms)) == 0)) {
        return 1U;
    }

    memset(&record, 0xFF, sizeof(record));
    record.magic = APP_SHUTTLE_NVM_MAGIC;
    record.version = APP_SHUTTLE_NVM_VERSION;
    record.sequence = (found != 0U) ? (latest.sequence + 1U) : 1U;
    for (index = 0U; index < BALL_SHUTTLE_PHASE_COUNT; index++) {
        record.duration_ms[index] = duration_ms[index];
    }
    record.crc32 = App_ShuttleNvmCrc32(&record);
    record.valid_marker = APP_SHUTTLE_NVM_VALID_MARKER;

    if (HAL_FLASH_Unlock() != HAL_OK) {
        return 0U;
    }
    if (address == APP_SHUTTLE_NVM_END_ADDRESS) {
        if (App_ShuttleNvmEraseSector() == 0U) {
            (void) HAL_FLASH_Lock();
            return 0U;
        }
        address = APP_SHUTTLE_NVM_ADDRESS;
    }
    status = HAL_FLASH_Program(FLASH_TYPEPROGRAM_FLASHWORD,
                               address,
                               (uint32_t) &record);
    (void) HAL_FLASH_Lock();
    if (status != HAL_OK) {
        return 0U;
    }

    __DSB();
    SCB_InvalidateDCache_by_Addr((uint32_t *) address,
                                APP_SHUTTLE_NVM_RECORD_SIZE);
    __DSB();
    __ISB();
    return (uint8_t) ((App_ShuttleNvmRecordIsValid(
                           (const AppShuttleNvmRecord *) address) != 0U) &&
                      (memcmp(((const AppShuttleNvmRecord *) address)->duration_ms,
                              duration_ms,
                              sizeof(record.duration_ms)) == 0));
}

static void App_ShuttleNvmLoad(void)
{
    AppShuttleNvmRecord latest;
    BallShuttleConfig config;

    if (App_ShuttleNvmFindLatest(&latest, 0) == 0U) {
        return;
    }
    BallShuttleControl_GetConfig(&s_ball_shuttle_control, &config);
    memcpy(config.duration_ms, latest.duration_ms,
           sizeof(config.duration_ms));
    (void) BallShuttleControl_SetConfig(&s_ball_shuttle_control, &config);
}

static const ChassisMotionConfig s_default_motion_config = {
    APP_BASE_SPEED_CM_S,       /* 第四至六项共用的平滑巡航目标速度。 */
    APP_LINE_MAX_ACCEL_CM_S2,  /* 起步阶段最大目标加速度。 */
    APP_LINE_MAX_DECEL_CM_S2,  /* 路况降速时最大目标减速度。 */
    APP_LINE_MAX_JERK_CM_S3,   /* 目标加速度变化率上限。 */
    0.0f                       /* 普通巡线不使用里程通过事件。 */
};

static const ChassisMotionConfig s_ab_motion_config = {
    APP_AB_SPEED_CM_S,
    APP_AB_MAX_ACCEL_CM_S2,
    APP_AB_MAX_DECEL_CM_S2,
    APP_AB_MAX_JERK_CM_S3,
    APP_AB_PASS_DISTANCE_CM
};

static const ChassisMotionConfig s_abcd_motion_config = {
    APP_ABCD_SPEED_CM_S,
    APP_ABCD_MAX_ACCEL_CM_S2,
    APP_ABCD_MAX_DECEL_CM_S2,
    APP_ABCD_MAX_JERK_CM_S3,
    0.0f
};

static const DynamicRunTaskConfig s_dynamic_run_config = {
    APP_DYNAMIC_PREPARE_TIMEOUT_MS,       /* 等待钢球任务目标稳定的最长时间。 */
    APP_DYNAMIC_READY_HOLD_MS,            /* 目标条件连续保持的起步确认时间。 */
    APP_DYNAMIC_READY_SAMPLE_GAP_MS,      /* 准备阶段视觉样本间隔上限。 */
    APP_AB_TIMEOUT_MS,                    /* 第四题从起步到通过 B 的限时。 */
    APP_ABCD_TIMEOUT_MS,                  /* 第五题从起步到通过 A 的限时。 */
    APP_DYNAMIC_READY_POSITION_MM,        /* 允许起步的位置误差绝对值。 */
    APP_DYNAMIC_READY_SPEED_MM_S,         /* 允许起步的钢球速度绝对值。 */
    APP_BALL_LIMIT_WARNING_MM,            /* 红圈和超差统计的误差门槛。 */
    APP_BALL_LIMIT_CLEAR_MM,              /* 红圈警告的回差清除门槛。 */
    APP_BALL_LIMIT_CLEAR_SAMPLES,         /* 清除红圈需要连续满足的样本数。 */
    APP_BALL_OVERRANGE_CONFIRM_SAMPLES,   /* 锁存超差结果需要连续越界的样本数。 */
    APP_DYNAMIC_IMU_FAULT_DEBOUNCE_MS,    /* IMU 故障或姿态无效持续时长才判失败。 */
    APP_DYNAMIC_VISION_FAULT_DEBOUNCE_MS  /* 视觉持续失联时长才判失败。 */
};

static float App_Abs(float value)
{
    return (value < 0.0f) ? -value : value;
}

static uint8_t App_IsPipeCalibrationBusy(void)
{
    return (uint8_t) ((s_pipe_zero_adjust_active != 0U) ||
                      (PipeAttitudeControl_IsCalibrationActive(
                           &s_pipe_attitude_control) != 0U));
}

static void App_CancelPipeZeroAdjustment(void)
{
    if (s_pipe_zero_adjust_active == 0U) {
        return;
    }

    /* 未确认的试调PWM不改变正式零位，退出时恢复已确认的零位。 */
    s_pipe_zero_adjust_active = 0U;
    s_pipe_zero_pwm_us = ServoDriver_GetCenterPulseUs();
    ServoDriver_SetPulseUs(s_pipe_zero_pwm_us);
    g_app_pipe_zero_adjust_active = 0U;
    g_app_pipe_zero_pwm_us = s_pipe_zero_pwm_us;
}

static void App_ApplyBallPidGains(void)
{
    float kp;
    float ki;
    float kd;
    float pipe_kp;
    float pipe_kd_s;

    /* 显式顺序读取volatile变量，便于IAR Live Watch在线调整两级控制增益。 */
    kp = g_app_ball_kp;
    ki = g_app_ball_ki;
    kd = g_app_ball_kd;
    pipe_kp = g_app_ball_pipe_kp;
    pipe_kd_s = g_app_ball_pipe_kd_s;
    BallBalanceControl_SetPidGains(&s_ball_balance_control, kp, ki, kd);
    BallBalanceControl_SetPipeGains(&s_ball_balance_control,
                                    pipe_kp,
                                    pipe_kd_s);
}

static void App_UpdateLapStopDebug(void)
{
    g_app_lap_distance_cm = s_lap_stop_detector.distance_cm;
    g_app_lap_left_start_line = s_lap_stop_detector.left_start_line;
    g_app_lap_stop_armed = s_lap_stop_detector.armed;
    g_app_lap_full_black_count = s_lap_stop_detector.full_black_count;
    g_app_lap_completed = s_lap_stop_detector.completed;
}

static void App_UpdateChassisMotionDebug(void)
{
    g_app_chassis_speed_filtered_cm_s =
        s_chassis_accel_output.filtered_speed_cm_s;
    g_app_chassis_accel_raw_cm_s2 = s_chassis_accel_output.raw_accel_cm_s2;
    g_app_chassis_accel_cm_s2 = s_chassis_accel_cm_s2;
    g_app_motion_profile_state = (uint8_t) s_motion_output.state;
    g_app_motion_distance_reached = s_motion_output.distance_reached;
    g_app_motion_target_speed_cm_s = s_motion_output.target_speed_cm_s;
    g_app_motion_command_speed_cm_s = s_motion_output.command_speed_cm_s;
    g_app_motion_command_accel_cm_s2 =
        s_motion_output.command_accel_cm_s2;
    g_app_motion_travelled_distance_cm =
        s_motion_output.travelled_distance_cm;
    g_app_dynamic_run_mode = (uint8_t) s_dynamic_run_task.mode;
    g_app_dynamic_run_state = (uint8_t) s_dynamic_run_task.state;
    g_app_dynamic_run_result = (uint8_t) s_dynamic_run_task.result;
    g_app_dynamic_run_failure = (uint8_t) s_dynamic_run_task.failure;
    g_app_dynamic_ready_elapsed_ms =
        s_dynamic_run_task.ready_stable_elapsed_ms;
    g_app_dynamic_elapsed_ms = s_dynamic_run_task.elapsed_ms;
    g_app_dynamic_sample_count = s_dynamic_run_task.accepted_sample_count;
    g_app_dynamic_overrange_latched =
        s_dynamic_run_task.overrange_latched;
    g_app_dynamic_overrange_duration_ms =
        s_dynamic_run_task.overrange_duration_ms;
    g_app_dynamic_distance_cm = s_dynamic_run_task.travelled_distance_cm;
    g_app_dynamic_max_abs_error_mm =
        s_dynamic_run_task.maximum_abs_error_mm;
    g_app_dynamic_rms_error_mm =
        DynamicRunTask_GetRmsErrorMm(&s_dynamic_run_task);
    g_app_ball_limit_warning = s_ball_limit_warning;
    g_app_ball_limit_warning_count =
        s_dynamic_run_task.overrange_event_count;
    g_app_lateral_valid = s_chassis_lateral_output.valid;
    g_app_turn_active = s_chassis_lateral_output.turn_active;
    g_app_lateral_bias_ready =
        s_chassis_lateral_output.accel_bias_ready;
    g_app_lateral_imu_sign =
        s_chassis_lateral_output.imu_lateral_sign;
    g_app_lateral_accel_x_bias_g =
        s_chassis_lateral_output.accel_x_bias_g;
    g_app_lateral_yaw_rate_dps =
        s_chassis_lateral_output.yaw_rate_dps;
    g_app_lateral_imu_accel_cm_s2 =
        s_chassis_lateral_output.imu_accel_cm_s2;
    g_app_lateral_kinematic_accel_cm_s2 =
        s_chassis_lateral_output.kinematic_accel_cm_s2;
    g_app_lateral_fused_accel_cm_s2 =
        s_chassis_lateral_output.lateral_accel_cm_s2;
    g_app_k230_tx_frame_count = K230Uart_GetTxFrameCount();
    g_app_k230_tx_drop_count = K230Uart_GetTxDropCount();
}

static void App_UpdateBallBalanceDebug(void)
{
    g_app_ball_balance_enabled = s_ball_balance_output.enabled;
    g_app_ball_enable_pending = s_ball_balance_output.enable_pending;
    g_app_ball_imu_fault = s_ball_balance_output.imu_fault;
    g_app_ball_mapping_fault = s_ball_balance_output.mapping_fault;
    g_app_ball_tracking_valid = s_ball_balance_output.tracking_valid;
    g_app_ball_timed_out = s_ball_balance_output.timed_out;
    g_app_ball_measurement_accepted =
        s_ball_balance_output.measurement_accepted;
    g_app_ball_target_mm = s_ball_balance_output.target_mm;
    g_app_ball_filtered_position_mm =
        s_ball_balance_output.filtered_position_mm;
    g_app_ball_predicted_position_mm =
        s_ball_balance_output.predicted_position_mm;
    g_app_ball_predicted_accel_mm_s2 =
        s_ball_balance_output.predicted_accel_mm_s2;
    g_app_ball_velocity_mm_s = s_ball_balance_output.velocity_mm_s;
    g_app_ball_error_mm = s_ball_balance_output.error_mm;
    g_app_ball_p_term_deg = s_ball_balance_output.p_term_deg;
    g_app_ball_i_term_deg = s_ball_balance_output.i_term_deg;
    g_app_ball_d_term_deg = s_ball_balance_output.d_term_deg;
    g_app_ball_brake_term_deg = s_ball_balance_output.brake_term_deg;
    g_app_ball_transport_mode_enabled =
        s_ball_balance_output.transport_mode_enabled;
    g_app_ball_transport_recovery_active =
        s_ball_balance_output.transport_recovery_active;
    g_app_ball_transport_command_accel_cm_s2 =
        s_ball_balance_output.transport_command_accel_cm_s2;
    g_app_ball_transport_position_deadband_mm =
        s_ball_balance_output.transport_position_deadband_mm;
    g_app_ball_transport_imu_disturbance_deg =
        s_ball_balance_output.transport_imu_disturbance_deg;
    g_app_ball_transport_feedforward_pitch_deg =
        s_ball_balance_output.transport_feedforward_pitch_deg;
    g_app_ball_transport_feedback_pitch_deg =
        s_ball_balance_output.transport_feedback_pitch_deg;
    g_app_ball_integral_mm_s = s_ball_balance_output.integral_mm_s;
    g_app_ball_integral_active = s_ball_balance_output.integral_active;
    g_app_ball_braking_active = s_ball_balance_output.braking_active;
    g_app_ball_zero_crossed = s_ball_balance_output.zero_crossed;
    g_app_ball_turnaround_freeze_active =
        s_ball_balance_output.turnaround_freeze_active;
    g_app_ball_pipe_settled = s_ball_balance_output.pipe_settled;
    g_app_ball_moving_sample_count =
        s_ball_balance_output.moving_sample_count;
    g_app_ball_moving_direction =
        s_ball_balance_output.moving_direction;
    g_app_ball_kick_attempt_count =
        s_ball_balance_output.kick_attempt_count;
    g_app_ball_trajectory_phase = s_ball_balance_output.trajectory_phase;
    g_app_ball_phase_pitch_deg = s_ball_balance_output.phase_pitch_deg;
    g_app_ball_stop_distance_mm = s_ball_balance_output.stop_distance_mm;
    g_app_ball_desired_pipe_pitch_deg =
        s_ball_balance_output.desired_pipe_pitch_deg;
    g_app_ball_command_pipe_pitch_deg =
        s_ball_balance_output.command_pipe_pitch_deg;
    g_app_ball_pipe_command_rate_dps =
        s_ball_balance_output.pipe_command_rate_dps;
    g_app_ball_measured_pipe_pitch_deg =
        s_ball_balance_output.measured_pipe_pitch_deg;
    g_app_ball_measured_pipe_rate_dps =
        s_ball_balance_output.measured_pipe_rate_dps;
    g_app_ball_pipe_error_deg = s_ball_balance_output.pipe_pitch_error_deg;
    g_app_ball_pipe_p_term_deg = s_ball_balance_output.pipe_p_term_deg;
    g_app_ball_pipe_d_term_deg = s_ball_balance_output.pipe_d_term_deg;
    g_app_ball_model_scale = s_ball_balance_output.model_scale;
    g_app_ball_model_offset_deg = s_ball_balance_output.model_offset_deg;
    g_app_ball_minimum_pipe_pitch_deg =
        s_ball_balance_output.minimum_pipe_pitch_deg;
    g_app_ball_maximum_pipe_pitch_deg =
        s_ball_balance_output.maximum_pipe_pitch_deg;
    g_app_ball_feedforward_model_pitch_deg =
        s_ball_balance_output.feedforward_model_pitch_deg;
    g_app_ball_desired_model_pitch_deg =
        s_ball_balance_output.desired_model_pitch_deg;
    g_app_ball_servo_feedforward_deg =
        s_ball_balance_output.servo_feedforward_deg;
    g_app_ball_desired_angle_deg = s_ball_balance_output.desired_angle_deg;
    g_app_ball_command_angle_deg = s_ball_balance_output.command_angle_deg;
    g_app_ball_feedforward_pwm_us = ServoDriver_PipePitchToPulseUs(
        s_ball_balance_output.servo_feedforward_deg);
    g_app_ball_desired_pwm_us = ServoDriver_PipePitchToPulseUs(
        s_ball_balance_output.desired_angle_deg);
    g_app_ball_command_pwm_us = ServoDriver_PipePitchToPulseUs(
        s_ball_balance_output.command_angle_deg);
    g_app_ball_actual_pwm_us = ServoDriver_GetPulseUs();
    g_app_ball_endpoint_hold_enabled =
        s_ball_balance_output.endpoint_hold_enabled;
}

static void App_UpdateBallShuttleDebug(void)
{
    uint8_t index;

    g_app_ball_shuttle_state = (uint8_t) s_ball_shuttle_control.state;
    g_app_ball_shuttle_elapsed_ms = s_ball_shuttle_control.elapsed_ms;
    g_app_ball_shuttle_target_pipe_pitch_deg =
        s_ball_shuttle_control.target_pipe_pitch_deg;
    for (index = 0U; index < BALL_SHUTTLE_PHASE_COUNT; index++) {
        g_app_ball_shuttle_angle_magnitude_deg[index] =
            s_ball_shuttle_control.config.angle_magnitude_deg[index];
        g_app_ball_shuttle_duration_ms[index] =
            s_ball_shuttle_control.config.duration_ms[index];
    }
}

static void App_UpdatePipeAttitudeDebug(void)
{
    uint8_t index;

    g_app_pipe_calibration_state =
        (uint8_t) s_pipe_attitude_output.calibration_state;
    g_app_pipe_calibration_active = App_IsPipeCalibrationBusy();
    g_app_pipe_calibration_valid =
        s_pipe_attitude_output.calibration_valid;
    g_app_pipe_zero_adjust_active = s_pipe_zero_adjust_active;
    g_app_pipe_zero_pwm_us = s_pipe_zero_pwm_us;
    g_app_pipe_attitude_valid = s_pipe_attitude_output.attitude_valid;
    g_app_pipe_accel_trusted = s_pipe_attitude_output.accel_trusted;
    g_app_pipe_imu_pitch_sign = s_pipe_attitude_output.imu_pitch_sign;
    g_app_pipe_calibration_point_index =
        s_pipe_attitude_output.calibration_point_index;
    g_app_pipe_calibration_sample_count =
        s_pipe_attitude_output.calibration_sample_count;
    g_app_pipe_gyro_x_bias_dps = s_pipe_attitude_output.gyro_x_bias_dps;
    g_app_pipe_accel_zero_deg =
        s_pipe_attitude_output.accel_pitch_zero_deg;
    g_app_pipe_raw_accel_pitch_deg =
        s_pipe_attitude_output.raw_accel_pitch_deg;
    g_app_pipe_raw_pitch_deg = s_pipe_attitude_output.raw_pitch_deg;
    g_app_pipe_raw_pitch_rate_dps =
        s_pipe_attitude_output.raw_pitch_rate_dps;
    g_app_pipe_accel_pitch_deg = s_pipe_attitude_output.accel_pitch_deg;
    g_app_pipe_pitch_deg = s_pipe_attitude_output.pitch_deg;
    g_app_pipe_pitch_rate_dps = s_pipe_attitude_output.pitch_rate_dps;
    g_app_pipe_negative_pitch_deg =
        s_pipe_attitude_output.negative_pitch_deg;
    g_app_pipe_center_pitch_deg = s_pipe_attitude_output.center_pitch_deg;
    g_app_pipe_positive_pitch_deg =
        s_pipe_attitude_output.positive_pitch_deg;
    g_app_pipe_return_pitch_deg = s_pipe_attitude_output.return_pitch_deg;
    g_app_pipe_model_scale = s_pipe_attitude_output.model_scale;
    g_app_pipe_model_offset_deg = s_pipe_attitude_output.model_offset_deg;
    for (index = 0U;
         index < LINKAGE_KINEMATICS_CALIBRATION_POINT_COUNT;
         index++) {
        g_app_pipe_mapping_pitch_deg[index] =
            s_pipe_attitude_output.mapping_pitch_deg[index];
    }
    g_app_pipe_calibration_target_pitch_deg =
        s_pipe_attitude_output.calibration_target_pitch_deg;
    g_app_pipe_servo_target_deg = s_pipe_attitude_output.servo_target_deg;
    g_app_pipe_servo_override_deg =
        s_pipe_attitude_output.servo_override_deg;
    g_app_chassis_accel_cm_s2 = s_chassis_accel_cm_s2;
}

static void App_BalanceDebugLogTask(uint32_t now_ms)
{
#if APP_ENABLE_BALANCE_LOG
    BalanceDebugLogInput input;

    input.now_ms = now_ms;
    input.imu_timestamp_ms = s_pipe_attitude_timestamp_ms;
    input.raw_position_d10_mm = g_app_ball_position_d10_mm;
    input.ball_sequence = g_app_ball_sequence;
    input.ball_quality = g_app_ball_quality;
    input.ball_sample_valid = g_app_ball_sample_valid;
    input.imu_accel_y_g = s_balance_log_imu_accel_y_g;
    input.imu_accel_z_g = s_balance_log_imu_accel_z_g;
    input.imu_gyro_x_dps = s_balance_log_imu_gyro_x_dps;
    input.actual_servo_pipe_pitch_deg =
        ServoDriver_GetPipePitchDeg();
    input.servo_pulse_us = ServoDriver_GetPulseUs();
    BalanceDebugLog_Update(&input,
                           &s_ball_balance_output,
                           &s_pipe_attitude_output);
#else
    (void) now_ms;
#endif
}

static void App_QueueTjcStatus(Tjc_StatusText status)
{
    s_pending_tjc_status = status;
    s_tjc_status_pending = 1U;
}

static void App_QueueTjcDynamicTarget(float target_mm)
{
    s_pending_tjc_dynamic_target_mm = target_mm;
    s_tjc_dynamic_target_pending = 1U;
}

static void App_QueueTjcShuttlePhase(uint8_t phase,
                                     float target_pipe_pitch_deg)
{
    s_pending_tjc_shuttle_phase = phase;
    s_pending_tjc_shuttle_target_deg = target_pipe_pitch_deg;
    s_tjc_shuttle_phase_pending = 1U;
}

static void App_SendPendingTjcStatus(void)
{
    if (s_tjc_status_pending != 0U) {
        s_tjc_status_pending = 0U;
        Protocol_Tjc_SendStatusText(s_pending_tjc_status);
    }
    if (s_tjc_dynamic_target_pending != 0U) {
        s_tjc_dynamic_target_pending = 0U;
        Protocol_Tjc_SendDynamicTarget(s_pending_tjc_dynamic_target_mm);
    }
    if (s_tjc_shuttle_phase_pending != 0U) {
        s_tjc_shuttle_phase_pending = 0U;
        Protocol_Tjc_SendShuttlePhase(
            s_pending_tjc_shuttle_phase,
            s_pending_tjc_shuttle_target_deg);
    }
}

static int16_t App_ValueToX100(float value)
{
    float scaled = value * 100.0f;

    if (scaled > 32767.0f) {
        return 32767;
    }
    if (scaled < -32767.0f) {
        return -32767;
    }
    return (int16_t) ((scaled >= 0.0f) ?
                      (scaled + 0.5f) : (scaled - 0.5f));
}

static void App_SendPipeCalibrationEvent(PipeCalibrationEvent event)
{
    switch (event) {
        case PIPE_CALIBRATION_EVENT_READY:
            Protocol_Tjc_SendStatusText(TJC_STATUS_CALIBRATION_READY);
            break;
        case PIPE_CALIBRATION_EVENT_LEVEL:
            Protocol_Tjc_SendStatusText(TJC_STATUS_CALIBRATION_LEVEL);
            break;
        case PIPE_CALIBRATION_EVENT_MAPPING_POINT:
            Protocol_Tjc_SendCalibrationPoint(
                (uint8_t) (s_pipe_attitude_output.calibration_point_index + 1U),
                LINKAGE_KINEMATICS_CALIBRATION_POINT_COUNT);
            break;
        case PIPE_CALIBRATION_EVENT_RETURN:
            Protocol_Tjc_SendStatusText(TJC_STATUS_CALIBRATION_RETURN);
            break;
        case PIPE_CALIBRATION_EVENT_SUCCESS:
            Protocol_Tjc_SendCalibrationSuccess(
                App_ValueToX100(s_pipe_attitude_output.model_scale),
                App_ValueToX100(s_pipe_attitude_output.model_offset_deg));
            break;
        case PIPE_CALIBRATION_EVENT_FAILED_IMU:
            Protocol_Tjc_SendStatusText(TJC_STATUS_CALIBRATION_FAILED_IMU);
            break;
        case PIPE_CALIBRATION_EVENT_FAILED_MOTION:
            Protocol_Tjc_SendStatusText(
                TJC_STATUS_CALIBRATION_FAILED_MOTION);
            break;
        case PIPE_CALIBRATION_EVENT_FAILED_RANGE:
            Protocol_Tjc_SendCalibrationRangeFailure(
                App_ValueToX100(s_pipe_attitude_output.model_scale),
                App_ValueToX100(s_pipe_attitude_output.model_offset_deg),
                App_ValueToX100(App_Abs(
                    s_pipe_attitude_output.return_pitch_deg -
                    s_pipe_attitude_output.center_pitch_deg)));
            break;
        case PIPE_CALIBRATION_EVENT_ABORTED:
            Protocol_Tjc_SendStatusText(TJC_STATUS_CALIBRATION_ABORTED);
            break;
        default:
            break;
    }
}

static void App_PipeAttitudeTask(uint32_t now_ms)
{
    const PipeAttitudeMeasurement *measurement_ptr = 0;
#if APP_USE_IMU
    ImuMotionSample imu_sample;
    PipeAttitudeMeasurement measurement;

    if (ImuHeading_TakeSample(&imu_sample) != 0U) {
        measurement.valid = imu_sample.valid;
        measurement.timestamp_ms = imu_sample.timestamp_ms;
        measurement.accel_x_g = imu_sample.accel_x_g;
        measurement.accel_y_g = imu_sample.accel_y_g;
        measurement.accel_z_g = imu_sample.accel_z_g;
        measurement.gyro_x_dps = imu_sample.gyro_x_dps;
        measurement.gyro_y_dps = imu_sample.gyro_y_dps;
        measurement.gyro_z_dps = imu_sample.gyro_z_dps;
        measurement_ptr = &measurement;
        s_pipe_attitude_timestamp_ms = imu_sample.timestamp_ms;
        s_latest_imu_accel_x_g = imu_sample.accel_x_g;
        ChassisLateralEstimator_Update(
            &s_chassis_lateral_estimator,
            imu_sample.timestamp_ms,
            s_chassis_accel_output.filtered_speed_cm_s,
            imu_sample.accel_x_g,
            imu_sample.gyro_z_dps,
            imu_sample.valid,
            &s_chassis_lateral_output);
#if APP_ENABLE_BALANCE_LOG
        s_balance_log_imu_accel_y_g = imu_sample.accel_y_g;
        s_balance_log_imu_accel_z_g = imu_sample.accel_z_g;
        s_balance_log_imu_gyro_x_dps = imu_sample.gyro_x_dps;
#endif
    }
#endif

    PipeAttitudeControl_Update(&s_pipe_attitude_control,
                               now_ms,
                               measurement_ptr,
                               s_chassis_accel_cm_s2,
                               &s_pipe_attitude_output);
    BallBalanceControl_SetPipeCalibration(
        &s_ball_balance_control,
        s_pipe_attitude_output.calibration_valid,
        s_pipe_attitude_output.model_scale,
        s_pipe_attitude_output.model_offset_deg,
        s_pipe_attitude_output.mapping_pitch_deg);
    if (s_pipe_attitude_output.status_changed != 0U) {
        App_SendPipeCalibrationEvent(
            s_pipe_attitude_output.calibration_event);
    }
    if ((s_pipe_attitude_output.servo_override_active != 0U) &&
        (App_GetHostCalibrationEnabled() == 0U)) {
        ServoDriver_SetPipePitchDeg(
            s_pipe_attitude_output.servo_override_deg);
    }
    App_UpdatePipeAttitudeDebug();
}

static void App_BallShuttleTask(uint32_t now_ms)
{
    BallShuttleEvent event;
    float target_pipe_pitch_deg;

    event = BallShuttleControl_Update(&s_ball_shuttle_control, now_ms);

    switch (event) {
        case BALL_SHUTTLE_EVENT_PHASE_2:
            target_pipe_pitch_deg =
                BallShuttleControl_GetTargetPipePitchDeg(
                    &s_ball_shuttle_control);
            BallBalanceControl_SetManualPipeMode(
                &s_ball_balance_control, 1U, target_pipe_pitch_deg);
            App_QueueTjcShuttlePhase(2U, target_pipe_pitch_deg);
            break;

        case BALL_SHUTTLE_EVENT_PHASE_3:
            target_pipe_pitch_deg =
                BallShuttleControl_GetTargetPipePitchDeg(
                    &s_ball_shuttle_control);
            BallBalanceControl_SetManualPipeMode(
                &s_ball_balance_control, 1U, target_pipe_pitch_deg);
            App_QueueTjcShuttlePhase(3U, target_pipe_pitch_deg);
            break;

        case BALL_SHUTTLE_EVENT_SUCCEEDED:
            BallBalanceControl_SetManualPipeMode(
                &s_ball_balance_control, 1U, 0.0f);
            App_QueueTjcStatus(TJC_STATUS_SHUTTLE_SUCCESS);
            break;

        default:
            break;
    }
    App_UpdateBallShuttleDebug();
}

static void App_BallBalanceTask(uint32_t now_ms)
{
    BallPositionSample sample;
    BallBalanceMeasurement measurement;
    BallBalanceAttitude attitude;
    const BallBalanceMeasurement *measurement_ptr = 0;
    const BallBalanceAttitude *attitude_ptr = 0;
    uint32_t overflow_count;
    uint32_t error_count;
    uint8_t byte;
    uint8_t wall_valid;
    float wall_accel_cm_s2;

    overflow_count = K230Uart_GetOverflowCount();
    error_count = K230Uart_GetErrorCount();
    if ((overflow_count != s_last_k230_overflow_count) ||
        (error_count != s_last_k230_error_count)) {
        BallPositionProtocol_ResetRx();
    }
    s_last_k230_overflow_count = overflow_count;
    s_last_k230_error_count = error_count;
    g_app_k230_overflow_count = overflow_count;
    g_app_k230_error_count = error_count;

    while (K230Uart_ReadByte(&byte) != 0U) {
        BallPositionProtocol_FeedByte(byte);
    }

    if (App_GetHostCalibrationEnabled() != 0U) {
        /* 上位机标定模式：跳过钢球平衡、往返和动态行驶任务，
         * 舵机由上机位独占，主循环只保留姿态估计与标定协议。 */
        return;
    }

    if (BallPositionProtocol_TakeSample(&sample) != 0U) {
        g_app_ball_sample_valid = sample.valid;
        g_app_ball_position_d10_mm = sample.position_d10_mm;
        g_app_ball_quality = sample.quality;
        g_app_ball_sequence = sample.sequence;
        s_last_ball_sample_ms = now_ms;
        measurement.valid = sample.valid;
        measurement.position_d10_mm = sample.position_d10_mm;
        measurement.quality = sample.quality;
        measurement.sequence = sample.sequence;
        measurement_ptr = &measurement;
    }

    if (s_pipe_attitude_output.attitude_valid != 0U) {
        attitude.valid = 1U;
        attitude.timestamp_ms = s_pipe_attitude_timestamp_ms;
        attitude.pitch_deg = s_pipe_attitude_output.pitch_deg;
        attitude.pitch_rate_dps = s_pipe_attitude_output.pitch_rate_dps;
        attitude_ptr = &attitude;
    }

    App_ApplyBallPidGains();
    BallBalanceControl_SetTransportInputs(
        &s_ball_balance_control,
        s_motion_output.command_accel_cm_s2,
        s_pipe_attitude_output.accel_pitch_deg -
        s_pipe_attitude_output.pitch_deg,
        s_pipe_attitude_output.attitude_valid);
    wall_valid = (uint8_t) (
        (App_DynamicRunIsLapMode() != 0U) &&
        (s_dynamic_run_task.state == DYNAMIC_RUN_STATE_RUNNING) &&
        (s_chassis_lateral_output.valid != 0U) &&
        (s_chassis_lateral_output.turn_active != 0U));
    wall_accel_cm_s2 = (wall_valid != 0U) ?
        (g_app_abcd_wall_coupling_gain *
         s_chassis_lateral_output.lateral_accel_cm_s2) : 0.0f;
    BallBalanceControl_SetTransportWallAcceleration(
        &s_ball_balance_control,
        wall_accel_cm_s2,
        wall_valid);
    BallBalanceControl_Update(&s_ball_balance_control,
                              now_ms,
                              measurement_ptr,
                              attitude_ptr,
                              &s_ball_balance_output);
    g_app_abcd_wall_accel_cm_s2 =
        s_ball_balance_output.transport_wall_accel_cm_s2;
    g_app_abcd_wall_pitch_deg =
        s_ball_balance_output.transport_wall_feedforward_pitch_deg;
    App_BallShuttleTask(now_ms);
    App_DynamicRunTask(now_ms, 0U, 1U);
    if ((s_pipe_zero_adjust_active == 0U) &&
        (s_pipe_attitude_output.servo_override_active == 0U) &&
        (App_GetHostCalibrationEnabled() == 0U)) {
        /* page2人工调平期间由n0.val独占舵机，禁止失能平衡输出每轮写回默认零位。 */
        ServoDriver_SetPipePitchDeg(
            s_ball_balance_output.command_angle_deg);
    }
    App_BalanceDebugLogTask(now_ms);

    if ((s_ball_balance_output.enabled != s_last_ball_active) &&
        (s_dynamic_run_task.state == DYNAMIC_RUN_STATE_IDLE)) {
        if (s_ball_balance_output.enabled != 0U) {
            Protocol_Tjc_SendStatusText(
                (s_last_ball_imu_fault != 0U) ?
                TJC_STATUS_BALANCE_RECOVERED :
                TJC_STATUS_BALANCE_ENABLED);
        } else if (s_ball_balance_output.imu_fault != 0U) {
            Protocol_Tjc_SendStatusText(TJC_STATUS_BALANCE_IMU_FAULT);
        }
    }
    s_last_ball_active = s_ball_balance_output.enabled;
    s_last_ball_imu_fault = s_ball_balance_output.imu_fault;
    App_UpdateBallBalanceDebug();
}

static void App_ShowTimerSeconds(uint32_t elapsed_s)
{
    uint8_t hours;
    uint8_t minutes;
    uint8_t seconds;

    elapsed_s %= 100UL * 60UL * 60UL;
    hours = (uint8_t) (elapsed_s / 3600UL);
    minutes = (uint8_t) ((elapsed_s / 60UL) % 60UL);
    seconds = (uint8_t) (elapsed_s % 60UL);
    OledDisplay_ShowTime(hours, minutes, seconds);
}

static void App_TimerResetAndStart(uint32_t now_ms)
{
    s_oled_timer_start_ms = now_ms;
    s_oled_timer_elapsed_s = 0U;
    s_oled_timer_last_display_s = 0U;
    s_oled_timer_running = 1U;
    App_ShowTimerSeconds(0U);
}

static void App_TimerStop(uint32_t now_ms)
{
    if (s_oled_timer_running != 0U) {
        s_oled_timer_elapsed_s = (uint32_t) ((now_ms - s_oled_timer_start_ms) / 1000U);
        s_oled_timer_running = 0U;
        App_ShowTimerSeconds(s_oled_timer_elapsed_s);
    }
}

static void App_TimerTask(uint32_t now_ms)
{
    uint32_t elapsed_s;

    if (s_oled_timer_running == 0U) {
        return;
    }

    elapsed_s = (uint32_t) ((now_ms - s_oled_timer_start_ms) / 1000U);
    if (elapsed_s != s_oled_timer_last_display_s) {
        s_oled_timer_elapsed_s = elapsed_s;
        s_oled_timer_last_display_s = elapsed_s;
        App_ShowTimerSeconds(elapsed_s);
    }
}

static uint8_t App_StartLineRunInternal(
    uint32_t now_ms,
    const ChassisMotionConfig *requested_config,
    uint8_t lap_stop_enabled,
    uint8_t app_mode,
    uint8_t smooth_response_enabled)
{
    ChassisMotionConfig motion_config;
    float initial_speed_cm_s = 0.0f;

    if (requested_config == 0) {
        return 0U;
    }
    motion_config = *requested_config;
    if (smooth_response_enabled == 0U) {
        /* 基础巡线按参考版本直接进入目标速度，不沿用第四、第五题速度轨迹。 */
        initial_speed_cm_s = motion_config.target_speed_cm_s;
    } else if ((s_line_enabled != 0U) &&
               (s_chassis_accel_output.valid != 0U)) {
        initial_speed_cm_s = s_chassis_accel_output.filtered_speed_cm_s;
        if (initial_speed_cm_s < 0.0f) {
            initial_speed_cm_s = 0.0f;
        }
    }

    InfraredLineControl_Init(&s_line_control);
    InfraredLineControl_SetSmoothResponse(&s_line_control,
                                          smooth_response_enabled);
    /* 路况降速后的恢复过程沿用本任务的加速度和jerk限制，避免二次突然加速。 */
    InfraredLineControl_SetMotionLimits(
        &s_line_control,
        motion_config.maximum_accel_cm_s2,
        motion_config.maximum_decel_cm_s2,
        motion_config.maximum_jerk_cm_s3);
    if (ChassisMotionProfile_Start(&s_motion_profile,
                                   now_ms,
                                   initial_speed_cm_s,
                                   &motion_config) == 0U) {
        return 0U;
    }
    ChassisMotionProfile_Update(&s_motion_profile, now_ms, 0.0f,
                                &s_motion_output);
    BallShuttleControl_Stop(&s_ball_shuttle_control);
    BallBalanceControl_SetManualPipeMode(
        &s_ball_balance_control, 0U, 0.0f);
    InfraredLineControl_SetBaseSpeed(
        &s_line_control, s_motion_output.command_speed_cm_s);
    LapStopDetector_Init(&s_lap_stop_detector);
    s_lap_stop_enabled = lap_stop_enabled;
    s_current_mode = app_mode;
    s_line_enabled = 1U;
    App_UpdateLapStopDebug();
    App_UpdateChassisMotionDebug();
    App_TimerResetAndStart(now_ms);
    return 1U;
}

static void App_StopClosedLoop(void)
{
    WheelSpeedPid_SetTarget(&s_left_speed_pid, 0.0f);
    WheelSpeedPid_SetTarget(&s_right_speed_pid, 0.0f);
    WheelSpeedPid_Reset(&s_left_speed_pid);
    WheelSpeedPid_Reset(&s_right_speed_pid);
    MotorDriver_Stop();
    g_app_line_command_valid = 0U;
    g_app_left_target_cm_s = 0.0f;
    g_app_right_target_cm_s = 0.0f;
    g_app_left_pwm_ratio = 0.0f;
    g_app_right_pwm_ratio = 0.0f;
}

static void App_EnterStopMode(uint32_t now_ms)
{
    s_current_mode = APP_MODE_STOP;
    s_line_enabled = 0U;
    s_lap_stop_enabled = 0U;
    ChassisMotionProfile_Stop(&s_motion_profile);
    ChassisMotionProfile_Update(&s_motion_profile, now_ms, 0.0f,
                                &s_motion_output);
    InfraredLineControl_SetBaseSpeed(&s_line_control, 0.0f);
    App_UpdateChassisMotionDebug();
    App_StopClosedLoop();
    App_TimerStop(now_ms);
}

static uint8_t App_DynamicRunIsActive(void)
{
    return DynamicRunTask_IsActive(&s_dynamic_run_task);
}

static uint8_t App_DynamicRunIsDriving(void)
{
    return DynamicRunTask_IsDriving(&s_dynamic_run_task);
}

static uint8_t App_DynamicRunIsLapMode(void)
{
    return (uint8_t) ((s_dynamic_run_task.mode == DYNAMIC_RUN_MODE_ABCD) ||
                      (s_dynamic_run_task.mode ==
                       DYNAMIC_RUN_MODE_ABCD_TARGET));
}

static uint8_t App_DynamicRunIsTargetMode(void)
{
    return (uint8_t) (s_dynamic_run_task.mode ==
                      DYNAMIC_RUN_MODE_ABCD_TARGET);
}

static void App_SetBallLimitWarning(uint8_t active)
{
    uint8_t normalized_active = (active != 0U) ? 1U : 0U;

    if (normalized_active == s_ball_limit_warning) {
        return;
    }
    s_ball_limit_warning = normalized_active;
    Protocol_Tjc_SetBallLimitWarning(normalized_active);
    App_UpdateChassisMotionDebug();
}

static Tjc_StatusText App_DynamicRunFailureStatus(
    DynamicRunFailure failure)
{
    switch (failure) {
        case DYNAMIC_RUN_FAILURE_IMU:
            return TJC_STATUS_RUN_FAILED_IMU;
        case DYNAMIC_RUN_FAILURE_MAPPING:
            return TJC_STATUS_RUN_FAILED_MAPPING;
        case DYNAMIC_RUN_FAILURE_PREPARE_TIMEOUT:
            return TJC_STATUS_RUN_FAILED_READY;
        case DYNAMIC_RUN_FAILURE_MOTION_START:
            return TJC_STATUS_RUN_FAILED_MOTION;
        case DYNAMIC_RUN_FAILURE_BALANCE_DISABLED:
            return TJC_STATUS_RUN_FAILED_BALANCE;
        case DYNAMIC_RUN_FAILURE_VISION:
            return TJC_STATUS_RUN_FAILED_VISION;
        case DYNAMIC_RUN_FAILURE_LINE_SENSOR:
            return TJC_STATUS_RUN_FAILED_SENSOR;
        case DYNAMIC_RUN_FAILURE_LINE_LOST:
            return TJC_STATUS_RUN_FAILED_LINE;
        case DYNAMIC_RUN_FAILURE_MOTOR:
            return TJC_STATUS_RUN_FAILED_MOTOR;
        default:
            if (s_dynamic_run_task.mode == DYNAMIC_RUN_MODE_AB) {
                return TJC_STATUS_AB_FAILED;
            }
            return (App_DynamicRunIsTargetMode() != 0U) ?
                   TJC_STATUS_ABCD_TARGET_FAILED :
                   TJC_STATUS_ABCD_FAILED;
    }
}

static void App_DynamicRunApplyHardFailure(uint32_t now_ms)
{
    s_dynamic_route_passed = 0U;
    App_SetBallLimitWarning(0U);
    App_EnterStopMode(now_ms);
    BallBalanceControl_SetTransportMode(&s_ball_balance_control, 0U);
    BallBalanceControl_SetEnabled(&s_ball_balance_control, 0U, now_ms);
    App_QueueTjcStatus(
        App_DynamicRunFailureStatus(s_dynamic_run_task.failure));
    App_UpdateChassisMotionDebug();
}

static void App_DynamicRunFail(uint32_t now_ms,
                               DynamicRunFailure failure)
{
    if (App_DynamicRunIsActive() == 0U) {
        return;
    }
    DynamicRunTask_Fail(&s_dynamic_run_task, failure, now_ms);
    App_DynamicRunApplyHardFailure(now_ms);
}

static const ChassisMotionConfig *App_DynamicRunMotionConfig(void)
{
    return (s_dynamic_run_task.mode == DYNAMIC_RUN_MODE_AB) ?
           &s_ab_motion_config : &s_abcd_motion_config;
}

static uint8_t App_DynamicRunAppMode(void)
{
    if (s_dynamic_run_task.mode == DYNAMIC_RUN_MODE_AB) {
        return APP_MODE_AB;
    }
    return (App_DynamicRunIsTargetMode() != 0U) ?
           APP_MODE_ABCD_TARGET : APP_MODE_ABCD;
}

static Tjc_StatusText App_DynamicRunRunningStatus(void)
{
    if (s_dynamic_run_task.mode == DYNAMIC_RUN_MODE_AB) {
        return TJC_STATUS_AB_RUNNING;
    }
    return (App_DynamicRunIsTargetMode() != 0U) ?
           TJC_STATUS_ABCD_TARGET_RUNNING : TJC_STATUS_ABCD_RUNNING;
}

static void App_DynamicRunTask(uint32_t now_ms,
                               uint8_t evaluate_completion,
                               uint8_t include_measurement)
{
    DynamicRunTaskInput input = {0};
    DynamicRunTaskActions actions;
    const ChassisMotionConfig *motion_config;
    uint8_t app_mode;
    uint8_t lap_stop_enabled;

    input.now_ms = now_ms;
    input.balance_enabled = s_ball_balance_output.enabled;
    input.tracking_valid = s_ball_balance_output.tracking_valid;
    input.timed_out = s_ball_balance_output.timed_out;
    input.attitude_valid = s_ball_balance_output.attitude_valid;
    input.imu_fault = s_ball_balance_output.imu_fault;
    input.mapping_fault = s_ball_balance_output.mapping_fault;
    input.measurement_accepted = (include_measurement != 0U) ?
        s_ball_balance_output.measurement_accepted : 0U;
    input.evaluate_completion = evaluate_completion;
    input.route_passed = (evaluate_completion != 0U) ?
        s_dynamic_route_passed : 0U;
    input.motion_stopped = s_motion_output.stopped;
    input.error_mm = s_ball_balance_output.error_mm;
    input.velocity_mm_s = s_ball_balance_output.velocity_mm_s;
    input.travelled_distance_cm = s_motion_output.travelled_distance_cm;
    DynamicRunTask_Update(&s_dynamic_run_task, &input, &actions);
    if (evaluate_completion != 0U) {
        s_dynamic_route_passed = 0U;
    }

    if (actions.warning_changed != 0U) {
        App_SetBallLimitWarning(s_dynamic_run_task.warning_active);
        if ((s_dynamic_run_task.warning_active == 0U) &&
            (actions.result_ready == 0U) &&
            (s_dynamic_run_task.state == DYNAMIC_RUN_STATE_RUNNING)) {
            App_QueueTjcStatus(App_DynamicRunRunningStatus());
        }
    }

    if (actions.hard_stop != 0U) {
        App_DynamicRunApplyHardFailure(now_ms);
        return;
    }

    if (actions.start_motion != 0U) {
        motion_config = App_DynamicRunMotionConfig();
        app_mode = App_DynamicRunAppMode();
        lap_stop_enabled = App_DynamicRunIsLapMode();
        BallBalanceControl_SetTransportMode(&s_ball_balance_control, 1U);
        if (App_StartLineRunInternal(now_ms,
                                     motion_config,
                                     lap_stop_enabled,
                                     app_mode,
                                     1U /* 第四、第五题启用平滑响应。 */) == 0U) {
            App_DynamicRunFail(now_ms,
                               DYNAMIC_RUN_FAILURE_MOTION_START);
            return;
        }
        DynamicRunTask_ConfirmMotionStarted(&s_dynamic_run_task, now_ms);
        App_QueueTjcStatus(App_DynamicRunRunningStatus());
    }

    if (actions.request_soft_stop != 0U) {
        ChassisMotionProfile_RequestStop(&s_motion_profile);
        if (s_dynamic_run_task.mode == DYNAMIC_RUN_MODE_ABCD) {
            /* 通过A即结束第五题；随后只让底盘在A前方柔停，
             * K230可以安全写SD卡，不再要求继续保持钢球。 */
            BallBalanceControl_SetTransportMode(&s_ball_balance_control, 0U);
            BallBalanceControl_SetEnabled(&s_ball_balance_control, 0U, now_ms);
        }
        /* 第六题保留运输模式至底盘停稳，继续使用柔停减速度前馈保持目标。 */
        App_TimerStop(now_ms);
        Protocol_Tjc_SendDynamicRunResult(
            (uint8_t) s_dynamic_run_task.mode,
            (uint8_t) s_dynamic_run_task.result,
            s_dynamic_run_task.elapsed_ms,
            s_dynamic_run_task.maximum_abs_error_mm);
    }

    if (actions.enter_stop_mode != 0U) {
        if (s_dynamic_run_task.mode == DYNAMIC_RUN_MODE_ABCD_TARGET) {
            BallBalanceControl_SetEnabled(&s_ball_balance_control,
                                          0U,
                                          now_ms);
        }
        App_EnterStopMode(now_ms);
    }
    App_UpdateChassisMotionDebug();
}

static void App_ControlStep(uint32_t now_ms)
{
    EncoderSample encoder;
    InfraredLineOutput line_output = {0};
    uint8_t sensor_mask = 0U;
    uint8_t sensor_read_ok;
    uint8_t sensor_failure_stop = 0U;
    uint8_t lap_stop_completed = 0U;
    uint8_t force_wide_slowdown = 0U;
    float left_command;
    float right_command;
    float distance_delta_cm;

    sensor_read_ok = InfraredLineSensor_ReadMask(&sensor_mask);
    g_app_line_sensor_raw = InfraredLineSensor_GetRawValue();
    g_app_line_sensor_i2c_error_count =
        InfraredLineSensor_GetErrorCount();

    if (sensor_read_ok != 0U) {
        s_last_line_sensor_mask = sensor_mask;
        s_has_line_sensor_sample = 1U;
        s_line_sensor_failure_count = 0U;
        g_app_line_sensor_stale_count = 0U;
        g_app_line_sensor_fault = 0U;
    } else {
        if (s_line_sensor_failure_count < 0xFFU) {
            s_line_sensor_failure_count++;
        }
        g_app_line_sensor_stale_count = s_line_sensor_failure_count;

        if ((s_has_line_sensor_sample == 0U) ||
            (s_line_sensor_failure_count >= APP_LINE_SENSOR_FAILURE_LIMIT)) {
            sensor_failure_stop = 1U;
            g_app_line_sensor_fault = 1U;
            InfraredLineControl_Init(&s_line_control);
        } else {
            /* Reuse the latest valid sensor mask during short I2C glitches. */
            sensor_mask = s_last_line_sensor_mask;
        }
    }

    EncoderDriver_Sample(&encoder);
    g_app_left_speed_cm_s = encoder.left_cm_s;
    g_app_right_speed_cm_s = encoder.right_cm_s;
    ChassisAccelEstimator_Update(&s_chassis_accel_estimator,
                                 now_ms,
                                 encoder.left_cm_s,
                                 encoder.right_cm_s,
                                 &s_chassis_accel_output);
    s_chassis_accel_cm_s2 = s_chassis_accel_output.accel_cm_s2;
    /* 将固定 20 ms 轮速样本还原为本周期左右轮平均行驶距离。 */
    distance_delta_cm = 0.5f *
                        (App_Abs(encoder.left_cm_s) +
                         App_Abs(encoder.right_cm_s)) *
                        ((float) APP_CONTROL_PERIOD_MS / 1000.0f);
    App_UpdateChassisMotionDebug();

    if (sensor_failure_stop != 0U) {
        g_app_line_sensor_mask = sensor_mask;
        g_app_line_active_count = 0U;
        g_app_line_lost = 1U;
        g_app_line_error = 0.0f;
        g_app_line_correction_cm_s = 0.0f;
        g_app_motor_fault = 0U;
        if (App_DynamicRunIsActive() != 0U) {
            App_DynamicRunFail(now_ms,
                               DYNAMIC_RUN_FAILURE_LINE_SENSOR);
        } else {
            App_StopClosedLoop();
        }
        return;
    }

    if (s_line_enabled != 0U) {
        ChassisMotionProfile_Update(&s_motion_profile,
                                    now_ms,
                                    distance_delta_cm,
                                    &s_motion_output);
        InfraredLineControl_SetBaseSpeed(
            &s_line_control, s_motion_output.command_speed_cm_s);
        if (s_lap_stop_enabled != 0U) {
            lap_stop_completed = LapStopDetector_Update(
                &s_lap_stop_detector,
                distance_delta_cm,
                sensor_read_ok,
                sensor_mask);
            /* 检测到返程 A 点横线候选后，暂时按宽线逻辑降低巡线速度。 */
            if (s_lap_stop_detector.full_black_count != 0U) {
                force_wide_slowdown = 1U;
            }
            App_UpdateLapStopDebug();
        }
        App_UpdateChassisMotionDebug();
    }

    if ((s_dynamic_run_task.state == DYNAMIC_RUN_STATE_RUNNING) &&
        (s_dynamic_run_task.mode == DYNAMIC_RUN_MODE_AB) &&
        (s_motion_output.distance_reached != 0U)) {
        s_dynamic_route_passed = 1U;
    }

    InfraredLineControl_Update(&s_line_control, now_ms, sensor_mask,
                               force_wide_slowdown, &line_output);
    g_app_line_sensor_mask = line_output.sensor_mask;
    g_app_line_active_count = line_output.active_count;
    g_app_line_lost = line_output.line_lost;
    g_app_line_error = line_output.filtered_error;
    g_app_line_correction_cm_s = line_output.correction_cm_s;

    if (lap_stop_completed != 0U) {
        if ((s_dynamic_run_task.state == DYNAMIC_RUN_STATE_RUNNING) &&
            (App_DynamicRunIsLapMode() != 0U)) {
            s_dynamic_route_passed = 1U;
        } else {
            App_EnterStopMode(now_ms);
            return;
        }
    }

    /* 上电默认停车，收到 TJC 基础巡线命令后才允许闭环输出。 */
    if ((s_line_enabled == 0U) || (line_output.command_valid == 0U)) {
        if ((s_line_enabled != 0U) &&
            (App_DynamicRunIsDriving() != 0U)) {
            App_DynamicRunFail(now_ms,
                               DYNAMIC_RUN_FAILURE_LINE_LOST);
        } else {
            App_StopClosedLoop();
        }
        return;
    }

    WheelSpeedPid_SetTarget(&s_left_speed_pid,
                            line_output.left_target_cm_s);
    WheelSpeedPid_SetTarget(&s_right_speed_pid,
                            line_output.right_target_cm_s);
    left_command = WheelSpeedPid_Step(&s_left_speed_pid,
                                      encoder.left_cm_s);
    right_command = WheelSpeedPid_Step(&s_right_speed_pid,
                                       encoder.right_cm_s);

    if ((WheelSpeedPid_HasFault(&s_left_speed_pid) != 0U) ||
        (WheelSpeedPid_HasFault(&s_right_speed_pid) != 0U)) {
        g_app_motor_fault = 1U;
        if (App_DynamicRunIsDriving() != 0U) {
            App_DynamicRunFail(now_ms, DYNAMIC_RUN_FAILURE_MOTOR);
        } else {
            App_StopClosedLoop();
        }
        return;
    }

    g_app_motor_fault = 0U;
    MotorDriver_Set(left_command, right_command);
    g_app_line_command_valid = 1U;
    g_app_left_target_cm_s = line_output.left_target_cm_s;
    g_app_right_target_cm_s = line_output.right_target_cm_s;
    g_app_left_pwm_ratio = left_command;
    g_app_right_pwm_ratio = right_command;
}

#if APP_ENABLE_K230_TELEMETRY
static void App_FillK230Telemetry(uint32_t now_ms,
                                  K230TelemetryInput *telemetry)
{
    uint16_t flags = 0U;

    memset(telemetry, 0, sizeof(*telemetry));
    if (s_chassis_lateral_output.valid != 0U) {
        flags |= APP_K230_TELEMETRY_FLAG_LATERAL_VALID;
    }
    if (s_chassis_lateral_output.turn_active != 0U) {
        flags |= APP_K230_TELEMETRY_FLAG_TURN_ACTIVE;
    }
    if (s_chassis_lateral_output.accel_bias_ready != 0U) {
        flags |= APP_K230_TELEMETRY_FLAG_BIAS_READY;
    }
    if (s_chassis_lateral_output.imu_sign_ready != 0U) {
        flags |= APP_K230_TELEMETRY_FLAG_IMU_SIGN_READY;
    }
    if (s_ball_balance_output.tracking_valid != 0U) {
        flags |= APP_K230_TELEMETRY_FLAG_BALL_VALID;
    }
    if (s_dynamic_run_task.overrange_latched != 0U) {
        flags |= APP_K230_TELEMETRY_FLAG_OVERRANGE;
    }
    if (s_pipe_attitude_output.accel_trusted != 0U) {
        flags |= APP_K230_TELEMETRY_FLAG_ACCEL_TRUSTED;
    }
    if ((App_DynamicRunIsLapMode() != 0U) &&
        (s_dynamic_run_task.state == DYNAMIC_RUN_STATE_RUNNING) &&
        (s_chassis_lateral_output.turn_active != 0U)) {
        flags |= APP_K230_TELEMETRY_FLAG_WALL_ACTIVE;
    }

    telemetry->timestamp_ms = now_ms;
    telemetry->run_mode = (uint8_t) s_dynamic_run_task.mode;
    telemetry->run_state = (uint8_t) s_dynamic_run_task.state;
    telemetry->flags = flags;
    telemetry->speed_cm_s = s_chassis_accel_output.filtered_speed_cm_s;
    telemetry->yaw_rate_dps = s_chassis_lateral_output.yaw_rate_dps;
    telemetry->accel_x_g = s_latest_imu_accel_x_g;
    telemetry->lateral_kinematic_cm_s2 =
        s_chassis_lateral_output.kinematic_accel_cm_s2;
    telemetry->lateral_imu_cm_s2 =
        s_chassis_lateral_output.imu_accel_cm_s2;
    telemetry->lateral_fused_cm_s2 =
        s_chassis_lateral_output.lateral_accel_cm_s2;
    telemetry->ball_error_mm = s_ball_balance_output.error_mm;
    telemetry->ball_velocity_mm_s = s_ball_balance_output.velocity_mm_s;
    telemetry->pipe_command_deg =
        s_ball_balance_output.command_pipe_pitch_deg;
    telemetry->wall_accel_cm_s2 =
        s_ball_balance_output.transport_wall_accel_cm_s2;
    telemetry->wall_pitch_deg =
        s_ball_balance_output.transport_wall_feedforward_pitch_deg;
    telemetry->travelled_distance_cm =
        s_motion_output.travelled_distance_cm;
    telemetry->transport_feedback_pitch_deg =
        s_ball_balance_output.transport_feedback_pitch_deg;
}

static void App_K230TelemetryTask(uint32_t now_ms)
{
    K230TelemetryInput telemetry;

    App_FillK230Telemetry(now_ms, &telemetry);
    if (s_k230_telemetry_session_active == 0U) {
        if (((s_dynamic_run_task.mode == DYNAMIC_RUN_MODE_AB) ||
             (App_DynamicRunIsLapMode() != 0U)) &&
            (s_dynamic_run_task.state == DYNAMIC_RUN_STATE_RUNNING) &&
            (K230Uart_SendTelemetry(K230_TELEMETRY_RUN_START,
                                    &telemetry) != 0U)) {
            s_k230_telemetry_session_active = 1U;
            s_k230_telemetry_last_sample_ms = now_ms;
        }
        return;
    }

    if (((s_dynamic_run_task.mode != DYNAMIC_RUN_MODE_AB) &&
         (App_DynamicRunIsLapMode() == 0U)) ||
        (s_dynamic_run_task.state != DYNAMIC_RUN_STATE_RUNNING)) {
        if (K230Uart_SendTelemetry(K230_TELEMETRY_RUN_END,
                                   &telemetry) != 0U) {
            s_k230_telemetry_session_active = 0U;
        }
        return;
    }

    if ((uint32_t) (now_ms - s_k230_telemetry_last_sample_ms) >=
        APP_K230_TELEMETRY_PERIOD_MS) {
        if (K230Uart_SendTelemetry(K230_TELEMETRY_SAMPLE,
                                   &telemetry) != 0U) {
            s_k230_telemetry_last_sample_ms = now_ms;
        }
    }
}
#else
static void App_K230TelemetryTask(uint32_t now_ms)
{
    /* 首轮 ABCD 实测仅肉眼观察；保留 K230 RX，不占用 UART4 TX。 */
    (void) now_ms;
}
#endif

void App_Init(void)
{
    InfraredLineControl_Init(&s_line_control);
    LapStopDetector_Init(&s_lap_stop_detector);
    ChassisMotionProfile_Init(&s_motion_profile);
    /* 配置来自编译期常量，参数合法性由主机单元测试覆盖。 */
    (void) DynamicRunTask_Init(&s_dynamic_run_task,
                               &s_dynamic_run_config);
    ChassisAccelEstimator_Init(&s_chassis_accel_estimator);
    ChassisLateralEstimator_Init(&s_chassis_lateral_estimator);
    PipeAttitudeControl_Init(&s_pipe_attitude_control);
    BallBalanceControl_Init(&s_ball_balance_control);
    BalanceDebugLog_Init();
    App_ApplyBallPidGains();
    BallShuttleControl_Init(&s_ball_shuttle_control);
    App_ShuttleNvmLoad();
    BallPositionProtocol_Init();
    WheelSpeedPid_Init(&s_left_speed_pid);
    WheelSpeedPid_Init(&s_right_speed_pid);
    InfraredLineSensor_Init();
    MotorDriver_Init();
    EncoderDriver_Init();
#if APP_USE_IMU
    ImuHeading_Init();
#endif
    K230Uart_Init();
#if APP_USE_HOST_CALIBRATION
    HostLinkUart_Init();
    HostCalibrationProtocol_Init();
#endif
    (void) ServoDriver_SetCenterPulseUs(SERVO_DRIVER_CENTER_PULSE_US);
    ServoDriver_InitPulseUs(SERVO_DRIVER_CENTER_PULSE_US);
    s_pipe_zero_adjust_active = 0U;
    s_pipe_zero_pwm_us = ServoDriver_GetCenterPulseUs();
    g_app_pipe_zero_adjust_active = 0U;
    g_app_pipe_zero_pwm_us = s_pipe_zero_pwm_us;
    TjcUart_Init();
    Protocol_Tjc_Init();
    OledDisplay_Init();
    App_ShowTimerSeconds(0U);
    s_last_control_ms = Board_TimeMs();
    s_oled_timer_start_ms = s_last_control_ms;
    s_oled_timer_elapsed_s = 0U;
    s_oled_timer_last_display_s = 0U;
    s_oled_timer_running = 0U;
    s_current_mode = APP_MODE_STOP;
    s_line_enabled = 0U;
    s_last_line_sensor_mask = 0U;
    s_has_line_sensor_sample = 0U;
    s_line_sensor_failure_count = 0U;
    s_last_k230_overflow_count = K230Uart_GetOverflowCount();
    s_last_k230_error_count = K230Uart_GetErrorCount();
    g_app_line_sensor_fault = 0U;
    g_app_ball_sample_valid = 0U;
    g_app_ball_position_d10_mm = 0;
    g_app_ball_quality = 0U;
    g_app_ball_sequence = 0U;
    g_app_k230_overflow_count = s_last_k230_overflow_count;
    g_app_k230_error_count = s_last_k230_error_count;
    s_pipe_attitude_timestamp_ms = 0U;
    s_chassis_accel_cm_s2 = 0.0f;
    s_latest_imu_accel_x_g = 0.0f;
    s_lap_stop_enabled = 0U;
    s_last_ball_active = 0U;
    s_last_ball_imu_fault = 0U;
    s_last_ball_sample_ms = 0U;
    s_dynamic_route_passed = 0U;
    s_ball_limit_warning = 0U;
    s_k230_telemetry_session_active = 0U;
    s_k230_telemetry_last_sample_ms = 0U;
    g_app_dynamic_run_mode = (uint8_t) DYNAMIC_RUN_MODE_NONE;
    g_app_dynamic_run_state = (uint8_t) DYNAMIC_RUN_STATE_IDLE;
    g_app_dynamic_run_result = (uint8_t) DYNAMIC_RUN_RESULT_NONE;
    g_app_dynamic_run_failure = (uint8_t) DYNAMIC_RUN_FAILURE_NONE;
    g_app_dynamic_start_reject =
        (uint8_t) APP_DYNAMIC_START_REJECT_NONE;
    g_app_ball_shuttle_start_reject =
        (uint8_t) APP_SHUTTLE_START_REJECT_NONE;
    g_app_dynamic_overrange_latched = 0U;
    g_app_dynamic_ready_elapsed_ms = 0U;
    g_app_dynamic_elapsed_ms = 0U;
    g_app_dynamic_sample_count = 0U;
    g_app_dynamic_overrange_duration_ms = 0U;
    g_app_dynamic_distance_cm = 0.0f;
    g_app_dynamic_max_abs_error_mm = 0.0f;
    g_app_dynamic_rms_error_mm = 0.0f;
    g_app_host_calibration_enabled = s_host_calibration_enabled;
    g_app_ball_limit_warning = 0U;
    g_app_ball_limit_warning_count = 0U;
    s_tjc_status_pending = 0U;
    s_pending_tjc_status = TJC_STATUS_BALANCE_DISABLED;
    s_tjc_dynamic_target_pending = 0U;
    s_pending_tjc_dynamic_target_mm = 0.0f;
    s_tjc_shuttle_phase_pending = 0U;
    s_pending_tjc_shuttle_phase = 0U;
    s_pending_tjc_shuttle_target_deg = 0.0f;
    PipeAttitudeControl_Update(&s_pipe_attitude_control,
                               s_last_control_ms,
                               0,
                               0.0f,
                               &s_pipe_attitude_output);
    BallBalanceControl_Update(&s_ball_balance_control,
                              s_last_control_ms,
                              0,
                              0,
                              &s_ball_balance_output);
    ChassisMotionProfile_Update(&s_motion_profile,
                                s_last_control_ms,
                                0.0f,
                                &s_motion_output);
    App_UpdatePipeAttitudeDebug();
    App_UpdateBallBalanceDebug();
    App_UpdateBallShuttleDebug();
    App_UpdateLapStopDebug();
    App_UpdateChassisMotionDebug();
    App_StopClosedLoop();
    g_app_init_step = 100U;
}

void App_Loop(void)
{
    uint32_t now_ms;

    Protocol_Tjc_Task();
#if APP_USE_HOST_CALIBRATION
    HostCalibrationProtocol_Task();
#endif
    App_SendPendingTjcStatus();
#if APP_USE_IMU
    ImuHeading_Poll();
#endif
    now_ms = Board_TimeMs();
    App_PipeAttitudeTask(now_ms);
    App_BallBalanceTask(now_ms);
    if ((uint32_t) (now_ms - s_last_control_ms) >= APP_CONTROL_PERIOD_MS) {
        /* Do not backfill missed periods with the same encoder sample. */
        s_last_control_ms = now_ms;
        App_ControlStep(now_ms);
        now_ms = Board_TimeMs();
        App_DynamicRunTask(now_ms, 1U, 0U);
        now_ms = Board_TimeMs();
    }
    App_K230TelemetryTask(now_ms);
    App_TimerTask(now_ms);
}

uint8_t App_RequestMode(uint8_t mode)
{
    if (mode == APP_MODE_STOP) {
        DynamicRunTask_Cancel(&s_dynamic_run_task);
        s_dynamic_route_passed = 0U;
        s_tjc_dynamic_target_pending = 0U;
        App_SetBallLimitWarning(0U);
        App_SetBallBalanceEnabled(0U);
        App_CancelPipeZeroAdjustment();
        App_EnterStopMode(Board_TimeMs());
        App_UpdateChassisMotionDebug();
        return 1U;
    }

    if (mode == APP_MODE_BASIC_LINE) {
        if ((DynamicRunTask_IsActive(&s_dynamic_run_task) != 0U) ||
            (App_IsPipeCalibrationBusy() != 0U) ||
            (BallShuttleControl_IsRunning(&s_ball_shuttle_control) != 0U) ||
            (BallShuttleControl_IsHolding(&s_ball_shuttle_control) != 0U) ||
            ((BallBalanceControl_IsEnabled(&s_ball_balance_control) != 0U) &&
             (BallBalanceControl_IsActive(&s_ball_balance_control) == 0U))) {
            return 0U;
        }
        return App_StartLineRunInternal(Board_TimeMs(),
                                        &s_default_motion_config,
                                        1U,
                                        APP_MODE_BASIC_LINE,
                                        0U /* 基础巡线关闭平滑响应。 */);
    }

    return 0U;
}

uint8_t App_GetCurrentMode(void)
{
    return s_current_mode;
}

uint8_t App_StartSmoothLineRun(float completion_distance_cm)
{
    uint32_t now_ms = Board_TimeMs();
    ChassisMotionConfig motion_config = s_default_motion_config;

    if ((completion_distance_cm <= 0.0f) ||
        (s_current_mode != APP_MODE_STOP) ||
        (DynamicRunTask_IsActive(&s_dynamic_run_task) != 0U) ||
        (App_IsPipeCalibrationBusy() != 0U) ||
        (BallShuttleControl_IsRunning(&s_ball_shuttle_control) != 0U) ||
        (BallShuttleControl_IsHolding(&s_ball_shuttle_control) != 0U) ||
        (BallBalanceControl_IsActive(&s_ball_balance_control) == 0U) ||
        (s_ball_balance_output.tracking_valid == 0U) ||
        (s_ball_balance_output.timed_out != 0U) ||
        (s_ball_balance_output.imu_fault != 0U) ||
        (s_ball_balance_output.mapping_fault != 0U)) {
        return 0U;
    }

    motion_config.completion_distance_cm = completion_distance_cm;
    return App_StartLineRunInternal(now_ms,
                                    &motion_config,
                                    0U,
                                    APP_MODE_BASIC_LINE,
                                    1U /* 显式请求平滑巡线响应。 */);
}

uint8_t App_StartSmoothAbRun(void)
{
    return App_StartSmoothLineRun(APP_AB_PASS_DISTANCE_CM);
}

uint8_t App_StartSmoothLapRun(void)
{
    return App_StartSmoothLineRun(APP_LAP_PASS_DISTANCE_CM);
}

static uint8_t App_RejectDynamicStart(
    AppDynamicStartRejectReason reason,
    Tjc_StatusText status)
{
    g_app_dynamic_start_reject = (uint8_t) reason;
    App_QueueTjcStatus(status);
    return 0U;
}

static uint8_t App_StartDynamicRun(DynamicRunMode mode,
                                   uint8_t app_mode,
                                   Tjc_StatusText preparing_status,
                                   uint8_t capture_current_target)
{
    uint32_t now_ms = Board_TimeMs();
    float target_mm = 0.0f;

    g_app_dynamic_start_reject =
        (uint8_t) APP_DYNAMIC_START_REJECT_NONE;
    s_tjc_dynamic_target_pending = 0U;
    if (s_current_mode != APP_MODE_STOP) {
        return App_RejectDynamicStart(
            APP_DYNAMIC_START_REJECT_NOT_STOPPED,
            TJC_STATUS_RUN_REJECT_STOP);
    }
    if (App_IsPipeCalibrationBusy() != 0U) {
        return App_RejectDynamicStart(
            APP_DYNAMIC_START_REJECT_CALIBRATING,
            TJC_STATUS_RUN_REJECT_CALIBRATING);
    }
    if (PipeAttitudeControl_IsCalibrationValid(
            &s_pipe_attitude_control) == 0U) {
        return App_RejectDynamicStart(
            APP_DYNAMIC_START_REJECT_NOT_CALIBRATED,
            TJC_STATUS_RUN_REJECT_CALIBRATION);
    }
    if ((BallShuttleControl_IsRunning(&s_ball_shuttle_control) != 0U) ||
        (BallShuttleControl_IsHolding(&s_ball_shuttle_control) != 0U) ||
        (DynamicRunTask_IsActive(&s_dynamic_run_task) != 0U)) {
        return App_RejectDynamicStart(
            APP_DYNAMIC_START_REJECT_OTHER_TASK,
            TJC_STATUS_RUN_REJECT_BUSY);
    }
    if (capture_current_target != 0U) {
        if (BallBalanceControl_GetFreshObservedPosition(
                &s_ball_balance_control,
                now_ms,
                APP_DYNAMIC_TARGET_SAMPLE_MAX_AGE_MS,
                &target_mm) == 0U) {
            return App_RejectDynamicStart(
                APP_DYNAMIC_START_REJECT_VISION,
                TJC_STATUS_RUN_REJECT_VISION);
        }
        if (App_Abs(target_mm) > BALL_BALANCE_TARGET_LIMIT_MM) {
            return App_RejectDynamicStart(
                APP_DYNAMIC_START_REJECT_TARGET_RANGE,
                TJC_STATUS_RUN_REJECT_TARGET);
        }
    }

    BallShuttleControl_Stop(&s_ball_shuttle_control);
    BallBalanceControl_SetManualPipeMode(
        &s_ball_balance_control, 0U, 0.0f);
    BallBalanceControl_SetEndpointHoldEnabled(&s_ball_balance_control, 0U);
    BallBalanceControl_SetTargetMm(&s_ball_balance_control, target_mm);
    /* 准备阶段先使用完整位置控制稳定目标，真正起步前再切换混合前馈。 */
    BallBalanceControl_SetTransportMode(&s_ball_balance_control, 0U);
    if (capture_current_target != 0U) {
        BallBalanceControl_SetEnabledPreserveVision(
            &s_ball_balance_control, 1U, now_ms);
    } else {
        BallBalanceControl_SetEnabled(&s_ball_balance_control, 1U, now_ms);
    }
    if (BallBalanceControl_IsEnabled(&s_ball_balance_control) == 0U) {
        BallBalanceControl_SetTransportMode(&s_ball_balance_control, 0U);
        return App_RejectDynamicStart(
            APP_DYNAMIC_START_REJECT_BALANCE_ENABLE,
            TJC_STATUS_RUN_REJECT_BALANCE);
    }
    if (DynamicRunTask_Start(&s_dynamic_run_task, mode, now_ms) == 0U) {
        BallBalanceControl_SetTransportMode(&s_ball_balance_control, 0U);
        return App_RejectDynamicStart(
            APP_DYNAMIC_START_REJECT_OTHER_TASK,
            TJC_STATUS_RUN_REJECT_BUSY);
    }

    App_SetBallLimitWarning(0U);
    s_dynamic_route_passed = 0U;
    s_current_mode = app_mode;
    s_line_enabled = 0U;
    s_lap_stop_enabled = 0U;
    App_StopClosedLoop();
    App_QueueTjcStatus(preparing_status);
    if (capture_current_target != 0U) {
        App_QueueTjcDynamicTarget(target_mm);
    }
    App_UpdateChassisMotionDebug();
    return 1U;
}

uint8_t App_StartAbRun(void)
{
    return App_StartDynamicRun(DYNAMIC_RUN_MODE_AB,
                               APP_MODE_AB,
                               TJC_STATUS_AB_PREPARING,
                               0U);
}

uint8_t App_StartAbcdRun(void)
{
    return App_StartDynamicRun(DYNAMIC_RUN_MODE_ABCD,
                               APP_MODE_ABCD,
                               TJC_STATUS_ABCD_PREPARING,
                               0U);
}

uint8_t App_StartAbcdTargetRun(void)
{
    return App_StartDynamicRun(DYNAMIC_RUN_MODE_ABCD_TARGET,
                               APP_MODE_ABCD_TARGET,
                               TJC_STATUS_ABCD_TARGET_PREPARING,
                               1U);
}

uint8_t App_IsDynamicRunCommandLocked(void)
{
    return DynamicRunTask_IsActive(&s_dynamic_run_task);
}

uint8_t App_SetBallBalanceEnabled(uint8_t enabled)
{
    uint32_t now_ms = Board_TimeMs();

    if ((enabled == 0U) &&
        (DynamicRunTask_IsActive(&s_dynamic_run_task) != 0U)) {
        App_DynamicRunFail(now_ms,
                           DYNAMIC_RUN_FAILURE_BALANCE_DISABLED);
        return 1U;
    }

    if (enabled != 0U) {
        if ((s_current_mode != APP_MODE_STOP) ||
            (App_IsPipeCalibrationBusy() != 0U) ||
            (PipeAttitudeControl_IsCalibrationValid(
                 &s_pipe_attitude_control) == 0U)) {
            return 0U;
        }
        BallShuttleControl_Stop(&s_ball_shuttle_control);
        BallBalanceControl_SetManualPipeMode(
            &s_ball_balance_control, 0U, 0.0f);
        BallBalanceControl_SetTransportMode(&s_ball_balance_control, 0U);
        BallBalanceControl_SetEndpointHoldEnabled(
            &s_ball_balance_control, 0U);
        BallBalanceControl_SetTargetMm(&s_ball_balance_control, 0.0f);
        BallBalanceControl_SetEnabled(&s_ball_balance_control, 1U, now_ms);
        App_UpdateBallShuttleDebug();
        App_QueueTjcStatus(TJC_STATUS_BALANCE_CHECKING);
    } else {
        if (PipeAttitudeControl_IsCalibrationActive(
                &s_pipe_attitude_control) != 0U) {
            PipeAttitudeControl_AbortCalibration(&s_pipe_attitude_control,
                                                  now_ms);
        }
        App_CancelPipeZeroAdjustment();
        BallShuttleControl_Stop(&s_ball_shuttle_control);
        BallBalanceControl_SetManualPipeMode(
            &s_ball_balance_control, 0U, 0.0f);
        App_SetBallLimitWarning(0U);
        BallBalanceControl_SetTransportMode(&s_ball_balance_control, 0U);
        BallBalanceControl_SetEndpointHoldEnabled(
            &s_ball_balance_control, 0U);
        BallBalanceControl_SetEnabled(&s_ball_balance_control, 0U, now_ms);
        App_UpdateBallShuttleDebug();
        App_QueueTjcStatus(TJC_STATUS_BALANCE_DISABLED);
    }
    return 1U;
}

uint8_t App_GetBallBalanceEnabled(void)
{
    return BallBalanceControl_IsEnabled(&s_ball_balance_control);
}

static uint8_t App_BallShuttleCenterFrameReady(uint32_t now_ms)
{
    int32_t position_d10_mm = g_app_ball_position_d10_mm;

    if (position_d10_mm < 0) {
        position_d10_mm = -position_d10_mm;
    }
    return (uint8_t) (
        (g_app_ball_sample_valid != 0U) &&
        (g_app_ball_quality >= APP_BALL_SHUTTLE_MIN_QUALITY) &&
        ((uint32_t) (now_ms - s_last_ball_sample_ms) <=
         APP_BALL_SHUTTLE_SAMPLE_MAX_AGE_MS) &&
        (position_d10_mm <= APP_BALL_SHUTTLE_CENTER_LIMIT_D10_MM));
}

uint8_t App_StartBallShuttle(void)
{
    uint32_t now_ms = Board_TimeMs();
    float target_pipe_pitch_deg;

    g_app_ball_shuttle_start_reject =
        (uint8_t) APP_SHUTTLE_START_REJECT_NONE;
    if (s_current_mode != APP_MODE_STOP) {
        g_app_ball_shuttle_start_reject =
            (uint8_t) APP_SHUTTLE_START_REJECT_NOT_STOPPED;
        return 0U;
    }
    if ((DynamicRunTask_IsActive(&s_dynamic_run_task) != 0U) ||
        (App_IsPipeCalibrationBusy() != 0U) ||
        (BallShuttleControl_IsRunning(&s_ball_shuttle_control) != 0U) ||
        (App_GetHostCalibrationEnabled() != 0U)) {
        g_app_ball_shuttle_start_reject =
            (uint8_t) APP_SHUTTLE_START_REJECT_BUSY;
        return 0U;
    }
    if (App_BallShuttleCenterFrameReady(now_ms) == 0U) {
        g_app_ball_shuttle_start_reject =
            (uint8_t) APP_SHUTTLE_START_REJECT_CENTER;
        App_QueueTjcStatus(TJC_STATUS_SHUTTLE_NOT_READY);
        return 0U;
    }
    if (BallShuttleControl_Start(
            &s_ball_shuttle_control, now_ms) == 0U) {
        g_app_ball_shuttle_start_reject =
            (uint8_t) APP_SHUTTLE_START_REJECT_CONFIG;
        return 0U;
    }

    target_pipe_pitch_deg =
        BallShuttleControl_GetTargetPipePitchDeg(
            &s_ball_shuttle_control);
    BallBalanceControl_SetTransportMode(&s_ball_balance_control, 0U);
    BallBalanceControl_SetEndpointHoldEnabled(
        &s_ball_balance_control, 0U);
    BallBalanceControl_SetManualPipeMode(
        &s_ball_balance_control, 1U, target_pipe_pitch_deg);
    BallBalanceControl_SetEnabled(&s_ball_balance_control, 1U, now_ms);
    App_UpdateBallShuttleDebug();
    App_QueueTjcShuttlePhase(1U, target_pipe_pitch_deg);
    return 1U;
}

uint8_t App_StopBallShuttle(void)
{
    uint32_t now_ms = Board_TimeMs();

    if ((BallShuttleControl_IsRunning(&s_ball_shuttle_control) == 0U) &&
        (BallShuttleControl_IsHolding(&s_ball_shuttle_control) == 0U)) {
        return 0U;
    }

    BallShuttleControl_HoldLevel(&s_ball_shuttle_control, now_ms);
    BallBalanceControl_SetTransportMode(&s_ball_balance_control, 0U);
    BallBalanceControl_SetEndpointHoldEnabled(
        &s_ball_balance_control, 0U);
    BallBalanceControl_SetManualPipeMode(
        &s_ball_balance_control, 1U, 0.0f);
    BallBalanceControl_SetEnabled(&s_ball_balance_control, 1U, now_ms);
    App_UpdateBallShuttleDebug();
    App_QueueTjcStatus(TJC_STATUS_SHUTTLE_SUCCESS);
    return 1U;
}

uint8_t App_SetBallShuttleConfig(float phase_1_angle_deg,
                                 uint32_t phase_1_time_ms,
                                 float phase_2_angle_deg,
                                 uint32_t phase_2_time_ms,
                                 float phase_3_angle_deg,
                                 uint32_t phase_3_time_ms)
{
    BallShuttleConfig config;

    config.angle_magnitude_deg[0] = phase_1_angle_deg;
    config.angle_magnitude_deg[1] = phase_2_angle_deg;
    config.angle_magnitude_deg[2] = phase_3_angle_deg;
    config.duration_ms[0] = phase_1_time_ms;
    config.duration_ms[1] = phase_2_time_ms;
    config.duration_ms[2] = phase_3_time_ms;
    if (BallShuttleControl_SetConfig(
            &s_ball_shuttle_control, &config) == 0U) {
        return 0U;
    }
    App_UpdateBallShuttleDebug();
    return 1U;
}

uint8_t App_SaveBallShuttleTimes(uint32_t phase_1_time_ms,
                                 uint32_t phase_2_time_ms,
                                 uint32_t phase_3_time_ms)
{
    BallShuttleConfig previous;
    BallShuttleConfig updated;

    if ((s_current_mode != APP_MODE_STOP) ||
        (BallShuttleControl_IsRunning(&s_ball_shuttle_control) != 0U) ||
        (BallShuttleControl_IsHolding(&s_ball_shuttle_control) != 0U) ||
        (DynamicRunTask_IsActive(&s_dynamic_run_task) != 0U) ||
        (App_IsPipeCalibrationBusy() != 0U)) {
        return 0U;
    }

    BallShuttleControl_GetConfig(&s_ball_shuttle_control, &previous);
    updated = previous;
    updated.duration_ms[0] = phase_1_time_ms;
    updated.duration_ms[1] = phase_2_time_ms;
    updated.duration_ms[2] = phase_3_time_ms;
    if (BallShuttleControl_SetConfig(
            &s_ball_shuttle_control, &updated) == 0U) {
        return 0U;
    }
    if (App_ShuttleNvmSave(updated.duration_ms) == 0U) {
        (void) BallShuttleControl_SetConfig(
            &s_ball_shuttle_control, &previous);
        App_UpdateBallShuttleDebug();
        return 0U;
    }
    App_UpdateBallShuttleDebug();
    return 1U;
}

void App_GetBallShuttleConfig(float *phase_1_angle_deg,
                              uint32_t *phase_1_time_ms,
                              float *phase_2_angle_deg,
                              uint32_t *phase_2_time_ms,
                              float *phase_3_angle_deg,
                              uint32_t *phase_3_time_ms)
{
    BallShuttleConfig config;

    BallShuttleControl_GetConfig(&s_ball_shuttle_control, &config);
    if (phase_1_angle_deg != 0) {
        *phase_1_angle_deg = config.angle_magnitude_deg[0];
    }
    if (phase_1_time_ms != 0) {
        *phase_1_time_ms = config.duration_ms[0];
    }
    if (phase_2_angle_deg != 0) {
        *phase_2_angle_deg = config.angle_magnitude_deg[1];
    }
    if (phase_2_time_ms != 0) {
        *phase_2_time_ms = config.duration_ms[1];
    }
    if (phase_3_angle_deg != 0) {
        *phase_3_angle_deg = config.angle_magnitude_deg[2];
    }
    if (phase_3_time_ms != 0) {
        *phase_3_time_ms = config.duration_ms[2];
    }
}

uint8_t App_ResetBallShuttleConfig(void)
{
    if (BallShuttleControl_ResetConfig(
            &s_ball_shuttle_control) == 0U) {
        return 0U;
    }
    App_UpdateBallShuttleDebug();
    return 1U;
}

uint8_t App_GetBallShuttleState(void)
{
    return (uint8_t) s_ball_shuttle_control.state;
}

uint8_t App_GetBallShuttleStartReject(void)
{
    return g_app_ball_shuttle_start_reject;
}

static uint8_t App_PreparePipeCalibration(void)
{
    if ((s_current_mode != APP_MODE_STOP) ||
        (DynamicRunTask_IsActive(&s_dynamic_run_task) != 0U) ||
        (PipeAttitudeControl_IsCalibrationActive(
             &s_pipe_attitude_control) != 0U) ||
        (App_GetHostCalibrationEnabled() != 0U)) {
        return 0U;
    }

    BallShuttleControl_Stop(&s_ball_shuttle_control);
    BallBalanceControl_SetManualPipeMode(
        &s_ball_balance_control, 0U, 0.0f);
    BallBalanceControl_SetTransportMode(&s_ball_balance_control, 0U);
    BallBalanceControl_SetEndpointHoldEnabled(&s_ball_balance_control, 0U);
    BallBalanceControl_SetEnabled(&s_ball_balance_control, 0U,
                                 Board_TimeMs());
    App_UpdateBallShuttleDebug();
    return 1U;
}

uint8_t App_StartPipeZeroAdjustment(void)
{
    if (s_pipe_zero_adjust_active != 0U) {
        return 1U;
    }
    if (App_PreparePipeCalibration() == 0U) {
        return 0U;
    }

    /* 进入新页面时先显示当前已确认零位，后续按钮只修改试调PWM。 */
    s_pipe_zero_pwm_us = ServoDriver_GetCenterPulseUs();
    ServoDriver_SetPulseUs(s_pipe_zero_pwm_us);
    s_pipe_zero_adjust_active = 1U;
    g_app_pipe_zero_adjust_active = 1U;
    g_app_pipe_zero_pwm_us = s_pipe_zero_pwm_us;
    return 1U;
}

uint8_t App_SetPipeZeroPwm(uint32_t pwm_us)
{
    if ((s_pipe_zero_adjust_active == 0U) ||
        (pwm_us < SERVO_DRIVER_MIN_PULSE_US) ||
        (pwm_us > SERVO_DRIVER_MAX_PULSE_US)) {
        return 0U;
    }

    s_pipe_zero_pwm_us = pwm_us;
    ServoDriver_SetPulseUs(s_pipe_zero_pwm_us);
    g_app_pipe_zero_pwm_us = s_pipe_zero_pwm_us;
    return 1U;
}

uint8_t App_ConfirmPipeZeroAndStartCalibration(uint32_t pwm_us)
{
    uint32_t previous_center_us;
    uint8_t accepted;

    if (App_SetPipeZeroPwm(pwm_us) == 0U) {
        return 0U;
    }

    previous_center_us = ServoDriver_GetCenterPulseUs();
    if (ServoDriver_SetCenterPulseUs(s_pipe_zero_pwm_us) == 0U) {
        return 0U;
    }
    accepted = PipeAttitudeControl_StartCalibration(
        &s_pipe_attitude_control,
        Board_TimeMs(),
        ServoDriver_GetPipePitchDeg());
    if (accepted == 0U) {
        (void) ServoDriver_SetCenterPulseUs(previous_center_us);
        ServoDriver_SetPulseUs(s_pipe_zero_pwm_us);
        return 0U;
    }

    /* 确认后才提交新的零位基准，IMU采样从该PWM对应的人工水平位置开始。 */
    s_pipe_zero_adjust_active = 0U;
    g_app_pipe_zero_adjust_active = 0U;
    g_app_pipe_zero_pwm_us = s_pipe_zero_pwm_us;
    return 1U;
}

uint8_t App_GetPipeZeroAdjustmentActive(void)
{
    return s_pipe_zero_adjust_active;
}

uint16_t App_GetPipeZeroPwmUs(void)
{
    return (uint16_t) s_pipe_zero_pwm_us;
}

uint8_t App_GetPipeCalibrationActive(void)
{
    return App_IsPipeCalibrationBusy();
}

uint8_t App_GetPipeCalibrationValid(void)
{
    return (uint8_t) ((s_pipe_zero_adjust_active == 0U) &&
                      (PipeAttitudeControl_IsCalibrationValid(
                           &s_pipe_attitude_control) != 0U));
}

void App_SetBallBalanceTargetMm(float target_mm)
{
    BallBalanceControl_SetTargetMm(&s_ball_balance_control, target_mm);
}

uint8_t App_SetBallPidGains(float position_kp,
                            float position_ki,
                            float position_kd,
                            float pipe_kp,
                            float pipe_kd_s)
{
    if ((position_kp < 0.0f) ||
        (position_kp > APP_BALL_POSITION_KP_MAX) ||
        (position_ki < 0.0f) ||
        (position_ki > APP_BALL_POSITION_KI_MAX) ||
        (position_kd < 0.0f) ||
        (position_kd > APP_BALL_POSITION_KD_MAX) ||
        (pipe_kp < 0.0f) ||
        (pipe_kp > APP_BALL_PIPE_KP_MAX) ||
        (pipe_kd_s < 0.0f) ||
        (pipe_kd_s > APP_BALL_PIPE_KD_MAX_S)) {
        return 0U;
    }

    g_app_ball_kp = position_kp;
    g_app_ball_ki = position_ki;
    g_app_ball_kd = position_kd;
    g_app_ball_pipe_kp = pipe_kp;
    g_app_ball_pipe_kd_s = pipe_kd_s;
    App_ApplyBallPidGains();
    return 1U;
}

void App_GetBallPidGains(float *position_kp,
                         float *position_ki,
                         float *position_kd,
                         float *pipe_kp,
                         float *pipe_kd_s)
{
    if (position_kp != 0) {
        *position_kp = g_app_ball_kp;
    }
    if (position_ki != 0) {
        *position_ki = g_app_ball_ki;
    }
    if (position_kd != 0) {
        *position_kd = g_app_ball_kd;
    }
    if (pipe_kp != 0) {
        *pipe_kp = g_app_ball_pipe_kp;
    }
    if (pipe_kd_s != 0) {
        *pipe_kd_s = g_app_ball_pipe_kd_s;
    }
}

void App_GetBallPidSnapshot(AppBallPidSnapshot *snapshot)
{
    if (snapshot == 0) {
        return;
    }

    snapshot->enabled = s_ball_balance_output.enabled;
    snapshot->tracking_valid = s_ball_balance_output.tracking_valid;
    snapshot->timed_out = s_ball_balance_output.timed_out;
    snapshot->integral_active = s_ball_balance_output.integral_active;
    snapshot->braking_active = s_ball_balance_output.braking_active;
    snapshot->attitude_valid = s_pipe_attitude_output.attitude_valid;
    snapshot->imu_fault = s_ball_balance_output.imu_fault;
    snapshot->mapping_fault = s_ball_balance_output.mapping_fault;
    snapshot->trajectory_phase = s_ball_balance_output.trajectory_phase;
    snapshot->target_mm = s_ball_balance_output.target_mm;
    snapshot->position_mm = s_ball_balance_output.filtered_position_mm;
    snapshot->velocity_mm_s = s_ball_balance_output.velocity_mm_s;
    snapshot->error_mm = s_ball_balance_output.error_mm;
    snapshot->p_term_deg = s_ball_balance_output.p_term_deg;
    snapshot->i_term_deg = s_ball_balance_output.i_term_deg;
    snapshot->d_term_deg = s_ball_balance_output.d_term_deg;
    snapshot->desired_pipe_pitch_deg =
        s_ball_balance_output.desired_pipe_pitch_deg;
    snapshot->measured_pipe_pitch_deg =
        s_ball_balance_output.measured_pipe_pitch_deg;
    snapshot->measured_pipe_rate_dps =
        s_ball_balance_output.measured_pipe_rate_dps;
    snapshot->pipe_p_term_deg = s_ball_balance_output.pipe_p_term_deg;
    snapshot->pipe_d_term_deg = s_ball_balance_output.pipe_d_term_deg;
    snapshot->servo_pwm_us = ServoDriver_GetPulseUs();
}

uint8_t App_SetHostCalibrationEnabled(uint8_t enabled)
{
    if (enabled != 0U) {
        /* 进入标定模式：先停车并禁用所有自动任务。
         * App_RequestMode(STOP) 内部已通过 App_SetBallBalanceEnabled(0)
         * 停止往返、禁用平衡、中止水管标定，这里只置标志；
         * 不要在此追加深层调用，CAL 路径与 1KB 栈的余量很小。 */
        if (App_RequestMode(APP_MODE_STOP) == 0U) {
            return 0U;
        }
        s_host_calibration_enabled = 1U;
    } else {
        s_host_calibration_enabled = 0U;
    }
    g_app_host_calibration_enabled = s_host_calibration_enabled;
    return 1U;
}

uint8_t App_GetHostCalibrationEnabled(void)
{
    return s_host_calibration_enabled;
}

float App_GetPipePitchDeg(void)
{
    return s_pipe_attitude_output.pitch_deg;
}

float App_GetPipePitchRateDps(void)
{
    return s_pipe_attitude_output.pitch_rate_dps;
}

uint8_t App_GetPipeAttitudeValid(void)
{
    return s_pipe_attitude_output.attitude_valid;
}

float App_GetPipeRawAccelPitchDeg(void)
{
    /* 原始加速度俯仰角不受单点水平标定门控，供USART1上位机直接使用。 */
    return s_pipe_attitude_output.raw_accel_pitch_deg;
}

float App_GetPipeRawPitchRateDps(void)
{
    ImuMotionSample sample;

    if (ImuHeading_GetLatestSample(&sample) != 0U) {
        return sample.gyro_x_dps;
    }
    return 0.0f;
}
