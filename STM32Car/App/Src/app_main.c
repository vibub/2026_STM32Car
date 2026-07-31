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
volatile float g_app_ball_transport_command_accel_cm_s2;
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
volatile uint8_t g_app_ball_shuttle_failure;
volatile uint8_t g_app_ball_shuttle_start_pending;
volatile uint8_t g_app_ball_shuttle_stable_ready;
volatile uint32_t g_app_ball_shuttle_elapsed_ms;
volatile uint8_t g_app_pipe_calibration_state;
volatile uint8_t g_app_pipe_calibration_active;
volatile uint8_t g_app_pipe_calibration_valid;
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
static BallBalanceControl s_ball_balance_control;
static BallBalanceOutput s_ball_balance_output;
static BallShuttleControl s_ball_shuttle_control;
static PipeAttitudeControl s_pipe_attitude_control;
static PipeAttitudeOutput s_pipe_attitude_output;
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
static uint8_t s_lap_stop_enabled;
static uint8_t s_last_ball_active;
static uint8_t s_last_ball_imu_fault;
static uint8_t s_ball_shuttle_start_pending;
static uint32_t s_ball_shuttle_prepare_since_ms;
static uint8_t s_tjc_status_pending;
static Tjc_StatusText s_pending_tjc_status;
static uint8_t s_host_calibration_enabled = APP_HOST_CALIBRATION_DEFAULT;

volatile uint8_t g_app_host_calibration_enabled;

static DynamicRunTask s_dynamic_run_task;
static uint8_t s_dynamic_route_passed;
static uint8_t s_ball_limit_warning;

static void App_DynamicRunTask(uint32_t now_ms,
                               uint8_t evaluate_completion,
                               uint8_t include_measurement);
static void App_DynamicRunFail(uint32_t now_ms,
                               DynamicRunFailure failure);

#define APP_LINE_SENSOR_FAILURE_LIMIT       3U    /* 红外 I2C 连续读取失败后停车的次数门槛。 */
#define APP_BALL_SHUTTLE_POSITIVE_TARGET_MM 50.0f /* 第三题第一阶段的正向目标位置。 */
#define APP_BALL_SHUTTLE_NEGATIVE_TARGET_MM -50.0f /* 第三题折返后的最终目标位置。 */
#define APP_BALL_SHUTTLE_PREPARE_TIMEOUT_MS 3000U /* 自动启用平衡和等待视觉有效的最长时间。 */

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
    APP_DYNAMIC_PREPARE_TIMEOUT_MS,       /* 等待钢球回中稳定的最长时间。 */
    APP_DYNAMIC_READY_HOLD_MS,            /* 中心条件连续保持的起步确认时间。 */
    APP_DYNAMIC_READY_SAMPLE_GAP_MS,      /* 准备阶段视觉样本间隔上限。 */
    APP_AB_TIMEOUT_MS,                    /* 第四题从起步到通过 B 的限时。 */
    APP_ABCD_TIMEOUT_MS,                  /* 第五题从起步到通过 A 的限时。 */
    APP_DYNAMIC_READY_POSITION_MM,        /* 允许起步的位置误差绝对值。 */
    APP_DYNAMIC_READY_SPEED_MM_S,         /* 允许起步的钢球速度绝对值。 */
    APP_BALL_LIMIT_WARNING_MM,            /* 红圈和超差统计的误差门槛。 */
    APP_BALL_LIMIT_CLEAR_MM,              /* 红圈警告的回差清除门槛。 */
    APP_BALL_LIMIT_CLEAR_SAMPLES,         /* 清除红圈需要连续满足的样本数。 */
    APP_BALL_OVERRANGE_CONFIRM_SAMPLES    /* 锁存超差结果需要连续越界的样本数。 */
};

static float App_Abs(float value)
{
    return (value < 0.0f) ? -value : value;
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
    g_app_ball_transport_command_accel_cm_s2 =
        s_ball_balance_output.transport_command_accel_cm_s2;
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
    g_app_ball_shuttle_state = (uint8_t) s_ball_shuttle_control.state;
    g_app_ball_shuttle_failure = (uint8_t) s_ball_shuttle_control.failure;
    g_app_ball_shuttle_start_pending = s_ball_shuttle_start_pending;
    g_app_ball_shuttle_stable_ready =
        s_ball_shuttle_control.stable_ready;
    g_app_ball_shuttle_elapsed_ms = s_ball_shuttle_control.elapsed_ms;
}

static void App_UpdatePipeAttitudeDebug(void)
{
    uint8_t index;

    g_app_pipe_calibration_state =
        (uint8_t) s_pipe_attitude_output.calibration_state;
    g_app_pipe_calibration_active =
        s_pipe_attitude_output.calibration_active;
    g_app_pipe_calibration_valid =
        s_pipe_attitude_output.calibration_valid;
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

static void App_SendPendingTjcStatus(void)
{
    if (s_tjc_status_pending == 0U) {
        return;
    }

    s_tjc_status_pending = 0U;
    Protocol_Tjc_SendStatusText(s_pending_tjc_status);
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
    BallShuttleMeasurement measurement;
    BallShuttleEvent event;

    if (s_ball_shuttle_start_pending != 0U) {
        if ((s_ball_balance_output.imu_fault != 0U) ||
            (s_ball_balance_output.mapping_fault != 0U) ||
            ((uint32_t) (now_ms - s_ball_shuttle_prepare_since_ms) >
             APP_BALL_SHUTTLE_PREPARE_TIMEOUT_MS)) {
            s_ball_shuttle_start_pending = 0U;
            BallBalanceControl_SetEndpointHoldEnabled(
                &s_ball_balance_control, 0U);
            BallBalanceControl_SetEnabled(&s_ball_balance_control,
                                          0U,
                                          now_ms);
            App_QueueTjcStatus(TJC_STATUS_SHUTTLE_FAILED);
            App_UpdateBallShuttleDebug();
            return;
        }
        if ((s_ball_balance_output.enabled != 0U) &&
            (s_ball_balance_output.tracking_valid != 0U) &&
            (s_ball_balance_output.timed_out == 0U)) {
            if (BallShuttleControl_Start(
                    &s_ball_shuttle_control, now_ms) != 0U) {
                s_ball_shuttle_start_pending = 0U;
                BallBalanceControl_SetEndpointHoldEnabled(
                    &s_ball_balance_control, 0U);
                BallBalanceControl_SetTargetMm(
                    &s_ball_balance_control,
                    APP_BALL_SHUTTLE_POSITIVE_TARGET_MM);
                App_QueueTjcStatus(TJC_STATUS_SHUTTLE_RUNNING);
            } else {
                s_ball_shuttle_start_pending = 0U;
                BallBalanceControl_SetEnabled(&s_ball_balance_control,
                                              0U,
                                              now_ms);
                App_QueueTjcStatus(TJC_STATUS_SHUTTLE_FAILED);
            }
        }
        App_UpdateBallShuttleDebug();
        return;
    }

    measurement.balance_enabled = s_ball_balance_output.enabled;
    measurement.tracking_valid = s_ball_balance_output.tracking_valid;
    measurement.timed_out = s_ball_balance_output.timed_out;
    measurement.imu_fault = s_ball_balance_output.imu_fault;
    measurement.mapping_fault = s_ball_balance_output.mapping_fault;
    measurement.filtered_position_mm =
        s_ball_balance_output.filtered_position_mm;
    measurement.velocity_mm_s = s_ball_balance_output.velocity_mm_s;
    event = BallShuttleControl_Update(&s_ball_shuttle_control,
                                      now_ms,
                                      &measurement);

    switch (event) {
        case BALL_SHUTTLE_EVENT_TURN_NEGATIVE:
            BallBalanceControl_SetTargetMm(
                &s_ball_balance_control,
                APP_BALL_SHUTTLE_NEGATIVE_TARGET_MM);
            App_QueueTjcStatus(TJC_STATUS_SHUTTLE_TURNING);
            break;

        case BALL_SHUTTLE_EVENT_ENABLE_ENDPOINT_HOLD:
            BallBalanceControl_SetEndpointHoldEnabled(
                &s_ball_balance_control, 1U);
            break;

        case BALL_SHUTTLE_EVENT_SUCCEEDED:
            App_QueueTjcStatus(TJC_STATUS_SHUTTLE_SUCCESS);
            break;

        case BALL_SHUTTLE_EVENT_FAILED:
            BallBalanceControl_SetEndpointHoldEnabled(
                &s_ball_balance_control, 0U);
            BallBalanceControl_SetEnabled(&s_ball_balance_control,
                                          0U,
                                          now_ms);
            App_QueueTjcStatus(TJC_STATUS_SHUTTLE_FAILED);
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
    BallBalanceControl_Update(&s_ball_balance_control,
                              now_ms,
                              measurement_ptr,
                              attitude_ptr,
                              &s_ball_balance_output);
    App_BallShuttleTask(now_ms);
    App_DynamicRunTask(now_ms, 0U, 1U);
    if ((s_pipe_attitude_output.servo_override_active == 0U) &&
        (App_GetHostCalibrationEnabled() == 0U)) {
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
    uint8_t app_mode)
{
    ChassisMotionConfig motion_config;
    float initial_speed_cm_s = 0.0f;

    if (requested_config == 0) {
        return 0U;
    }
    motion_config = *requested_config;
    if ((s_line_enabled != 0U) &&
        (s_chassis_accel_output.valid != 0U)) {
        initial_speed_cm_s = s_chassis_accel_output.filtered_speed_cm_s;
        if (initial_speed_cm_s < 0.0f) {
            initial_speed_cm_s = 0.0f;
        }
    }

    InfraredLineControl_Init(&s_line_control);
    if (ChassisMotionProfile_Start(&s_motion_profile,
                                   now_ms,
                                   initial_speed_cm_s,
                                   &motion_config) == 0U) {
        return 0U;
    }
    ChassisMotionProfile_Update(&s_motion_profile, now_ms, 0.0f,
                                &s_motion_output);
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
            return (s_dynamic_run_task.mode == DYNAMIC_RUN_MODE_AB) ?
                   TJC_STATUS_AB_FAILED : TJC_STATUS_ABCD_FAILED;
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
    return (s_dynamic_run_task.mode == DYNAMIC_RUN_MODE_AB) ?
           APP_MODE_AB : APP_MODE_ABCD;
}

static Tjc_StatusText App_DynamicRunRunningStatus(void)
{
    return (s_dynamic_run_task.mode == DYNAMIC_RUN_MODE_AB) ?
           TJC_STATUS_AB_RUNNING : TJC_STATUS_ABCD_RUNNING;
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
        lap_stop_enabled = (uint8_t) (
            s_dynamic_run_task.mode == DYNAMIC_RUN_MODE_ABCD);
        BallBalanceControl_SetTransportMode(&s_ball_balance_control, 1U);
        if (App_StartLineRunInternal(now_ms,
                                     motion_config,
                                     lap_stop_enabled,
                                     app_mode) == 0U) {
            App_DynamicRunFail(now_ms,
                               DYNAMIC_RUN_FAILURE_MOTION_START);
            return;
        }
        DynamicRunTask_ConfirmMotionStarted(&s_dynamic_run_task, now_ms);
        App_QueueTjcStatus(App_DynamicRunRunningStatus());
    }

    if (actions.request_soft_stop != 0U) {
        ChassisMotionProfile_RequestStop(&s_motion_profile);
        App_TimerStop(now_ms);
        Protocol_Tjc_SendDynamicRunResult(
            (uint8_t) s_dynamic_run_task.mode,
            (uint8_t) s_dynamic_run_task.result,
            s_dynamic_run_task.elapsed_ms,
            s_dynamic_run_task.maximum_abs_error_mm);
    }

    if (actions.enter_stop_mode != 0U) {
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
            (s_dynamic_run_task.mode == DYNAMIC_RUN_MODE_ABCD)) {
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

void App_Init(void)
{
    InfraredLineControl_Init(&s_line_control);
    LapStopDetector_Init(&s_lap_stop_detector);
    ChassisMotionProfile_Init(&s_motion_profile);
    /* 配置来自编译期常量，参数合法性由主机单元测试覆盖。 */
    (void) DynamicRunTask_Init(&s_dynamic_run_task,
                               &s_dynamic_run_config);
    ChassisAccelEstimator_Init(&s_chassis_accel_estimator);
    PipeAttitudeControl_Init(&s_pipe_attitude_control);
    BallBalanceControl_Init(&s_ball_balance_control);
    BalanceDebugLog_Init();
    App_ApplyBallPidGains();
    BallShuttleControl_Init(&s_ball_shuttle_control);
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
    ServoDriver_InitPulseUs(SERVO_DRIVER_CENTER_PULSE_US);
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
    s_lap_stop_enabled = 0U;
    s_last_ball_active = 0U;
    s_last_ball_imu_fault = 0U;
    s_ball_shuttle_start_pending = 0U;
    s_ball_shuttle_prepare_since_ms = 0U;
    s_dynamic_route_passed = 0U;
    s_ball_limit_warning = 0U;
    g_app_dynamic_run_mode = (uint8_t) DYNAMIC_RUN_MODE_NONE;
    g_app_dynamic_run_state = (uint8_t) DYNAMIC_RUN_STATE_IDLE;
    g_app_dynamic_run_result = (uint8_t) DYNAMIC_RUN_RESULT_NONE;
    g_app_dynamic_run_failure = (uint8_t) DYNAMIC_RUN_FAILURE_NONE;
    g_app_dynamic_start_reject =
        (uint8_t) APP_DYNAMIC_START_REJECT_NONE;
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
    App_TimerTask(now_ms);
}

uint8_t App_RequestMode(uint8_t mode)
{
    if (mode == APP_MODE_STOP) {
        DynamicRunTask_Cancel(&s_dynamic_run_task);
        s_dynamic_route_passed = 0U;
        App_SetBallLimitWarning(0U);
        App_SetBallBalanceEnabled(0U);
        App_EnterStopMode(Board_TimeMs());
        App_UpdateChassisMotionDebug();
        return 1U;
    }

    if (mode == APP_MODE_BASIC_LINE) {
        if ((DynamicRunTask_IsActive(&s_dynamic_run_task) != 0U) ||
            (PipeAttitudeControl_IsCalibrationActive(
                 &s_pipe_attitude_control) != 0U) ||
            (s_ball_shuttle_start_pending != 0U) ||
            (BallShuttleControl_IsRunning(&s_ball_shuttle_control) != 0U) ||
            (BallShuttleControl_IsHolding(&s_ball_shuttle_control) != 0U) ||
            ((BallBalanceControl_IsEnabled(&s_ball_balance_control) != 0U) &&
             (BallBalanceControl_IsActive(&s_ball_balance_control) == 0U))) {
            return 0U;
        }
        return App_StartLineRunInternal(Board_TimeMs(),
                                        &s_default_motion_config,
                                        1U,
                                        APP_MODE_BASIC_LINE);
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
        (PipeAttitudeControl_IsCalibrationActive(
             &s_pipe_attitude_control) != 0U) ||
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
                                    APP_MODE_BASIC_LINE);
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
                                   Tjc_StatusText preparing_status)
{
    uint32_t now_ms = Board_TimeMs();

    g_app_dynamic_start_reject =
        (uint8_t) APP_DYNAMIC_START_REJECT_NONE;
    if (s_current_mode != APP_MODE_STOP) {
        return App_RejectDynamicStart(
            APP_DYNAMIC_START_REJECT_NOT_STOPPED,
            TJC_STATUS_RUN_REJECT_STOP);
    }
    if (PipeAttitudeControl_IsCalibrationActive(
            &s_pipe_attitude_control) != 0U) {
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
    if ((s_ball_shuttle_start_pending != 0U) ||
        (BallShuttleControl_IsRunning(&s_ball_shuttle_control) != 0U) ||
        (BallShuttleControl_IsHolding(&s_ball_shuttle_control) != 0U) ||
        (DynamicRunTask_IsActive(&s_dynamic_run_task) != 0U)) {
        return App_RejectDynamicStart(
            APP_DYNAMIC_START_REJECT_OTHER_TASK,
            TJC_STATUS_RUN_REJECT_BUSY);
    }

    BallShuttleControl_Stop(&s_ball_shuttle_control);
    BallBalanceControl_SetEndpointHoldEnabled(&s_ball_balance_control, 0U);
    BallBalanceControl_SetTargetMm(&s_ball_balance_control, 0.0f);
    /* 准备阶段先使用完整位置控制回中，真正起步前再切换混合前馈。 */
    BallBalanceControl_SetTransportMode(&s_ball_balance_control, 0U);
    BallBalanceControl_SetEnabled(&s_ball_balance_control, 1U, now_ms);
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
    App_UpdateChassisMotionDebug();
    return 1U;
}

uint8_t App_StartAbRun(void)
{
    return App_StartDynamicRun(DYNAMIC_RUN_MODE_AB,
                               APP_MODE_AB,
                               TJC_STATUS_AB_PREPARING);
}

uint8_t App_StartAbcdRun(void)
{
    return App_StartDynamicRun(DYNAMIC_RUN_MODE_ABCD,
                               APP_MODE_ABCD,
                               TJC_STATUS_ABCD_PREPARING);
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
            (PipeAttitudeControl_IsCalibrationActive(
                 &s_pipe_attitude_control) != 0U) ||
            (PipeAttitudeControl_IsCalibrationValid(
                 &s_pipe_attitude_control) == 0U) ||
            (s_ball_shuttle_start_pending != 0U) ||
            (BallShuttleControl_IsRunning(
                 &s_ball_shuttle_control) != 0U) ||
            (BallShuttleControl_IsHolding(
                 &s_ball_shuttle_control) != 0U)) {
            return 0U;
        }
        s_ball_shuttle_start_pending = 0U;
        BallShuttleControl_Stop(&s_ball_shuttle_control);
        BallBalanceControl_SetTransportMode(&s_ball_balance_control, 0U);
        BallBalanceControl_SetEndpointHoldEnabled(
            &s_ball_balance_control, 0U);
        BallBalanceControl_SetTargetMm(&s_ball_balance_control, 0.0f);
        BallBalanceControl_SetEnabled(&s_ball_balance_control, 1U, now_ms);
        App_QueueTjcStatus(TJC_STATUS_BALANCE_CHECKING);
    } else {
        if (PipeAttitudeControl_IsCalibrationActive(
                &s_pipe_attitude_control) != 0U) {
            PipeAttitudeControl_AbortCalibration(&s_pipe_attitude_control,
                                                  now_ms);
        }
        s_ball_shuttle_start_pending = 0U;
        BallShuttleControl_Stop(&s_ball_shuttle_control);
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

uint8_t App_StartBallShuttle(void)
{
    uint32_t now_ms = Board_TimeMs();

    if ((s_current_mode != APP_MODE_STOP) ||
        (DynamicRunTask_IsActive(&s_dynamic_run_task) != 0U) ||
        (PipeAttitudeControl_IsCalibrationActive(
             &s_pipe_attitude_control) != 0U) ||
        (PipeAttitudeControl_IsCalibrationValid(
             &s_pipe_attitude_control) == 0U) ||
        (s_ball_shuttle_start_pending != 0U) ||
        (BallShuttleControl_IsRunning(&s_ball_shuttle_control) != 0U) ||
        (BallShuttleControl_IsHolding(&s_ball_shuttle_control) != 0U)) {
        return 0U;
    }

    BallShuttleControl_Stop(&s_ball_shuttle_control);
    BallBalanceControl_SetTransportMode(&s_ball_balance_control, 0U);
    BallBalanceControl_SetEndpointHoldEnabled(
        &s_ball_balance_control, 0U);
    BallBalanceControl_SetTargetMm(&s_ball_balance_control, 0.0f);
    BallBalanceControl_SetEnabled(&s_ball_balance_control, 1U, now_ms);
    if (BallBalanceControl_IsEnabled(&s_ball_balance_control) == 0U) {
        return 0U;
    }

    s_ball_shuttle_start_pending = 1U;
    s_ball_shuttle_prepare_since_ms = now_ms;
    App_UpdateBallShuttleDebug();
    App_QueueTjcStatus(TJC_STATUS_SHUTTLE_PREPARING);
    return 1U;
}

uint8_t App_StartPipeCalibration(void)
{
    if ((s_current_mode != APP_MODE_STOP) ||
        (DynamicRunTask_IsActive(&s_dynamic_run_task) != 0U) ||
        (PipeAttitudeControl_IsCalibrationActive(
             &s_pipe_attitude_control) != 0U)) {
        return 0U;
    }

    s_ball_shuttle_start_pending = 0U;
    BallShuttleControl_Stop(&s_ball_shuttle_control);
    BallBalanceControl_SetTransportMode(&s_ball_balance_control, 0U);
    BallBalanceControl_SetEndpointHoldEnabled(&s_ball_balance_control, 0U);
    BallBalanceControl_SetEnabled(&s_ball_balance_control, 0U,
                                 Board_TimeMs());
    App_UpdateBallShuttleDebug();
    return PipeAttitudeControl_StartCalibration(
        &s_pipe_attitude_control,
        Board_TimeMs(),
        ServoDriver_GetPipePitchDeg());
}

uint8_t App_GetPipeCalibrationActive(void)
{
    return PipeAttitudeControl_IsCalibrationActive(
        &s_pipe_attitude_control);
}

uint8_t App_GetPipeCalibrationValid(void)
{
    return PipeAttitudeControl_IsCalibrationValid(
        &s_pipe_attitude_control);
}

void App_SetBallBalanceTargetMm(float target_mm)
{
    BallBalanceControl_SetTargetMm(&s_ball_balance_control, target_mm);
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
