#include "app_main.h"
#include "app_config.h"
#include "ball_balance_control.h"
#include "ball_position_protocol.h"
#include "ball_shuttle_control.h"
#include "board_time.h"
#include "encoder_driver.h"
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

/* Debug variables for IAR Watch only; they do not drive module control. */
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
volatile float g_app_ball_velocity_mm_s;
volatile float g_app_ball_error_mm;
volatile float g_app_ball_p_term_deg;
volatile float g_app_ball_i_term_deg;
volatile float g_app_ball_d_term_deg;
volatile float g_app_ball_desired_pipe_pitch_deg;
volatile float g_app_ball_command_pipe_pitch_deg;
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
volatile float g_app_chassis_accel_cm_s2;
volatile uint32_t g_app_k230_overflow_count;
volatile uint32_t g_app_k230_error_count;

static uint32_t s_last_control_ms;
static uint32_t s_oled_timer_start_ms;
static uint32_t s_oled_timer_elapsed_s;
static uint32_t s_oled_timer_last_display_s;
static uint8_t s_oled_timer_running;
static InfraredLineControl s_line_control;
static LapStopDetector s_lap_stop_detector;
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
static float s_last_chassis_speed_cm_s;
static float s_chassis_accel_cm_s2;
static uint8_t s_chassis_speed_ready;
static uint8_t s_last_ball_active;
static uint8_t s_last_ball_imu_fault;
static uint8_t s_ball_shuttle_start_pending;
static uint32_t s_ball_shuttle_prepare_since_ms;
static uint8_t s_tjc_status_pending;
static Tjc_StatusText s_pending_tjc_status;

#define APP_LINE_SENSOR_FAILURE_LIMIT       3U    /* 红外 I2C 连续读取失败后停车的次数门槛。 */
#define APP_BALL_SHUTTLE_POSITIVE_TARGET_MM 50.0f /* 第三题第一阶段的正向目标位置。 */
#define APP_BALL_SHUTTLE_NEGATIVE_TARGET_MM -50.0f /* 第三题折返后的最终目标位置。 */
#define APP_BALL_SHUTTLE_PREPARE_TIMEOUT_MS 3000U /* 自动启用平衡和等待视觉有效的最长时间。 */

static float App_Abs(float value)
{
    return (value < 0.0f) ? -value : value;
}

static void App_UpdateLapStopDebug(void)
{
    g_app_lap_distance_cm = s_lap_stop_detector.distance_cm;
    g_app_lap_stop_armed = s_lap_stop_detector.armed;
    g_app_lap_full_black_count = s_lap_stop_detector.full_black_count;
    g_app_lap_completed = s_lap_stop_detector.completed;
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
    g_app_ball_velocity_mm_s = s_ball_balance_output.velocity_mm_s;
    g_app_ball_error_mm = s_ball_balance_output.error_mm;
    g_app_ball_p_term_deg = s_ball_balance_output.p_term_deg;
    g_app_ball_i_term_deg = s_ball_balance_output.i_term_deg;
    g_app_ball_d_term_deg = s_ball_balance_output.d_term_deg;
    g_app_ball_desired_pipe_pitch_deg =
        s_ball_balance_output.desired_pipe_pitch_deg;
    g_app_ball_command_pipe_pitch_deg =
        s_ball_balance_output.command_pipe_pitch_deg;
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
    if (s_pipe_attitude_output.servo_override_active != 0U) {
        ServoDriver_SetAngle(s_pipe_attitude_output.servo_override_deg);
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

    BallBalanceControl_Update(&s_ball_balance_control,
                              now_ms,
                              measurement_ptr,
                              attitude_ptr,
                              &s_ball_balance_output);
    App_BallShuttleTask(now_ms);
    if (s_pipe_attitude_output.servo_override_active == 0U) {
        ServoDriver_SetAngle(s_ball_balance_output.command_angle_deg);
    }

    if (s_ball_balance_output.enabled != s_last_ball_active) {
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
    App_StopClosedLoop();
    App_TimerStop(now_ms);
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
    float chassis_speed_cm_s;

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
    chassis_speed_cm_s = 0.5f * (encoder.left_cm_s + encoder.right_cm_s);
    if (s_chassis_speed_ready != 0U) {
        s_chassis_accel_cm_s2 =
            (chassis_speed_cm_s - s_last_chassis_speed_cm_s) /
            ((float) APP_CONTROL_PERIOD_MS * 0.001f);
    } else {
        s_chassis_accel_cm_s2 = 0.0f;
        s_chassis_speed_ready = 1U;
    }
    s_last_chassis_speed_cm_s = chassis_speed_cm_s;
    g_app_chassis_accel_cm_s2 = s_chassis_accel_cm_s2;

    if (sensor_failure_stop != 0U) {
        g_app_line_sensor_mask = sensor_mask;
        g_app_line_active_count = 0U;
        g_app_line_lost = 1U;
        g_app_line_error = 0.0f;
        g_app_line_correction_cm_s = 0.0f;
        g_app_motor_fault = 0U;
        App_StopClosedLoop();
        return;
    }

    if (s_line_enabled != 0U) {
        /* 将固定 20 ms 轮速样本还原为本周期左右轮平均行驶距离。 */
        distance_delta_cm = 0.5f *
                            (App_Abs(encoder.left_cm_s) +
                             App_Abs(encoder.right_cm_s)) *
                            ((float) APP_CONTROL_PERIOD_MS / 1000.0f);
        lap_stop_completed = LapStopDetector_Update(&s_lap_stop_detector,
                                                    distance_delta_cm,
                                                    sensor_read_ok,
                                                    sensor_mask);
        /* 首次满足终点黑线数量门槛时，先请求巡线控制器降低基础速度。 */
        if (s_lap_stop_detector.full_black_count != 0U) {
            force_wide_slowdown = 1U;
        }
        App_UpdateLapStopDebug();
    }

    InfraredLineControl_Update(&s_line_control, now_ms, sensor_mask,
                               force_wide_slowdown, &line_output);
    g_app_line_sensor_mask = line_output.sensor_mask;
    g_app_line_active_count = line_output.active_count;
    g_app_line_lost = line_output.line_lost;
    g_app_line_error = line_output.filtered_error;
    g_app_line_correction_cm_s = line_output.correction_cm_s;

    if (lap_stop_completed != 0U) {
        App_EnterStopMode(now_ms);
        return;
    }

    /* 上电默认停车，收到 TJC 基础巡线命令后才允许闭环输出。 */
    if ((s_line_enabled == 0U) || (line_output.command_valid == 0U)) {
        App_StopClosedLoop();
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
        App_StopClosedLoop();
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
    PipeAttitudeControl_Init(&s_pipe_attitude_control);
    BallBalanceControl_Init(&s_ball_balance_control);
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
    ServoDriver_Init(BallBalanceControl_GetNeutralAngleDeg());
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
    s_last_chassis_speed_cm_s = 0.0f;
    s_chassis_accel_cm_s2 = 0.0f;
    s_chassis_speed_ready = 0U;
    s_last_ball_active = 0U;
    s_last_ball_imu_fault = 0U;
    s_ball_shuttle_start_pending = 0U;
    s_ball_shuttle_prepare_since_ms = 0U;
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
    App_UpdatePipeAttitudeDebug();
    App_UpdateBallBalanceDebug();
    App_UpdateBallShuttleDebug();
    App_UpdateLapStopDebug();
    App_StopClosedLoop();
    g_app_init_step = 100U;
}

void App_Loop(void)
{
    uint32_t now_ms;

    Protocol_Tjc_Task();
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
    }
    App_TimerTask(now_ms);
}

uint8_t App_RequestMode(uint8_t mode)
{
    if (mode == APP_MODE_STOP) {
        App_SetBallBalanceEnabled(0U);
        App_EnterStopMode(Board_TimeMs());
        return 1U;
    }

    if (mode == APP_MODE_BASIC_LINE) {
        if ((PipeAttitudeControl_IsCalibrationActive(
                 &s_pipe_attitude_control) != 0U) ||
            (s_ball_shuttle_start_pending != 0U) ||
            (BallShuttleControl_IsRunning(&s_ball_shuttle_control) != 0U) ||
            (BallShuttleControl_IsHolding(&s_ball_shuttle_control) != 0U) ||
            ((BallBalanceControl_IsEnabled(&s_ball_balance_control) != 0U) &&
             (BallBalanceControl_IsActive(&s_ball_balance_control) == 0U))) {
            return 0U;
        }
        LapStopDetector_Init(&s_lap_stop_detector);
        App_UpdateLapStopDebug();
        s_current_mode = APP_MODE_BASIC_LINE;
        s_line_enabled = 1U;
        App_TimerResetAndStart(Board_TimeMs());
        return 1U;
    }

    return 0U;
}

uint8_t App_GetCurrentMode(void)
{
    return s_current_mode;
}

uint8_t App_SetBallBalanceEnabled(uint8_t enabled)
{
    uint32_t now_ms = Board_TimeMs();

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
        (PipeAttitudeControl_IsCalibrationActive(
             &s_pipe_attitude_control) != 0U)) {
        return 0U;
    }

    s_ball_shuttle_start_pending = 0U;
    BallShuttleControl_Stop(&s_ball_shuttle_control);
    BallBalanceControl_SetEndpointHoldEnabled(&s_ball_balance_control, 0U);
    BallBalanceControl_SetEnabled(&s_ball_balance_control, 0U,
                                 Board_TimeMs());
    App_UpdateBallShuttleDebug();
    return PipeAttitudeControl_StartCalibration(
        &s_pipe_attitude_control,
        Board_TimeMs(),
        ServoDriver_GetAngle());
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
