#include "app_main.h"
#include "app_config.h"
#include "ball_balance_control.h"
#include "ball_position_protocol.h"
#include "board_time.h"
#include "encoder_driver.h"
#include "imu_heading.h"
#include "infrared_line_control.h"
#include "infrared_line_sensor.h"
#include "lap_stop_detector.h"
#include "k230_uart.h"
#include "motor_driver.h"
#include "oled_display.h"
#include "protocol_tjc.h"
#include "servo_driver.h"
#include "tjc_uart.h"
#include "wheel_speed_pid.h"

/* Debug variables for IAR Watch only; they do not drive module control. */
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
volatile float g_app_ball_desired_angle_deg;
volatile float g_app_ball_command_angle_deg;
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
static WheelSpeedPid s_left_speed_pid;
static WheelSpeedPid s_right_speed_pid;
static uint8_t s_current_mode;
static uint8_t s_line_enabled;
static uint8_t s_last_line_sensor_mask;
static uint8_t s_has_line_sensor_sample;
static uint8_t s_line_sensor_failure_count;
static uint32_t s_last_k230_overflow_count;
static uint32_t s_last_k230_error_count;

#define APP_LINE_SENSOR_FAILURE_LIMIT 3U /* 红外 I2C 连续读取失败后停车的次数门槛。 */

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
    g_app_ball_desired_angle_deg = s_ball_balance_output.desired_angle_deg;
    g_app_ball_command_angle_deg = s_ball_balance_output.command_angle_deg;
}

static void App_BallBalanceTask(uint32_t now_ms)
{
    BallPositionSample sample;
    BallBalanceMeasurement measurement;
    const BallBalanceMeasurement *measurement_ptr = 0;
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

    BallBalanceControl_Update(&s_ball_balance_control,
                              now_ms,
                              measurement_ptr,
                              &s_ball_balance_output);
    ServoDriver_SetAngle(s_ball_balance_output.command_angle_deg);
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
    BallBalanceControl_Init(&s_ball_balance_control);
    BallPositionProtocol_Init();
    WheelSpeedPid_Init(&s_left_speed_pid);
    WheelSpeedPid_Init(&s_right_speed_pid);
    InfraredLineSensor_Init();
    MotorDriver_Init();
    EncoderDriver_Init();
    ImuHeading_Init();
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
    BallBalanceControl_Update(&s_ball_balance_control,
                              s_last_control_ms,
                              0,
                              &s_ball_balance_output);
    App_UpdateBallBalanceDebug();
    App_UpdateLapStopDebug();
    App_StopClosedLoop();
}

void App_Loop(void)
{
    uint32_t now_ms;

    Protocol_Tjc_Task();
    ImuHeading_Poll();
    now_ms = Board_TimeMs();
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
        BallBalanceControl_SetTargetMm(&s_ball_balance_control, 0.0f);
        BallBalanceControl_SetEnabled(&s_ball_balance_control, 1U, now_ms);
    } else {
        BallBalanceControl_SetEnabled(&s_ball_balance_control, 0U, now_ms);
    }
    return 1U;
}

uint8_t App_GetBallBalanceEnabled(void)
{
    return BallBalanceControl_IsEnabled(&s_ball_balance_control);
}

void App_SetBallBalanceTargetMm(float target_mm)
{
    BallBalanceControl_SetTargetMm(&s_ball_balance_control, target_mm);
}
