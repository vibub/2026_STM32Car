#include "app_main.h"
#include "app_config.h"
#include "board_time.h"
#include "encoder_driver.h"
#include "imu_heading.h"
#include "infrared_line_control.h"
#include "infrared_line_sensor.h"
#include "k230_uart.h"
#include "motor_driver.h"
#include "protocol_tjc.h"
#include "tjc_uart.h"
#include "wheel_speed_pid.h"

/* 这些变量用于 IAR Watch 调试，不参与模块间控制。 */
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

static uint32_t s_last_control_ms;
static InfraredLineControl s_line_control;
static WheelSpeedPid s_left_speed_pid;
static WheelSpeedPid s_right_speed_pid;
static uint8_t s_current_mode;
static uint8_t s_line_enabled;

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

static void App_ControlStep(uint32_t now_ms)
{
    EncoderSample encoder;
    InfraredLineOutput line_output;
    uint8_t sensor_mask;
    float left_command;
    float right_command;

    sensor_mask = InfraredLineSensor_ReadMask();
    InfraredLineControl_Update(&s_line_control, now_ms, sensor_mask,
                               &line_output);

    EncoderDriver_Sample(&encoder);
    g_app_left_speed_cm_s = encoder.left_cm_s;
    g_app_right_speed_cm_s = encoder.right_cm_s;
    g_app_line_sensor_mask = line_output.sensor_mask;
    g_app_line_active_count = line_output.active_count;
    g_app_line_lost = line_output.line_lost;
    g_app_line_error = line_output.filtered_error;
    g_app_line_correction_cm_s = line_output.correction_cm_s;

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
    WheelSpeedPid_Init(&s_left_speed_pid);
    WheelSpeedPid_Init(&s_right_speed_pid);
    InfraredLineSensor_Init();
    MotorDriver_Init();
    EncoderDriver_Init();
    ImuHeading_Init();
    K230Uart_Init();
    TjcUart_Init();
    Protocol_Tjc_Init();
    s_last_control_ms = Board_TimeMs();
    s_current_mode = APP_MODE_STOP;
    s_line_enabled = 0U;
    App_StopClosedLoop();
}

void App_Loop(void)
{
    uint32_t now_ms;

    Protocol_Tjc_Task();
    ImuHeading_Poll();
    now_ms = Board_TimeMs();
    if ((uint32_t) (now_ms - s_last_control_ms) < APP_CONTROL_PERIOD_MS) {
        return;
    }

    /* 超过多个周期时不连续补算，避免使用同一份编码器样本重复更新 PID。 */
    s_last_control_ms = now_ms;
    App_ControlStep(now_ms);
}

uint8_t App_RequestMode(uint8_t mode)
{
    if (mode == APP_MODE_STOP) {
        s_current_mode = APP_MODE_STOP;
        s_line_enabled = 0U;
        App_StopClosedLoop();
        return 1U;
    }

    if (mode == APP_MODE_BASIC_LINE) {
        s_current_mode = APP_MODE_BASIC_LINE;
        s_line_enabled = 1U;
        return 1U;
    }

    return 0U;
}

uint8_t App_GetCurrentMode(void)
{
    return s_current_mode;
}
