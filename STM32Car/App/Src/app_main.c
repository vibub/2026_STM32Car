#include "app_main.h"
#include "app_config.h"
#include "board_time.h"
#include "encoder_driver.h"
#include "imu_heading.h"
#include "k230_line_protocol.h"
#include "k230_uart.h"
#include "line_follow_control.h"
#include "motor_driver.h"
#include "wheel_speed_pid.h"
#include <string.h>

/* 这些变量用于 IAR Watch 调试，不参与模块间控制。 */
volatile uint8_t g_app_line_command_valid;
volatile uint8_t g_app_motor_fault;
volatile float g_app_left_target_cm_s;
volatile float g_app_right_target_cm_s;
volatile float g_app_left_speed_cm_s;
volatile float g_app_right_speed_cm_s;
volatile float g_app_left_pwm_ratio;
volatile float g_app_right_pwm_ratio;
volatile float g_app_line_error_px;
volatile float g_app_line_angle_deg;

static uint32_t s_last_control_ms;
static K230LineFrame s_line_frame;
static LineFollowControl s_line_control;
static WheelSpeedPid s_left_speed_pid;
static WheelSpeedPid s_right_speed_pid;

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
    LineFollowOutput line_output;
    uint8_t line_new;
    uint8_t line_fresh;
    float left_command;
    float right_command;

    line_new = K230LineProtocol_TakeLatest(&s_line_frame);
    line_fresh = (uint8_t) (
        (K230LineProtocol_HasFrame() != 0U) &&
        (K230LineProtocol_GetAgeMs(now_ms) <= APP_K230_LINK_TIMEOUT_MS));

    EncoderDriver_Sample(&encoder);
    g_app_left_speed_cm_s = encoder.left_cm_s;
    g_app_right_speed_cm_s = encoder.right_cm_s;

    LineFollowControl_Update(&s_line_control, now_ms, &s_line_frame,
                             line_new, line_fresh, &line_output);
    g_app_line_error_px = line_output.filtered_error_px;
    g_app_line_angle_deg = line_output.filtered_angle_deg;
    if (line_output.command_valid == 0U) {
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
    memset(&s_line_frame, 0, sizeof(s_line_frame));
    K230LineProtocol_Init();
    LineFollowControl_Init(&s_line_control);
    WheelSpeedPid_Init(&s_left_speed_pid);
    WheelSpeedPid_Init(&s_right_speed_pid);
    MotorDriver_Init();
    EncoderDriver_Init();
    ImuHeading_Init();
    K230Uart_Init();
    s_last_control_ms = Board_TimeMs();
    App_StopClosedLoop();
}

void App_Loop(void)
{
    uint32_t now_ms;

    K230LineProtocol_Poll();
    ImuHeading_Poll();
    now_ms = Board_TimeMs();
    if ((uint32_t) (now_ms - s_last_control_ms) < APP_CONTROL_PERIOD_MS) {
        return;
    }

    /* 超过多个周期时不连续补算，避免使用同一份编码器样本重复更新 PID。 */
    s_last_control_ms = now_ms;
    App_ControlStep(now_ms);
}
