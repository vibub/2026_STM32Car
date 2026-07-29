#include "app_main.h"
#include "app_config.h"
#include "board_time.h"
#include "encoder_driver.h"
#include "imu_heading.h"
#include "motor_driver.h"
#include "protocol_tjc.h"
#include "tjc_uart.h"
#include "wheel_speed_pid.h"

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

static void App_ControlStep(void)
{
    EncoderSample encoder;

    EncoderDriver_Sample(&encoder);
    g_app_left_speed_cm_s = encoder.left_cm_s;
    g_app_right_speed_cm_s = encoder.right_cm_s;

    if (s_line_enabled == 0U) {
        App_StopClosedLoop();
        return;
    }

    /* Four-channel grayscale line following is the active sensor path now.
       Keep motors stopped until the grayscale sensor pins and polarity are wired in. */
    App_StopClosedLoop();
}

void App_Init(void)
{
    WheelSpeedPid_Init(&s_left_speed_pid);
    WheelSpeedPid_Init(&s_right_speed_pid);
    MotorDriver_Init();
    EncoderDriver_Init();
    ImuHeading_Init();
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

    s_last_control_ms = now_ms;
    App_ControlStep();
}

uint8_t App_RequestMode(uint8_t mode)
{
    if (mode == APP_MODE_STOP) {
        s_current_mode = APP_MODE_STOP;
        s_line_enabled = 0U;
        App_StopClosedLoop();
        return 1U;
    }

    if (mode == APP_MODE_GRAY_LINE) {
        s_current_mode = mode;
        s_line_enabled = 1U;
        return 1U;
    }

    return 0U;
}

uint8_t App_GetCurrentMode(void)
{
    return s_current_mode;
}


