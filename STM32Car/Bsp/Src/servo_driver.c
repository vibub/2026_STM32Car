#include "servo_driver.h"
#include "main.h"

/* TIM4 以 1 MHz 计数，500~2500 us 脉宽映射至 SG90 的 0~180°。 */
#define SERVO_DRIVER_MIN_PULSE_US       500U
#define SERVO_DRIVER_MAX_PULSE_US       2500U
#define SERVO_DRIVER_START_ANGLE_DEG    SERVO_DRIVER_MIN_ANGLE_DEG
#define SERVO_DRIVER_END_ANGLE_DEG      SERVO_DRIVER_MAX_ANGLE_DEG
#define SERVO_DRIVER_SWEEP_STEP_DEG     1.0f
#define SERVO_DRIVER_SWEEP_PERIOD_MS    30U

static float s_angle_deg = SERVO_DRIVER_START_ANGLE_DEG;
static int8_t s_sweep_direction = 1;
static uint8_t s_sweep_active;
static uint32_t s_last_sweep_ms;

static float ServoDriver_LimitAngle(float angle_deg)
{
    if (angle_deg < SERVO_DRIVER_MIN_ANGLE_DEG) {
        return SERVO_DRIVER_MIN_ANGLE_DEG;
    }
    if (angle_deg > SERVO_DRIVER_MAX_ANGLE_DEG) {
        return SERVO_DRIVER_MAX_ANGLE_DEG;
    }
    return angle_deg;
}

static uint32_t ServoDriver_AngleToPulseUs(float angle_deg)
{
    float ratio;
    float pulse_us;

    angle_deg = ServoDriver_LimitAngle(angle_deg);
    ratio = angle_deg / SERVO_DRIVER_MAX_ANGLE_DEG;
    pulse_us = (float) SERVO_DRIVER_MIN_PULSE_US +
               ratio * (float) (SERVO_DRIVER_MAX_PULSE_US -
                                SERVO_DRIVER_MIN_PULSE_US);
    return (uint32_t) (pulse_us + 0.5f);
}

void ServoDriver_Init(void)
{
    s_sweep_active = 0U;
    s_sweep_direction = 1;
    s_last_sweep_ms = 0U;
    ServoDriver_SetAngle(SERVO_DRIVER_START_ANGLE_DEG);
    if (HAL_TIM_PWM_Start(&htim4, SERVO_PWM_CHANNEL) != HAL_OK) {
        Error_Handler();
    }
    ServoDriver_SetAngle(SERVO_DRIVER_START_ANGLE_DEG);
}

void ServoDriver_SetAngle(float angle_deg)
{
    s_angle_deg = ServoDriver_LimitAngle(angle_deg);
    __HAL_TIM_SET_COMPARE(&htim4, SERVO_PWM_CHANNEL,
                          ServoDriver_AngleToPulseUs(s_angle_deg));
}

float ServoDriver_GetAngle(void)
{
    return s_angle_deg;
}

void ServoDriver_StartContinuousSweep(void)
{
    s_sweep_active = 1U;
    s_sweep_direction = 1;
    s_last_sweep_ms = 0U;
    ServoDriver_SetAngle(SERVO_DRIVER_START_ANGLE_DEG);
}

void ServoDriver_StartPowerOnClockwiseSweep(void)
{
    ServoDriver_StartContinuousSweep();
}

void ServoDriver_Task(uint32_t now_ms)
{
    if (s_sweep_active == 0U) {
        return;
    }
    if ((uint32_t) (now_ms - s_last_sweep_ms) < SERVO_DRIVER_SWEEP_PERIOD_MS) {
        return;
    }

    s_last_sweep_ms = now_ms;
    if (s_sweep_direction > 0) {
        if (s_angle_deg >= SERVO_DRIVER_END_ANGLE_DEG) {
            s_sweep_direction = -1;
            ServoDriver_SetAngle(SERVO_DRIVER_END_ANGLE_DEG);
            return;
        }
        ServoDriver_SetAngle(s_angle_deg + SERVO_DRIVER_SWEEP_STEP_DEG);
        return;
    }

    if (s_angle_deg <= SERVO_DRIVER_START_ANGLE_DEG) {
        s_sweep_direction = 1;
        ServoDriver_SetAngle(SERVO_DRIVER_START_ANGLE_DEG);
        return;
    }
    ServoDriver_SetAngle(s_angle_deg - SERVO_DRIVER_SWEEP_STEP_DEG);
}
