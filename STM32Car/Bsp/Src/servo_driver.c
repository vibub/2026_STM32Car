#include "servo_driver.h"
#include "main.h"

/* TIM4 以 1 MHz 计数，500~2500 us 脉宽映射至 270°位置舵机的 0~270°。 */
#define SERVO_DRIVER_MIN_PULSE_US       500U   /* 舵机 0°对应的最小控制脉宽。 */
#define SERVO_DRIVER_MAX_PULSE_US       2500U  /* 舵机 270°对应的最大控制脉宽。 */
#define SERVO_DRIVER_PHYSICAL_SPAN_DEG  270.0f /* 舵机完整电气角度范围。 */

static float s_angle_deg = SERVO_DRIVER_MIN_ANGLE_DEG;

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
    ratio = angle_deg / SERVO_DRIVER_PHYSICAL_SPAN_DEG;
    pulse_us = (float) SERVO_DRIVER_MIN_PULSE_US +
               ratio * (float) (SERVO_DRIVER_MAX_PULSE_US -
                                SERVO_DRIVER_MIN_PULSE_US);
    return (uint32_t) (pulse_us + 0.5f);
}

void ServoDriver_Init(float initial_angle_deg)
{
    ServoDriver_SetAngle(initial_angle_deg);
    if (HAL_TIM_PWM_Start(&htim4, SERVO_PWM_CHANNEL) != HAL_OK) {
        Error_Handler();
    }
    ServoDriver_SetAngle(initial_angle_deg);
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
