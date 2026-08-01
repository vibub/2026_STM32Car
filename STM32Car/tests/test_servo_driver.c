#include "servo_driver.h"
#include "main.h"

#include <assert.h>
#include <math.h>
#include <stdio.h>

#define TEST_FLOAT_EPSILON 0.0001f /* 舵机角度换算测试允许的绝对误差。 */

TIM_HandleTypeDef htim4;

void Error_Handler(void)
{
    assert(0);
}

static void assert_float_near(float actual, float expected)
{
    assert(fabsf(actual - expected) <= TEST_FLOAT_EPSILON);
}

int main(void)
{
    ServoDriver_InitPulseUs(SERVO_DRIVER_CENTER_PULSE_US);
    assert(ServoDriver_GetCenterPulseUs() == SERVO_DRIVER_CENTER_PULSE_US);
    assert(ServoDriver_GetPulseUs() == SERVO_DRIVER_CENTER_PULSE_US);
    assert(htim4.compare == SERVO_DRIVER_CENTER_PULSE_US);
    assert_float_near(ServoDriver_GetPipePitchDeg(), 0.0f);
    assert(ServoDriver_PipePitchToPulseUs(0.0f) == 970U);
    assert(ServoDriver_PipePitchToPulseUs(
               SERVO_DRIVER_MIN_PIPE_PITCH_DEG) == 500U);
    assert(ServoDriver_PipePitchToPulseUs(
               SERVO_DRIVER_MAX_PIPE_PITCH_DEG) == 1600U);
    assert_float_near(
        ServoDriver_PulseUsToPipePitchDeg(500U), -4.7423f);
    assert_float_near(
        ServoDriver_PulseUsToPipePitchDeg(1600U), 6.3567f);

    /* 人工确认1150 us后，正式角度换算应立即改用新的零位基准。 */
    assert(ServoDriver_SetCenterPulseUs(1150U) != 0U);
    ServoDriver_SetPulseUs(1150U);
    assert(ServoDriver_GetCenterPulseUs() == 1150U);
    assert_float_near(ServoDriver_GetPipePitchDeg(), 0.0f);
    assert(ServoDriver_PipePitchToPulseUs(1.0f) == 1249U);
    assert_float_near(ServoDriver_PulseUsToPipePitchDeg(1249U),
                      0.99891f);

    /* 越过实测安全脉宽的零位必须被拒绝，并保留最近一次有效值。 */
    assert(ServoDriver_SetCenterPulseUs(499U) == 0U);
    assert(ServoDriver_SetCenterPulseUs(1601U) == 0U);
    assert(ServoDriver_GetCenterPulseUs() == 1150U);

    puts("舵机动态零位PWM主机测试通过");
    return 0;
}
