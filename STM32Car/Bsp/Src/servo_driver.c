#include "servo_driver.h"
#include "main.h"

/* TIM4以1 MHz计数；舵机完整电气规格仍为500～2500 us对应0～270°。 */
#define SERVO_DRIVER_ELECTRICAL_MIN_PULSE_US   500.0f /* 电气角0°对应脉宽。 */
#define SERVO_DRIVER_ELECTRICAL_PULSE_SPAN_US 2000.0f /* 500～2500 us完整电气跨度。 */
#define SERVO_DRIVER_ELECTRICAL_SPAN_DEG        270.0f /* 舵机完整电气角度跨度。 */

static float s_angle_deg = 0.0f;
static uint32_t s_pulse_us = SERVO_DRIVER_CENTER_PULSE_US;

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

static uint32_t ServoDriver_LimitPulseUs(uint32_t pulse_us)
{
    if (pulse_us < SERVO_DRIVER_MIN_PULSE_US) {
        return SERVO_DRIVER_MIN_PULSE_US;
    }
    if (pulse_us > SERVO_DRIVER_MAX_PULSE_US) {
        return SERVO_DRIVER_MAX_PULSE_US;
    }
    return pulse_us;
}

static uint32_t ServoDriver_AngleToPulseUs(float angle_deg)
{
    float pulse_us;

    angle_deg = ServoDriver_LimitAngle(angle_deg);
    pulse_us = SERVO_DRIVER_ELECTRICAL_MIN_PULSE_US +
               angle_deg / SERVO_DRIVER_ELECTRICAL_SPAN_DEG *
               SERVO_DRIVER_ELECTRICAL_PULSE_SPAN_US;
    return ServoDriver_LimitPulseUs((uint32_t) (pulse_us + 0.5f));
}

static float ServoDriver_PulseUsToAngle(uint32_t pulse_us)
{
    pulse_us = ServoDriver_LimitPulseUs(pulse_us);
    return ((float) pulse_us - SERVO_DRIVER_ELECTRICAL_MIN_PULSE_US) /
           SERVO_DRIVER_ELECTRICAL_PULSE_SPAN_US *
           SERVO_DRIVER_ELECTRICAL_SPAN_DEG;
}

void ServoDriver_Init(float initial_angle_deg)
{
    ServoDriver_InitPulseUs(ServoDriver_AngleToPulseUs(initial_angle_deg));
}

void ServoDriver_InitPulseUs(uint32_t initial_pulse_us)
{
    ServoDriver_SetPulseUs(initial_pulse_us);
    if (HAL_TIM_PWM_Start(&htim4, SERVO_PWM_CHANNEL) != HAL_OK) {
        Error_Handler();
    }
    ServoDriver_SetPulseUs(initial_pulse_us);
}

void ServoDriver_SetAngle(float angle_deg)
{
    ServoDriver_SetPulseUs(ServoDriver_AngleToPulseUs(angle_deg));
}

void ServoDriver_SetPulseUs(uint32_t pulse_us)
{
    s_pulse_us = ServoDriver_LimitPulseUs(pulse_us);
    s_angle_deg = ServoDriver_PulseUsToAngle(s_pulse_us);
    __HAL_TIM_SET_COMPARE(&htim4, SERVO_PWM_CHANNEL, s_pulse_us);
}

void ServoDriver_SetPipePitchDeg(float pipe_pitch_deg)
{
    ServoDriver_SetPulseUs(ServoDriver_PipePitchToPulseUs(pipe_pitch_deg));
}

float ServoDriver_GetAngle(void)
{
    return s_angle_deg;
}

uint32_t ServoDriver_GetPulseUs(void)
{
    return s_pulse_us;
}

float ServoDriver_GetPipePitchDeg(void)
{
    return ServoDriver_PulseUsToPipePitchDeg(s_pulse_us);
}

uint32_t ServoDriver_PipePitchToPulseUs(float pipe_pitch_deg)
{
    float pulse_us;

    if (pipe_pitch_deg < SERVO_DRIVER_MIN_PIPE_PITCH_DEG) {
        pipe_pitch_deg = SERVO_DRIVER_MIN_PIPE_PITCH_DEG;
    }
    if (pipe_pitch_deg > SERVO_DRIVER_MAX_PIPE_PITCH_DEG) {
        pipe_pitch_deg = SERVO_DRIVER_MAX_PIPE_PITCH_DEG;
    }
    pulse_us = (float) SERVO_DRIVER_CENTER_PULSE_US +
               pipe_pitch_deg / SERVO_DRIVER_PIPE_DEG_PER_US;
    return ServoDriver_LimitPulseUs((uint32_t) (pulse_us + 0.5f));
}

float ServoDriver_PulseUsToPipePitchDeg(uint32_t pulse_us)
{
    pulse_us = ServoDriver_LimitPulseUs(pulse_us);
    return SERVO_DRIVER_PIPE_DEG_PER_US *
           ((float) pulse_us - (float) SERVO_DRIVER_CENTER_PULSE_US);
}
