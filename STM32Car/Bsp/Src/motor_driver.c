#include "motor_driver.h"
#include "main.h"

#define MOTOR_DRIVER_MAX_RATIO 0.50f /* 单侧电机允许输出的最大 PWM 比例。 */

static float s_left_ratio;
static float s_right_ratio;
static uint8_t s_pwm_outputs_enabled;

static void MotorDriver_LockPwmOutputs(void)
{
    GPIO_InitTypeDef gpio = {0};

    __HAL_TIM_SET_COMPARE(&htim2, MOTOR_LEFT_CHANNEL, 0U);
    __HAL_TIM_SET_COMPARE(&htim2, MOTOR_RIGHT_CHANNEL, 0U);
    (void) HAL_TIM_PWM_Stop(&htim2, MOTOR_RIGHT_CHANNEL);
    (void) HAL_TIM_PWM_Stop(&htim2, MOTOR_LEFT_CHANNEL);

    HAL_GPIO_WritePin(MOTOR_RIGHT_PWM_PORT, MOTOR_RIGHT_PWM_PIN,
                      GPIO_PIN_RESET);
    HAL_GPIO_WritePin(MOTOR_LEFT_PWM_PORT, MOTOR_LEFT_PWM_PIN,
                      GPIO_PIN_RESET);
    gpio.Mode = GPIO_MODE_OUTPUT_PP;
    gpio.Pull = GPIO_NOPULL;
    gpio.Speed = GPIO_SPEED_FREQ_LOW;
    gpio.Pin = MOTOR_RIGHT_PWM_PIN;
    HAL_GPIO_Init(MOTOR_RIGHT_PWM_PORT, &gpio);
    gpio.Pin = MOTOR_LEFT_PWM_PIN;
    HAL_GPIO_Init(MOTOR_LEFT_PWM_PORT, &gpio);
    s_pwm_outputs_enabled = 0U;
}

static void MotorDriver_EnablePwmOutputs(void)
{
    GPIO_InitTypeDef gpio = {0};

    if (s_pwm_outputs_enabled != 0U) {
        return;
    }

    gpio.Mode = GPIO_MODE_AF_PP;
    gpio.Pull = GPIO_NOPULL;
    gpio.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    gpio.Alternate = GPIO_AF1_TIM2;
    gpio.Pin = MOTOR_RIGHT_PWM_PIN;
    HAL_GPIO_Init(MOTOR_RIGHT_PWM_PORT, &gpio);
    gpio.Pin = MOTOR_LEFT_PWM_PIN;
    HAL_GPIO_Init(MOTOR_LEFT_PWM_PORT, &gpio);
    if (HAL_TIM_PWM_Start(&htim2, MOTOR_RIGHT_CHANNEL) != HAL_OK) {
        Error_Handler();
    }
    if (HAL_TIM_PWM_Start(&htim2, MOTOR_LEFT_CHANNEL) != HAL_OK) {
        Error_Handler();
    }
    s_pwm_outputs_enabled = 1U;
}

static float MotorDriver_Limit(float ratio)
{
    if (ratio > MOTOR_DRIVER_MAX_RATIO) {
        return MOTOR_DRIVER_MAX_RATIO;
    }
    if (ratio < -MOTOR_DRIVER_MAX_RATIO) {
        return -MOTOR_DRIVER_MAX_RATIO;
    }
    return ratio;
}

/*
 * 参考板使用双输入 H 桥：正反转对应不同的 PWM/方向组合。
 * 这里严格保留参考工程的电气表达，不把 compare 简化成普通 PWM+DIR。
 */
static void MotorDriver_SetOne(float ratio, uint32_t channel,
                               GPIO_TypeDef *dir_port, uint16_t dir_pin)
{
    uint32_t period = __HAL_TIM_GET_AUTORELOAD(&htim2);
    uint32_t compare;

    ratio = MotorDriver_Limit(ratio);
    if (ratio > 0.0f) {
        HAL_GPIO_WritePin(dir_port, dir_pin, GPIO_PIN_SET);
        compare = (uint32_t) ((1.0f - ratio) * (float) period);
    } else if (ratio < 0.0f) {
        HAL_GPIO_WritePin(dir_port, dir_pin, GPIO_PIN_RESET);
        compare = (uint32_t) ((-ratio) * (float) period);
    } else {
        HAL_GPIO_WritePin(dir_port, dir_pin, GPIO_PIN_RESET);
        compare = 0U;
    }

    __HAL_TIM_SET_COMPARE(&htim2, channel, compare);
}

void MotorDriver_Init(void)
{
    s_left_ratio = 0.0f;
    s_right_ratio = 0.0f;
    HAL_GPIO_WritePin(MOTOR_LEFT_DIR_PORT, MOTOR_LEFT_DIR_PIN, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(MOTOR_RIGHT_DIR_PORT, MOTOR_RIGHT_DIR_PIN, GPIO_PIN_RESET);
    /*
     * TIM2 is configured by MX_TIM2_Init() before this function runs.  Do not
     * merely leave its compare registers at zero: actively detach PA0/PA2
     * from TIM2 and drive both PWM pins low until a motion command arrives.
     */
    MotorDriver_LockPwmOutputs();
}

void MotorDriver_Set(float left_ratio, float right_ratio)
{
    s_left_ratio = MotorDriver_Limit(left_ratio);
    s_right_ratio = MotorDriver_Limit(right_ratio);
    if ((s_left_ratio == 0.0f) && (s_right_ratio == 0.0f)) {
        MotorDriver_Stop();
        return;
    }
    MotorDriver_EnablePwmOutputs();
    MotorDriver_SetOne(s_left_ratio, MOTOR_LEFT_CHANNEL,
                       MOTOR_LEFT_DIR_PORT, MOTOR_LEFT_DIR_PIN);
    MotorDriver_SetOne(s_right_ratio, MOTOR_RIGHT_CHANNEL,
                       MOTOR_RIGHT_DIR_PORT, MOTOR_RIGHT_DIR_PIN);
}

void MotorDriver_Stop(void)
{
    s_left_ratio = 0.0f;
    s_right_ratio = 0.0f;
    __HAL_TIM_SET_COMPARE(&htim2, MOTOR_LEFT_CHANNEL, 0U);
    __HAL_TIM_SET_COMPARE(&htim2, MOTOR_RIGHT_CHANNEL, 0U);
    HAL_GPIO_WritePin(MOTOR_LEFT_DIR_PORT, MOTOR_LEFT_DIR_PIN, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(MOTOR_RIGHT_DIR_PORT, MOTOR_RIGHT_DIR_PIN, GPIO_PIN_RESET);
    if (s_pwm_outputs_enabled != 0U) {
        MotorDriver_LockPwmOutputs();
    }
}

float MotorDriver_GetLeftRatio(void)
{
    return s_left_ratio;
}

float MotorDriver_GetRightRatio(void)
{
    return s_right_ratio;
}
