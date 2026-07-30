#include "main.h"

void HAL_MspInit(void)
{
    __HAL_RCC_SYSCFG_CLK_ENABLE();
    HAL_NVIC_SetPriorityGrouping(NVIC_PRIORITYGROUP_4);
}

void HAL_UART_MspInit(UART_HandleTypeDef *huart)
{
    GPIO_InitTypeDef gpio = {0};
    RCC_PeriphCLKInitTypeDef clock = {0};

    if (huart->Instance == USART2) {
        __HAL_RCC_GPIOD_CLK_ENABLE();
        __HAL_RCC_USART2_CLK_ENABLE();
        clock.PeriphClockSelection = RCC_PERIPHCLK_USART2;
        clock.Usart234578ClockSelection = RCC_USART2CLKSOURCE_HSI;
        if (HAL_RCCEx_PeriphCLKConfig(&clock) != HAL_OK) {
            Error_Handler();
        }

        gpio.Pin = GPIO_PIN_5 | GPIO_PIN_6;
        gpio.Mode = GPIO_MODE_AF_PP;
        gpio.Pull = GPIO_PULLUP;
        gpio.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
        gpio.Alternate = GPIO_AF7_USART2;
        HAL_GPIO_Init(GPIOD, &gpio);
        HAL_NVIC_SetPriority(USART2_IRQn, 5U, 0U);
        HAL_NVIC_EnableIRQ(USART2_IRQn);
    } else if (huart->Instance == UART4) {
        __HAL_RCC_GPIOB_CLK_ENABLE();
        __HAL_RCC_UART4_CLK_ENABLE();
        clock.PeriphClockSelection = RCC_PERIPHCLK_UART4;
        clock.Usart234578ClockSelection = RCC_UART4CLKSOURCE_HSI;
        if (HAL_RCCEx_PeriphCLKConfig(&clock) != HAL_OK) {
            Error_Handler();
        }

        gpio.Pin = GPIO_PIN_8 | GPIO_PIN_9;
        gpio.Mode = GPIO_MODE_AF_PP;
        gpio.Pull = GPIO_PULLUP;
        gpio.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
        gpio.Alternate = GPIO_AF8_UART4;
        HAL_GPIO_Init(GPIOB, &gpio);
        HAL_NVIC_SetPriority(UART4_IRQn, 5U, 0U);
        HAL_NVIC_EnableIRQ(UART4_IRQn);
    }
}

void HAL_I2C_MspInit(I2C_HandleTypeDef *hi2c)
{
    GPIO_InitTypeDef gpio = {0};
    RCC_PeriphCLKInitTypeDef clock = {0};

    if (hi2c->Instance == OLED_I2C_INSTANCE) {
        __HAL_RCC_GPIOF_CLK_ENABLE();
        __HAL_RCC_I2C4_CLK_ENABLE();

        clock.PeriphClockSelection = RCC_PERIPHCLK_I2C4;
        clock.I2c4ClockSelection = RCC_I2C4CLKSOURCE_HSI;
        if (HAL_RCCEx_PeriphCLKConfig(&clock) != HAL_OK) {
            Error_Handler();
        }

        gpio.Pin = OLED_I2C_SCL_PIN | OLED_I2C_SDA_PIN;
        gpio.Mode = GPIO_MODE_AF_OD;
        gpio.Pull = GPIO_PULLUP;
        gpio.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
        gpio.Alternate = GPIO_AF4_I2C4;
        HAL_GPIO_Init(GPIOF, &gpio);
    } else if (hi2c->Instance == IMU_I2C_INSTANCE) {
        __HAL_RCC_GPIOB_CLK_ENABLE();
        __HAL_RCC_I2C2_CLK_ENABLE();

        clock.PeriphClockSelection = RCC_PERIPHCLK_I2C2;
        clock.I2c123ClockSelection = RCC_I2C2CLKSOURCE_HSI;
        if (HAL_RCCEx_PeriphCLKConfig(&clock) != HAL_OK) {
            Error_Handler();
        }

        gpio.Pin = IMU_I2C_SCL_PIN | IMU_I2C_SDA_PIN;
        gpio.Mode = GPIO_MODE_AF_OD;
        gpio.Pull = GPIO_PULLUP;
        gpio.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
        gpio.Alternate = GPIO_AF4_I2C2;
        HAL_GPIO_Init(GPIOB, &gpio);
    }
}

void HAL_TIM_PWM_MspInit(TIM_HandleTypeDef *htim)
{
    GPIO_InitTypeDef gpio = {0};

    if (htim->Instance == TIM2) {
        __HAL_RCC_TIM2_CLK_ENABLE();
        __HAL_RCC_GPIOA_CLK_ENABLE();
        gpio.Pin = GPIO_PIN_0 | GPIO_PIN_2;
        gpio.Mode = GPIO_MODE_AF_PP;
        gpio.Pull = GPIO_NOPULL;
        gpio.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
        gpio.Alternate = GPIO_AF1_TIM2;
        HAL_GPIO_Init(GPIOA, &gpio);
    } else if (htim->Instance == SERVO_PWM_INSTANCE) {
        /* PD15 复用为 TIM4_CH4，输出 SG90 所需的 50 Hz PWM。 */
        __HAL_RCC_TIM4_CLK_ENABLE();
        __HAL_RCC_GPIOD_CLK_ENABLE();
        gpio.Pin = SERVO_PWM_PIN;
        gpio.Mode = GPIO_MODE_AF_PP;
        gpio.Pull = GPIO_NOPULL;
        gpio.Speed = GPIO_SPEED_FREQ_LOW;
        gpio.Alternate = GPIO_AF2_TIM4;
        HAL_GPIO_Init(SERVO_PWM_PORT, &gpio);
    }
}

void HAL_TIM_Encoder_MspInit(TIM_HandleTypeDef *htim)
{
    GPIO_InitTypeDef gpio = {0};

    gpio.Mode = GPIO_MODE_AF_PP;
    gpio.Pull = GPIO_PULLUP;
    gpio.Speed = GPIO_SPEED_FREQ_VERY_HIGH;

    if (htim->Instance == TIM1) {
        __HAL_RCC_TIM1_CLK_ENABLE();
        __HAL_RCC_GPIOA_CLK_ENABLE();
        __HAL_RCC_GPIOE_CLK_ENABLE();
        gpio.Pin = GPIO_PIN_8;
        gpio.Alternate = GPIO_AF1_TIM1;
        HAL_GPIO_Init(GPIOA, &gpio);
        gpio.Pin = GPIO_PIN_11;
        HAL_GPIO_Init(GPIOE, &gpio);
    } else if (htim->Instance == TIM8) {
        __HAL_RCC_TIM8_CLK_ENABLE();
        __HAL_RCC_GPIOC_CLK_ENABLE();
        gpio.Pin = GPIO_PIN_6 | GPIO_PIN_7;
        gpio.Alternate = GPIO_AF3_TIM8;
        HAL_GPIO_Init(GPIOC, &gpio);
    }
}

void HAL_UART_MspDeInit(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART2) {
        __HAL_RCC_USART2_FORCE_RESET();
        __HAL_RCC_USART2_RELEASE_RESET();
        HAL_GPIO_DeInit(GPIOD, GPIO_PIN_5 | GPIO_PIN_6);
        HAL_NVIC_DisableIRQ(USART2_IRQn);
    } else if (huart->Instance == UART4) {
        __HAL_RCC_UART4_FORCE_RESET();
        __HAL_RCC_UART4_RELEASE_RESET();
        HAL_GPIO_DeInit(GPIOB, GPIO_PIN_8 | GPIO_PIN_9);
        HAL_NVIC_DisableIRQ(UART4_IRQn);
    }
}

void HAL_I2C_MspDeInit(I2C_HandleTypeDef *hi2c)
{
    if (hi2c->Instance == OLED_I2C_INSTANCE) {
        __HAL_RCC_I2C4_FORCE_RESET();
        __HAL_RCC_I2C4_RELEASE_RESET();
        HAL_GPIO_DeInit(GPIOF, OLED_I2C_SCL_PIN | OLED_I2C_SDA_PIN);
    } else if (hi2c->Instance == IMU_I2C_INSTANCE) {
        __HAL_RCC_I2C2_FORCE_RESET();
        __HAL_RCC_I2C2_RELEASE_RESET();
        HAL_GPIO_DeInit(GPIOB, IMU_I2C_SCL_PIN | IMU_I2C_SDA_PIN);
    }
}

void HAL_TIM_PWM_MspDeInit(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM2) {
        __HAL_RCC_TIM2_FORCE_RESET();
        __HAL_RCC_TIM2_RELEASE_RESET();
        HAL_GPIO_DeInit(GPIOA, GPIO_PIN_0 | GPIO_PIN_2);
    } else if (htim->Instance == SERVO_PWM_INSTANCE) {
        __HAL_RCC_TIM4_FORCE_RESET();
        __HAL_RCC_TIM4_RELEASE_RESET();
        HAL_GPIO_DeInit(SERVO_PWM_PORT, SERVO_PWM_PIN);
    }
}

void HAL_TIM_Encoder_MspDeInit(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM1) {
        __HAL_RCC_TIM1_FORCE_RESET();
        __HAL_RCC_TIM1_RELEASE_RESET();
        HAL_GPIO_DeInit(GPIOA, GPIO_PIN_8);
        HAL_GPIO_DeInit(GPIOE, GPIO_PIN_11);
    } else if (htim->Instance == TIM8) {
        __HAL_RCC_TIM8_FORCE_RESET();
        __HAL_RCC_TIM8_RELEASE_RESET();
        HAL_GPIO_DeInit(GPIOC, GPIO_PIN_6 | GPIO_PIN_7);
    }
}
