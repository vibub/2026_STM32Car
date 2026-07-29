#ifndef STM32CAR_MAIN_H
#define STM32CAR_MAIN_H

#include "stm32h7xx_hal.h"

#ifdef __cplusplus
extern "C" {
#endif

/* 电机：右轮 TIM2_CH1/PA0 + PA1，左轮 TIM2_CH3/PA2 + PA3。 */
#define MOTOR_RIGHT_DIR_PORT GPIOA
#define MOTOR_RIGHT_DIR_PIN  GPIO_PIN_1
#define MOTOR_RIGHT_CHANNEL  TIM_CHANNEL_1
#define MOTOR_LEFT_DIR_PORT  GPIOA
#define MOTOR_LEFT_DIR_PIN   GPIO_PIN_3
#define MOTOR_LEFT_CHANNEL   TIM_CHANNEL_3

/* K230：UART4，PB9 为 STM32 TX，PB8 为 STM32 RX。 */
#define K230_UART_INSTANCE UART4
#define K230_UART_IRQn     UART4_IRQn

extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim8;
extern UART_HandleTypeDef huart4;

void SystemClock_Config(void);
void Error_Handler(void);

#ifdef __cplusplus
}
#endif

#endif /* STM32CAR_MAIN_H */
