#ifndef STM32CAR_MAIN_H
#define STM32CAR_MAIN_H

#include "stm32h7xx_hal.h"

#ifdef __cplusplus
extern "C" {
#endif

#define MOTOR_RIGHT_DIR_PORT GPIOA
#define MOTOR_RIGHT_DIR_PIN  GPIO_PIN_1
#define MOTOR_RIGHT_CHANNEL  TIM_CHANNEL_1
#define MOTOR_LEFT_DIR_PORT  GPIOA
#define MOTOR_LEFT_DIR_PIN   GPIO_PIN_3
#define MOTOR_LEFT_CHANNEL   TIM_CHANNEL_3

/* TJC3224T124 touch panel: USART2, PD5 is STM32 TX, PD6 is STM32 RX. */
#define TJC_UART_INSTANCE USART2
#define TJC_UART_IRQn     USART2_IRQn

extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim8;
extern UART_HandleTypeDef huart2;

void SystemClock_Config(void);
void Error_Handler(void);

#ifdef __cplusplus
}
#endif

#endif /* STM32CAR_MAIN_H */
