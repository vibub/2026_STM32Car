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

/* TJC3224T124 串口屏：USART2，PD5 为 STM32 TX，PD6 为 STM32 RX。 */
#define TJC_UART_INSTANCE USART2
#define TJC_UART_IRQn     USART2_IRQn

/* K230 通讯接口：UART4，PB9 为 STM32 TX，PB8 为 STM32 RX。 */
#define K230_UART_INSTANCE UART4
#define K230_UART_IRQn     UART4_IRQn

/* OLED 与八路红外模块共用 I2C4：SCL PF14，SDA PF15。 */
#define OLED_I2C_INSTANCE I2C4
#define OLED_I2C_SCL_PORT GPIOF
#define OLED_I2C_SCL_PIN  GPIO_PIN_14
#define OLED_I2C_SDA_PORT GPIOF
#define OLED_I2C_SDA_PIN  GPIO_PIN_15
#define SCL_PF14_PORT     OLED_I2C_SCL_PORT
#define SCL_PF14_PIN      OLED_I2C_SCL_PIN
#define SDA_PF15_PORT     OLED_I2C_SDA_PORT
#define SDA_PF15_PIN      OLED_I2C_SDA_PIN

extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim8;
extern I2C_HandleTypeDef hi2c4;
extern UART_HandleTypeDef huart2;
extern UART_HandleTypeDef huart4;

/* 发生 HAL 初始化错误时，记录触发错误的源文件和行号供 IAR Watch 查看。 */
extern const char * volatile g_error_file;
extern volatile uint32_t g_error_line;

void SystemClock_Config(void);
void Error_HandlerAt(const char *file, uint32_t line);
#define Error_Handler() Error_HandlerAt(__FILE__, (uint32_t) __LINE__)

#ifdef __cplusplus
}
#endif

#endif /* STM32CAR_MAIN_H */
