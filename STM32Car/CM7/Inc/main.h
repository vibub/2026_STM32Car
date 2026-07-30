#ifndef STM32CAR_MAIN_H
#define STM32CAR_MAIN_H

#include "stm32h7xx_hal.h"

#ifdef __cplusplus
extern "C" {
#endif

#define MOTOR_RIGHT_DIR_PORT GPIOA         /* 右电机方向控制 GPIO 端口。 */
#define MOTOR_RIGHT_DIR_PIN  GPIO_PIN_1    /* 右电机方向控制引脚 PA1。 */
#define MOTOR_RIGHT_CHANNEL  TIM_CHANNEL_1 /* 右电机 PWM 使用 TIM2 通道 1。 */
#define MOTOR_LEFT_DIR_PORT  GPIOA         /* 左电机方向控制 GPIO 端口。 */
#define MOTOR_LEFT_DIR_PIN   GPIO_PIN_3    /* 左电机方向控制引脚 PA3。 */
#define MOTOR_LEFT_CHANNEL   TIM_CHANNEL_3 /* 左电机 PWM 使用 TIM2 通道 3。 */

/* SG90 舵机 PWM：TIM4_CH4，PD15，输出频率 50 Hz。 */
#define SERVO_PWM_INSTANCE TIM4        /* 舵机 PWM 使用的定时器实例。 */
#define SERVO_PWM_PORT     GPIOD       /* 舵机 PWM 输出 GPIO 端口。 */
#define SERVO_PWM_PIN      GPIO_PIN_15 /* 舵机 PWM 输出引脚 PD15。 */
#define SERVO_PWM_CHANNEL  TIM_CHANNEL_4 /* 舵机 PWM 使用 TIM4 通道 4。 */

/* TJC3224T124 串口屏：USART2，PD5 为 STM32 TX，PD6 为 STM32 RX。 */
#define TJC_UART_INSTANCE USART2     /* TJC 串口使用的外设实例。 */
#define TJC_UART_IRQn     USART2_IRQn /* TJC 串口对应的中断号。 */

/* K230 通讯接口：UART4，PB9 为 STM32 TX，PB8 为 STM32 RX。 */
#define K230_UART_INSTANCE UART4     /* K230 串口使用的外设实例。 */
#define K230_UART_IRQn     UART4_IRQn /* K230 串口对应的中断号。 */

/* OLED 与八路红外模块共用 I2C4：SCL PF14，SDA PF15。 */
#define OLED_I2C_INSTANCE I2C4          /* OLED 和红外模块共用的 I2C 外设。 */
#define OLED_I2C_SCL_PORT GPIOF         /* I2C4 时钟线 GPIO 端口。 */
#define OLED_I2C_SCL_PIN  GPIO_PIN_14   /* I2C4 时钟线引脚 PF14。 */
#define OLED_I2C_SDA_PORT GPIOF         /* I2C4 数据线 GPIO 端口。 */
#define OLED_I2C_SDA_PIN  GPIO_PIN_15   /* I2C4 数据线引脚 PF15。 */
#define SCL_PF14_PORT     OLED_I2C_SCL_PORT /* PF14 时钟线的兼容端口别名。 */
#define SCL_PF14_PIN      OLED_I2C_SCL_PIN  /* PF14 时钟线的兼容引脚别名。 */
#define SDA_PF15_PORT     OLED_I2C_SDA_PORT /* PF15 数据线的兼容端口别名。 */
#define SDA_PF15_PIN      OLED_I2C_SDA_PIN  /* PF15 数据线的兼容引脚别名。 */

extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim4;
extern TIM_HandleTypeDef htim8;
extern I2C_HandleTypeDef hi2c4;
extern UART_HandleTypeDef huart2;
extern UART_HandleTypeDef huart4;

/* 发生 HAL 初始化错误时，记录触发错误的源文件和行号供 IAR Watch 查看。 */
extern const char * volatile g_error_file;
extern volatile uint32_t g_error_line;

void SystemClock_Config(void);
void Error_HandlerAt(const char *file, uint32_t line);
#define Error_Handler() Error_HandlerAt(__FILE__, (uint32_t) __LINE__) /* 自动记录 HAL 错误发生的文件和行号。 */

#ifdef __cplusplus
}
#endif

#endif /* STM32CAR_MAIN_H */
