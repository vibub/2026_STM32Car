#ifndef K230_UART_H
#define K230_UART_H

#include "stm32h7xx_hal.h"
#include <stdint.h>

/* Live Watch diagnostics: these change as soon as UART4 receives any byte. */
extern volatile uint32_t g_k230_uart_rx_irq_count;
extern volatile uint8_t g_k230_uart_last_rx_byte;
extern volatile uint32_t g_k230_uart_last_rx_tick_ms;
extern volatile uint32_t g_k230_uart_error_irq_count;

void K230Uart_Init(void);
uint8_t K230Uart_ReadByte(uint8_t *byte);
uint32_t K230Uart_GetRxByteCount(void);
uint32_t K230Uart_GetOverflowCount(void);
uint32_t K230Uart_GetErrorCount(void);
void K230Uart_HandleRxComplete(UART_HandleTypeDef *huart);
void K230Uart_HandleError(UART_HandleTypeDef *huart);

#endif /* K230_UART_H */
