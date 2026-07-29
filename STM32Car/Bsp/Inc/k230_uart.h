#ifndef K230_UART_H
#define K230_UART_H

#include "stm32h7xx_hal.h"
#include <stdint.h>

void K230Uart_Init(void);
uint8_t K230Uart_ReadByte(uint8_t *byte);
uint32_t K230Uart_GetRxByteCount(void);
uint32_t K230Uart_GetOverflowCount(void);
uint32_t K230Uart_GetErrorCount(void);
void K230Uart_HandleRxComplete(UART_HandleTypeDef *huart);
void K230Uart_HandleError(UART_HandleTypeDef *huart);

#endif /* K230_UART_H */
