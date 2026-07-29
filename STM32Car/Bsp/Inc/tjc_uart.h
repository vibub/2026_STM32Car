#ifndef TJC_UART_H
#define TJC_UART_H
#include "stm32h7xx_hal.h"
#include <stdint.h>
void TjcUart_Init(void);
void TjcUart_SendByte(uint8_t byte);
void TjcUart_SendData(const uint8_t *data, uint16_t length);
uint8_t TjcUart_ReadByte(uint8_t *byte);
void TjcUart_FlushRx(void);
uint32_t TjcUart_GetRxByteCount(void);
uint32_t TjcUart_GetOverflowCount(void);
uint32_t TjcUart_GetErrorCount(void);
void TjcUart_HandleRxComplete(UART_HandleTypeDef *huart);
void TjcUart_HandleError(UART_HandleTypeDef *huart);
#endif /* TJC_UART_H */