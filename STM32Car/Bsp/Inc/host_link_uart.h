#ifndef HOST_LINK_UART_H
#define HOST_LINK_UART_H

#include "stm32h7xx_hal.h"
#include <stdint.h>

/* 上位机标定串口（USART1 / PB6-PB7 / 115200 8N1）接收与发送底层。 */
void HostLinkUart_Init(void);
void HostLinkUart_SendData(const uint8_t *data, uint16_t length);
uint8_t HostLinkUart_ReadByte(uint8_t *byte);
void HostLinkUart_FlushRx(void);
uint32_t HostLinkUart_GetRxByteCount(void);
uint32_t HostLinkUart_GetOverflowCount(void);
uint32_t HostLinkUart_GetErrorCount(void);
void HostLinkUart_HandleRxComplete(UART_HandleTypeDef *huart);
void HostLinkUart_HandleError(UART_HandleTypeDef *huart);

#endif /* HOST_LINK_UART_H */
