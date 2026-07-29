#ifndef K230_UART_H
#define K230_UART_H

#include <stdint.h>

void K230Uart_Init(void);
uint8_t K230Uart_ReadByte(uint8_t *byte);
uint32_t K230Uart_GetRxByteCount(void);
uint32_t K230Uart_GetOverflowCount(void);
uint32_t K230Uart_GetErrorCount(void);

#endif /* K230_UART_H */
