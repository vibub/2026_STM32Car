#include "host_link_uart.h"
#include "main.h"

#define HOST_LINK_RX_BUFFER_SIZE 256U                             /* USART1 接收环形缓冲区容量。 */
#define HOST_LINK_RX_BUFFER_MASK (HOST_LINK_RX_BUFFER_SIZE - 1U)  /* 环形缓冲区下标回绕掩码。 */

static volatile uint8_t s_rx_buffer[HOST_LINK_RX_BUFFER_SIZE];
static volatile uint16_t s_rx_head;
static volatile uint16_t s_rx_tail;
static uint8_t s_rx_byte;
static volatile uint32_t s_rx_byte_count;
static volatile uint32_t s_overflow_count;
static volatile uint32_t s_error_count;

void HostLinkUart_Init(void)
{
    s_rx_head = 0U;
    s_rx_tail = 0U;
    s_rx_byte_count = 0U;
    s_overflow_count = 0U;
    s_error_count = 0U;
    if (HAL_UART_Receive_IT(&huart1, &s_rx_byte, 1U) != HAL_OK) {
        Error_Handler();
    }
}

void HostLinkUart_SendData(const uint8_t *data, uint16_t length)
{
    if ((data == 0) || (length == 0U)) {
        return;
    }
    (void) HAL_UART_Transmit(&huart1, (uint8_t *) data, length, 20U);
}

uint8_t HostLinkUart_ReadByte(uint8_t *byte)
{
    uint16_t tail;
    uint16_t head;

    if (byte == 0) {
        return 0U;
    }
    tail = s_rx_tail;
    head = s_rx_head;
    if (tail == head) {
        return 0U;
    }

    *byte = s_rx_buffer[tail];
    s_rx_tail = (uint16_t) ((tail + 1U) & HOST_LINK_RX_BUFFER_MASK);
    return 1U;
}

void HostLinkUart_FlushRx(void)
{
    s_rx_tail = s_rx_head;
}

uint32_t HostLinkUart_GetRxByteCount(void)
{
    return s_rx_byte_count;
}

uint32_t HostLinkUart_GetOverflowCount(void)
{
    return s_overflow_count;
}

uint32_t HostLinkUart_GetErrorCount(void)
{
    return s_error_count;
}

void HostLinkUart_HandleRxComplete(UART_HandleTypeDef *huart)
{
    uint16_t next_head;

    if (huart->Instance != HOST_DEBUG_UART_INSTANCE) {
        return;
    }

    next_head = (uint16_t) ((s_rx_head + 1U) & HOST_LINK_RX_BUFFER_MASK);
    if (next_head == s_rx_tail) {
        s_overflow_count++;
    } else {
        s_rx_buffer[s_rx_head] = s_rx_byte;
        s_rx_head = next_head;
        s_rx_byte_count++;
    }

    if (HAL_UART_Receive_IT(&huart1, &s_rx_byte, 1U) != HAL_OK) {
        s_error_count++;
    }
}

void HostLinkUart_HandleError(UART_HandleTypeDef *huart)
{
    if (huart->Instance != HOST_DEBUG_UART_INSTANCE) {
        return;
    }

    s_error_count++;
    __HAL_UART_CLEAR_OREFLAG(huart);
    if (HAL_UART_Receive_IT(&huart1, &s_rx_byte, 1U) != HAL_OK) {
        s_error_count++;
    }
}
