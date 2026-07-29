#include "k230_uart.h"
#include "main.h"

#define K230_UART_RX_BUFFER_SIZE 256U
#define K230_UART_RX_BUFFER_MASK (K230_UART_RX_BUFFER_SIZE - 1U)

static volatile uint8_t s_rx_buffer[K230_UART_RX_BUFFER_SIZE];
static volatile uint16_t s_rx_head;
static volatile uint16_t s_rx_tail;
static uint8_t s_rx_byte;
static volatile uint32_t s_rx_byte_count;
static volatile uint32_t s_overflow_count;
static volatile uint32_t s_error_count;

void K230Uart_Init(void)
{
    s_rx_head = 0U;
    s_rx_tail = 0U;
    s_rx_byte_count = 0U;
    s_overflow_count = 0U;
    s_error_count = 0U;
    if (HAL_UART_Receive_IT(&huart4, &s_rx_byte, 1U) != HAL_OK) {
        Error_Handler();
    }
}

uint8_t K230Uart_ReadByte(uint8_t *byte)
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
    s_rx_tail = (uint16_t) ((tail + 1U) & K230_UART_RX_BUFFER_MASK);
    return 1U;
}

uint32_t K230Uart_GetRxByteCount(void)
{
    return s_rx_byte_count;
}

uint32_t K230Uart_GetOverflowCount(void)
{
    return s_overflow_count;
}

uint32_t K230Uart_GetErrorCount(void)
{
    return s_error_count;
}

void K230Uart_HandleRxComplete(UART_HandleTypeDef *huart)
{
    uint16_t next_head;

    if (huart->Instance != K230_UART_INSTANCE) {
        return;
    }

    next_head = (uint16_t) ((s_rx_head + 1U) & K230_UART_RX_BUFFER_MASK);
    if (next_head == s_rx_tail) {
        s_overflow_count++;
    } else {
        s_rx_buffer[s_rx_head] = s_rx_byte;
        s_rx_head = next_head;
        s_rx_byte_count++;
    }

    if (HAL_UART_Receive_IT(&huart4, &s_rx_byte, 1U) != HAL_OK) {
        s_error_count++;
    }
}

void K230Uart_HandleError(UART_HandleTypeDef *huart)
{
    if (huart->Instance != K230_UART_INSTANCE) {
        return;
    }

    s_error_count++;
    __HAL_UART_CLEAR_OREFLAG(huart);
    if (HAL_UART_Receive_IT(&huart4, &s_rx_byte, 1U) != HAL_OK) {
        s_error_count++;
    }
}
