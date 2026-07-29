#include "tjc_uart.h"
#include "k230_uart.h"
#include "main.h"

#define TJC_UART_RX_BUFFER_SIZE 64U
#define TJC_UART_RX_BUFFER_MASK (TJC_UART_RX_BUFFER_SIZE - 1U)

static volatile uint8_t s_rx_buffer[TJC_UART_RX_BUFFER_SIZE];
static volatile uint16_t s_rx_head;
static volatile uint16_t s_rx_tail;
static uint8_t s_rx_byte;
static volatile uint32_t s_rx_byte_count;
static volatile uint32_t s_overflow_count;
static volatile uint32_t s_error_count;

void TjcUart_Init(void)
{
    s_rx_head = 0U;
    s_rx_tail = 0U;
    s_rx_byte_count = 0U;
    s_overflow_count = 0U;
    s_error_count = 0U;
    if (HAL_UART_Receive_IT(&huart2, &s_rx_byte, 1U) != HAL_OK) {
        Error_Handler();
    }
}

void TjcUart_SendByte(uint8_t byte)
{
    (void) HAL_UART_Transmit(&huart2, &byte, 1U, 10U);
}

void TjcUart_SendData(const uint8_t *data, uint16_t length)
{
    if ((data == 0) || (length == 0U)) {
        return;
    }
    (void) HAL_UART_Transmit(&huart2, (uint8_t *) data, length, 10U);
}

uint8_t TjcUart_ReadByte(uint8_t *byte)
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
    s_rx_tail = (uint16_t) ((tail + 1U) & TJC_UART_RX_BUFFER_MASK);
    return 1U;
}

void TjcUart_FlushRx(void)
{
    s_rx_tail = s_rx_head;
}

uint32_t TjcUart_GetRxByteCount(void)
{
    return s_rx_byte_count;
}

uint32_t TjcUart_GetOverflowCount(void)
{
    return s_overflow_count;
}

uint32_t TjcUart_GetErrorCount(void)
{
    return s_error_count;
}

void TjcUart_HandleRxComplete(UART_HandleTypeDef *huart)
{
    uint16_t next_head;

    if (huart->Instance != TJC_UART_INSTANCE) {
        return;
    }

    next_head = (uint16_t) ((s_rx_head + 1U) & TJC_UART_RX_BUFFER_MASK);
    if (next_head == s_rx_tail) {
        s_overflow_count++;
    } else {
        s_rx_buffer[s_rx_head] = s_rx_byte;
        s_rx_head = next_head;
        s_rx_byte_count++;
    }

    if (HAL_UART_Receive_IT(&huart2, &s_rx_byte, 1U) != HAL_OK) {
        s_error_count++;
    }
}

void TjcUart_HandleError(UART_HandleTypeDef *huart)
{
    if (huart->Instance != TJC_UART_INSTANCE) {
        return;
    }

    s_error_count++;
    __HAL_UART_CLEAR_OREFLAG(huart);
    if (HAL_UART_Receive_IT(&huart2, &s_rx_byte, 1U) != HAL_OK) {
        s_error_count++;
    }
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    K230Uart_HandleRxComplete(huart);
    TjcUart_HandleRxComplete(huart);
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    K230Uart_HandleError(huart);
    TjcUart_HandleError(huart);
}
