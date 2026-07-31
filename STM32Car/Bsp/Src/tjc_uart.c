#include "tjc_uart.h"
#include "k230_uart.h"
#include "main.h"

#define TJC_UART_RX_BUFFER_SIZE 64U                            /* TJC UART 接收环形缓冲区容量。 */
#define TJC_UART_RX_BUFFER_MASK (TJC_UART_RX_BUFFER_SIZE - 1U) /* 环形缓冲区下标回绕掩码。 */
#define TJC_UART_TX_TIMEOUT_MS  100U                           /* 9600 波特率下最长文本帧的发送超时。 */

/**
 * @brief 淘晶驰串口屏按钮“弹起事件”需要发送的命令。
 *
 * printh 5A A5 00 FF       全局停止，禁用钢球平衡并中断标定。
 * printh 5A A5 01 FE       进入八路红外巡线模式。
 * printh 5A A5 02 FD       启用中心平衡，中止第三题时回到 O 点并重新等待就绪。
 * printh 5A A5 03 FC       禁用钢球平衡并将舵机恢复到中位。
 * printh 5A A5 04 FB       停车且移除钢球后启动水管九点标定。
 * printh 5A A5 05 FA       仅在中心稳定并显示“第三题已就绪”后启动往返，不自动启用平衡。
 * printh 5A A5 0E F1       查询当前底盘停车或巡线模式。
 *
 * 推荐启动顺序：0x04 标定成功 -> 放球回 O 点 -> 0x02 中心平衡
 *               -> 等待“第三题已就绪” -> 0x05 启动第三题。
 * 注意：printh 直接发送上述四个字节，不需要追加 FF FF FF。
 */

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
    if (HAL_UART_Transmit(&huart2, &byte, 1U,
                          TJC_UART_TX_TIMEOUT_MS) != HAL_OK) {
        s_error_count++;
    }
}

void TjcUart_SendData(const uint8_t *data, uint16_t length)
{
    if ((data == 0) || (length == 0U)) {
        return;
    }
    if (HAL_UART_Transmit(&huart2, (uint8_t *) data, length,
                          TJC_UART_TX_TIMEOUT_MS) != HAL_OK) {
        s_error_count++;
    }
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
