#include "k230_uart.h"
#include "main.h"

#define K230_UART_RX_BUFFER_SIZE 256U                             /* K230 UART 接收环形缓冲区容量。 */
#define K230_UART_RX_BUFFER_MASK (K230_UART_RX_BUFFER_SIZE - 1U) /* 环形缓冲区下标回绕掩码。 */

static volatile uint8_t s_rx_buffer[K230_UART_RX_BUFFER_SIZE];
static volatile uint16_t s_rx_head;
static volatile uint16_t s_rx_tail;
static uint8_t s_rx_byte;
static volatile uint32_t s_rx_byte_count;
static volatile uint32_t s_overflow_count;
static volatile uint32_t s_error_count;
static uint8_t s_tx_buffer[K230_TELEMETRY_FRAME_SIZE];
static volatile uint8_t s_tx_busy;
static uint16_t s_tx_sequence;
static volatile uint32_t s_tx_frame_count;
static volatile uint32_t s_tx_drop_count;

volatile uint32_t g_k230_uart_rx_irq_count;
volatile uint8_t g_k230_uart_last_rx_byte;
volatile uint32_t g_k230_uart_last_rx_tick_ms;
volatile uint32_t g_k230_uart_error_irq_count;

static int16_t K230Uart_ScaleToInt16(float value, float scale)
{
    float scaled = value * scale;

    if (scaled > 32767.0f) {
        return 32767;
    }
    if (scaled < -32768.0f) {
        return -32768;
    }
    return (int16_t) ((scaled >= 0.0f) ?
                     (scaled + 0.5f) : (scaled - 0.5f));
}

static void K230Uart_WriteU16(uint8_t *buffer,
                              uint16_t offset,
                              uint16_t value)
{
    buffer[offset] = (uint8_t) value;
    buffer[offset + 1U] = (uint8_t) (value >> 8U);
}

static void K230Uart_WriteU32(uint8_t *buffer,
                              uint16_t offset,
                              uint32_t value)
{
    buffer[offset] = (uint8_t) value;
    buffer[offset + 1U] = (uint8_t) (value >> 8U);
    buffer[offset + 2U] = (uint8_t) (value >> 16U);
    buffer[offset + 3U] = (uint8_t) (value >> 24U);
}

static uint16_t K230Uart_Crc16(const uint8_t *data, uint16_t length)
{
    uint16_t crc = 0xFFFFU;
    uint16_t index;
    uint8_t bit;

    for (index = 0U; index < length; index++) {
        crc ^= (uint16_t) data[index] << 8U;
        for (bit = 0U; bit < 8U; bit++) {
            crc = ((crc & 0x8000U) != 0U) ?
                  (uint16_t) ((crc << 1U) ^ 0x1021U) :
                  (uint16_t) (crc << 1U);
        }
    }
    return crc;
}

void K230Uart_Init(void)
{
    s_rx_head = 0U;
    s_rx_tail = 0U;
    s_rx_byte_count = 0U;
    s_overflow_count = 0U;
    s_error_count = 0U;
    s_tx_busy = 0U;
    s_tx_sequence = 0U;
    s_tx_frame_count = 0U;
    s_tx_drop_count = 0U;
    g_k230_uart_rx_irq_count = 0U;
    g_k230_uart_last_rx_byte = 0U;
    g_k230_uart_last_rx_tick_ms = 0U;
    g_k230_uart_error_irq_count = 0U;
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

uint8_t K230Uart_SendTelemetry(K230TelemetryType type,
                               const K230TelemetryInput *input)
{
    uint16_t crc;

    if ((input == 0) ||
        (type < K230_TELEMETRY_RUN_START) ||
        (type > K230_TELEMETRY_RUN_END)) {
        return 0U;
    }
    if (s_tx_busy != 0U) {
        s_tx_drop_count++;
        return 0U;
    }

    s_tx_buffer[0] = 0xA5U;
    s_tx_buffer[1] = 0x5AU;
    s_tx_buffer[2] = 1U;
    s_tx_buffer[3] = (uint8_t) type;
    s_tx_buffer[4] = K230_TELEMETRY_FRAME_SIZE;
    s_tx_buffer[5] = 0U;
    K230Uart_WriteU16(s_tx_buffer, 6U, s_tx_sequence++);
    K230Uart_WriteU32(s_tx_buffer, 8U, input->timestamp_ms);
    s_tx_buffer[12] = input->run_mode;
    s_tx_buffer[13] = input->run_state;
    K230Uart_WriteU16(s_tx_buffer, 14U, input->flags);
    K230Uart_WriteU16(s_tx_buffer, 16U, (uint16_t)
        K230Uart_ScaleToInt16(input->speed_cm_s, 100.0f));
    K230Uart_WriteU16(s_tx_buffer, 18U, (uint16_t)
        K230Uart_ScaleToInt16(input->yaw_rate_dps, 100.0f));
    K230Uart_WriteU16(s_tx_buffer, 20U, (uint16_t)
        K230Uart_ScaleToInt16(input->accel_x_g, 1000.0f));
    K230Uart_WriteU16(s_tx_buffer, 22U, (uint16_t)
        K230Uart_ScaleToInt16(input->lateral_kinematic_cm_s2, 100.0f));
    K230Uart_WriteU16(s_tx_buffer, 24U, (uint16_t)
        K230Uart_ScaleToInt16(input->lateral_imu_cm_s2, 100.0f));
    K230Uart_WriteU16(s_tx_buffer, 26U, (uint16_t)
        K230Uart_ScaleToInt16(input->lateral_fused_cm_s2, 100.0f));
    K230Uart_WriteU16(s_tx_buffer, 28U, (uint16_t)
        K230Uart_ScaleToInt16(input->ball_error_mm, 10.0f));
    K230Uart_WriteU16(s_tx_buffer, 30U, (uint16_t)
        K230Uart_ScaleToInt16(input->ball_velocity_mm_s, 10.0f));
    K230Uart_WriteU16(s_tx_buffer, 32U, (uint16_t)
        K230Uart_ScaleToInt16(input->pipe_command_deg, 1000.0f));
    K230Uart_WriteU16(s_tx_buffer, 34U, (uint16_t)
        K230Uart_ScaleToInt16(input->wall_accel_cm_s2, 100.0f));
    K230Uart_WriteU16(s_tx_buffer, 36U, (uint16_t)
        K230Uart_ScaleToInt16(input->wall_pitch_deg, 1000.0f));
    K230Uart_WriteU16(s_tx_buffer, 38U, (uint16_t)
        K230Uart_ScaleToInt16(input->travelled_distance_cm, 10.0f));
    K230Uart_WriteU16(s_tx_buffer, 40U, (uint16_t)
        K230Uart_ScaleToInt16(input->transport_feedback_pitch_deg,
                              1000.0f));
    crc = K230Uart_Crc16(s_tx_buffer,
                         K230_TELEMETRY_FRAME_SIZE - 2U);
    K230Uart_WriteU16(s_tx_buffer, 42U, crc);

    s_tx_busy = 1U;
    if (HAL_UART_Transmit_IT(&huart4,
                             s_tx_buffer,
                             K230_TELEMETRY_FRAME_SIZE) != HAL_OK) {
        s_tx_busy = 0U;
        s_tx_drop_count++;
        return 0U;
    }
    return 1U;
}

uint32_t K230Uart_GetTxFrameCount(void)
{
    return s_tx_frame_count;
}

uint32_t K230Uart_GetTxDropCount(void)
{
    return s_tx_drop_count;
}

void K230Uart_HandleRxComplete(UART_HandleTypeDef *huart)
{
    uint16_t next_head;

    if (huart->Instance != K230_UART_INSTANCE) {
        return;
    }

    g_k230_uart_rx_irq_count++;
    g_k230_uart_last_rx_byte = s_rx_byte;
    g_k230_uart_last_rx_tick_ms = HAL_GetTick();

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

void K230Uart_HandleTxComplete(UART_HandleTypeDef *huart)
{
    if (huart->Instance != K230_UART_INSTANCE) {
        return;
    }
    s_tx_busy = 0U;
    s_tx_frame_count++;
}

void K230Uart_HandleError(UART_HandleTypeDef *huart)
{
    if (huart->Instance != K230_UART_INSTANCE) {
        return;
    }

    g_k230_uart_error_irq_count++;
    s_error_count++;
    __HAL_UART_CLEAR_OREFLAG(huart);
    if (HAL_UART_Receive_IT(&huart4, &s_rx_byte, 1U) != HAL_OK) {
        s_error_count++;
    }
}
