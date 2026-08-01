#ifndef K230_UART_H
#define K230_UART_H

#include "stm32h7xx_hal.h"
#include <stdint.h>

/* Live Watch diagnostics: these change as soon as UART4 receives any byte. */
extern volatile uint32_t g_k230_uart_rx_irq_count;
extern volatile uint8_t g_k230_uart_last_rx_byte;
extern volatile uint32_t g_k230_uart_last_rx_tick_ms;
extern volatile uint32_t g_k230_uart_error_irq_count;

#define K230_TELEMETRY_FRAME_SIZE 44U

typedef enum {
    K230_TELEMETRY_RUN_START = 1,
    K230_TELEMETRY_SAMPLE = 2,
    K230_TELEMETRY_RUN_END = 3
} K230TelemetryType;

typedef struct {
    uint32_t timestamp_ms;
    uint8_t run_mode;
    uint8_t run_state;
    uint16_t flags;
    float speed_cm_s;
    float yaw_rate_dps;
    float accel_x_g;
    float lateral_kinematic_cm_s2;
    float lateral_imu_cm_s2;
    float lateral_fused_cm_s2;
    float ball_error_mm;
    float ball_velocity_mm_s;
    float pipe_command_deg;
    float wall_accel_cm_s2;
    float wall_pitch_deg;
    float travelled_distance_cm;
    float transport_feedback_pitch_deg;
} K230TelemetryInput;

void K230Uart_Init(void);
uint8_t K230Uart_ReadByte(uint8_t *byte);
uint32_t K230Uart_GetRxByteCount(void);
uint32_t K230Uart_GetOverflowCount(void);
uint32_t K230Uart_GetErrorCount(void);
uint8_t K230Uart_SendTelemetry(K230TelemetryType type,
                               const K230TelemetryInput *input);
uint32_t K230Uart_GetTxFrameCount(void);
uint32_t K230Uart_GetTxDropCount(void);
void K230Uart_HandleRxComplete(UART_HandleTypeDef *huart);
void K230Uart_HandleTxComplete(UART_HandleTypeDef *huart);
void K230Uart_HandleError(UART_HandleTypeDef *huart);

#endif /* K230_UART_H */
