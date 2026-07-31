#ifndef HOST_CALIBRATION_PROTOCOL_H
#define HOST_CALIBRATION_PROTOCOL_H

#include <stdint.h>

/* USART1标定与PID调参协议（见Tools/pipe_calibration/PROTOCOL.md）。
 * CAL 1仅用于停车后的舵机/原始IMU标定；PID调参命令在CAL 0下运行，
 * 写增益、设置目标和启停平衡仍受停车模式及原有安全门控约束。 */
void HostCalibrationProtocol_Init(void);
void HostCalibrationProtocol_Task(void);

/* IAR Watch 调试变量，仅用于观察，不参与控制。 */
extern volatile uint8_t g_host_calibration_enabled;
extern volatile uint32_t g_host_calibration_rx_byte_count;
extern volatile uint32_t g_host_calibration_valid_command_count;
extern volatile uint32_t g_host_calibration_invalid_command_count;
extern volatile uint32_t g_host_calibration_stream_count;
extern volatile char g_host_calibration_last_command[16];

#endif /* HOST_CALIBRATION_PROTOCOL_H */
