#ifndef HOST_CALIBRATION_PROTOCOL_H
#define HOST_CALIBRATION_PROTOCOL_H

#include <stdint.h>

/* 上位机标定协议（见 Tools/pipe_calibration/PROTOCOL.md）。
 * 标定模式开关：CAL 1 进入（同时停车），CAL 0 退出；默认关闭。
 * 未开启标定模式时，除 PING/CAL 外的命令一律返回 ERR disabled。 */
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
