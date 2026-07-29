#ifndef APP_CONFIG_H
#define APP_CONFIG_H

/* 第一阶段只启用 K230 视觉直接差速；IMU 接线确认后再打开。 */
#define APP_USE_IMU 0U

#define APP_CONTROL_PERIOD_MS       20U
#define APP_K230_LINK_TIMEOUT_MS   800U
#define APP_LINE_INVALID_HOLD_MS   120U
#define APP_K230_MIN_QUALITY        45U
#define APP_BASE_SPEED_CM_S         15.0f

#endif /* APP_CONFIG_H */
