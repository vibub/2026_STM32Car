#ifndef APP_CONFIG_H
#define APP_CONFIG_H

/* 当前使用四路红外巡线；IMU 接线确认后再打开。 */
#define APP_USE_IMU 0U

#define APP_CONTROL_PERIOD_MS             20U
#define APP_BASE_SPEED_CM_S               40.0f
#define APP_IR_LINE_GAP_BRIDGE_MS        200U
#define APP_IR_LINE_LOST_STOP_MS         500U

#endif /* APP_CONFIG_H */
