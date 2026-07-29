#ifndef APP_CONFIG_H
#define APP_CONFIG_H

/* 当前使用四路红外巡线；IMU 接线确认后再打开。 */
#define APP_USE_IMU 0U

#define APP_CONTROL_PERIOD_MS          20U
#define APP_BASE_SPEED_CM_S            30.0f
#define APP_IR_LINE_LOST_HOLD_MS       80U

#endif /* APP_CONFIG_H */
