#ifndef APP_CONFIG_H
#define APP_CONFIG_H

/* 当前使用四路红外巡线；IMU 接线确认后再打开。 */
#define APP_USE_IMU 0U

#define APP_CONTROL_PERIOD_MS          20U
#define APP_BASE_SPEED_CM_S            15.0f
#define APP_IR_LINE_LOST_HOLD_MS       80U

/* 保留 K230 控制器参数，便于后续恢复视觉输入时复用现有模块。 */
#define APP_K230_LINK_TIMEOUT_MS      800U
#define APP_LINE_INVALID_HOLD_MS      120U
#define APP_K230_MIN_QUALITY           45U

#endif /* APP_CONFIG_H */
