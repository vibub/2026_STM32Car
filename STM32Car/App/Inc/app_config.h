#ifndef APP_CONFIG_H
#define APP_CONFIG_H

/* 当前使用八路 I2C 红外巡线；IMU 接线确认后再打开。 */
#define APP_USE_IMU 0U                         /* IMU 功能开关，0 表示接口暂不启用。 */

#define APP_CONTROL_PERIOD_MS             20U  /* 主控制循环周期，单位为毫秒。 */
#define APP_BASE_SPEED_CM_S               40.0f /* 基础巡线目标速度，单位为厘米每秒。 */
#define APP_IR_LINE_LOST_RECOVERY_MS     200U  /* 丢线后按最近方向温和恢复的持续时间。 */
#define APP_IR_LINE_LOST_STOP_MS         500U  /* 持续丢线达到该时间后停车。 */

#endif /* APP_CONFIG_H */
