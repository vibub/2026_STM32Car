#ifndef APP_CONFIG_H
#define APP_CONFIG_H

/* LSM6DSV16X 已通过 I2C2 接入，使用 PB10/PB11。 */
#define APP_USE_IMU                         1U  /* IMU 功能开关，1 表示启用初始化和轮询。 */

#define APP_CONTROL_PERIOD_MS              20U  /* 主控制循环周期，单位为毫秒。 */
#define APP_BASE_SPEED_CM_S                40.0f /* 基础巡线目标速度，单位为厘米每秒。 */
#define APP_IR_LINE_LOST_RECOVERY_MS      200U  /* 丢线后按最近方向温和恢复的持续时间。 */
#define APP_IR_LINE_LOST_STOP_MS          500U  /* 持续丢线达到该时间后停车。 */

#endif /* APP_CONFIG_H */
