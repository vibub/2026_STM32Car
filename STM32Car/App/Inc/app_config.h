#ifndef APP_CONFIG_H
#define APP_CONFIG_H

/* LSM6DSV16X 已通过 I2C2 接入，使用 PB10/PB11。 */
#define APP_USE_IMU                         1U  /* IMU 功能开关，1 表示启用初始化和轮询。 */
#ifndef APP_ENABLE_BALANCE_LOG
#define APP_ENABLE_BALANCE_LOG              0U  /* 钢球平衡诊断日志开关，比赛固件可设为0释放约98 KiB RAM。 */
#endif

#define APP_CONTROL_PERIOD_MS              20U  /* 主控制循环周期，单位为毫秒。 */
#define APP_BASE_SPEED_CM_S                40.0f /* 基础巡线目标速度，单位为厘米每秒。 */
#define APP_LINE_MAX_ACCEL_CM_S2           25.0f /* 动态滚球行驶时允许的最大目标加速度。 */
#define APP_LINE_MAX_DECEL_CM_S2           30.0f /* 动态滚球行驶时允许的最大目标减速度。 */
#define APP_LINE_MAX_JERK_CM_S3           100.0f /* 动态滚球行驶时允许的最大目标加速度变化率。 */
#define APP_AB_SPEED_CM_S                   35.0f /* 第四项 AB 在 8 秒限制内保留余量的巡航速度。 */
#define APP_AB_MAX_ACCEL_CM_S2              10.0f /* AB 起步使用较小纵向加速度，减小钢珠瞬态位移。 */
#define APP_AB_MAX_DECEL_CM_S2               8.0f /* 通过 B 后柔和停车，不参与通过计时。 */
#define APP_AB_MAX_JERK_CM_S3                30.0f /* AB 加速度变化率上限。 */
#define APP_ABCD_SPEED_CM_S                  26.0f /* 第五项整圈目标约 25~26 秒，低于 30 秒限制。 */
#define APP_ABCD_MAX_ACCEL_CM_S2              8.0f /* ABCD 起步加速度，优先保证中心稳定。 */
#define APP_ABCD_MAX_DECEL_CM_S2              8.0f /* 通过 A 后柔和停车。 */
#define APP_ABCD_MAX_JERK_CM_S3               24.0f /* ABCD 加速度变化率上限。 */
#define APP_AB_PASS_DISTANCE_CM           150.0f /* 第四项 A 到 B 直线的名义通过里程。 */
#define APP_LAP_PASS_DISTANCE_CM          614.2f /* 第五、六项整圈的名义通过里程。 */
#define APP_IR_LINE_LOST_RECOVERY_MS      200U  /* 丢线后按最近方向温和恢复的持续时间。 */
#define APP_IR_LINE_LOST_STOP_MS          500U  /* 持续丢线达到该时间后停车。 */

#endif /* APP_CONFIG_H */
