#ifndef APP_CONFIG_H
#define APP_CONFIG_H

/* LSM6DSV16X 已通过 I2C2 接入，使用 PB10/PB11。 */
#define APP_USE_IMU                         1U  /* IMU 功能开关，1 表示启用初始化和轮询。 */
#ifndef APP_ENABLE_BALANCE_LOG
#define APP_ENABLE_BALANCE_LOG              0U  /* 默认关闭诊断日志；需要采集时设为1，占用约98 KiB RAM。 */
#endif

/* 上位机标定链路：USART1（PB6/PB7，115200 8N1）。
 * 默认关闭（APP_HOST_CALIBRATION_DEFAULT=0），收到 CAL 1 后才进入标定模式并停车，
 * 避免标定命令与循迹/平衡任务互相干扰。 */
#define APP_USE_HOST_CALIBRATION            1U  /* 上位机标定协议总开关，0 表示整模块不编译。 */
#define APP_HOST_CALIBRATION_DEFAULT        0U  /* 上电时标定模式默认状态。 */

#define APP_CONTROL_PERIOD_MS              20U  /* 主控制循环周期，单位为毫秒。 */
#define APP_BASE_SPEED_CM_S                40.0f /* 基础巡线目标速度，单位为厘米每秒。 */
#define APP_LINE_MAX_ACCEL_CM_S2           25.0f /* 动态滚球行驶时允许的最大目标加速度。 */
#define APP_LINE_MAX_DECEL_CM_S2           30.0f /* 动态滚球行驶时允许的最大目标减速度。 */
#define APP_LINE_MAX_JERK_CM_S3           100.0f /* 动态滚球行驶时允许的最大目标加速度变化率。 */
#define APP_AB_SPEED_CM_S                   35.0f /* 第四项 AB 在 8 秒限制内保留余量的巡航速度。 */
#define APP_AB_MAX_ACCEL_CM_S2              10.0f /* AB 起步使用较小纵向加速度，减小钢珠瞬态位移。 */
#define APP_AB_MAX_DECEL_CM_S2               8.0f /* 通过 B 后柔和停车，不参与通过计时。 */
#define APP_AB_MAX_JERK_CM_S3                30.0f /* AB 加速度变化率上限。 */
#define APP_ABCD_SPEED_CM_S                  27.5f /* 35 cm/s实测22 s/圈，按比例降速到约28 s/圈；不改加速度曲线。 */
#define APP_ABCD_MAX_ACCEL_CM_S2 APP_AB_MAX_ACCEL_CM_S2
#define APP_ABCD_MAX_DECEL_CM_S2 APP_AB_MAX_DECEL_CM_S2
#define APP_ABCD_MAX_JERK_CM_S3   APP_AB_MAX_JERK_CM_S3
#define APP_ABCD_WALL_COUPLING_GAIN           0.020f /* 首圈实测：B-C约4.49 s内由0漂到-50 mm，对应约2%的向心加速度轴向耦合。 */
#define APP_ENABLE_K230_TELEMETRY                 1U /* 实车调参：向 K230 发送遥测，过 A 后由 K230 落盘到 SD 卡。 */
#define APP_K230_TELEMETRY_PERIOD_MS             50U /* ABCD遥测20 Hz，K230仅缓存，过A后再写SD卡。 */
#define APP_AB_PASS_DISTANCE_CM           150.0f /* 第四项 A 到 B 直线的名义通过里程。 */
#define APP_LAP_PASS_DISTANCE_CM          614.2f /* 第五、六项整圈的名义通过里程。 */
#define APP_DYNAMIC_PREPARE_TIMEOUT_MS   10000U  /* 第四、五题等待钢球回中稳定的最长时间。 */
#define APP_DYNAMIC_READY_HOLD_MS          300U  /* 钢球中心条件需要连续保持的起步确认时间。 */
#define APP_DYNAMIC_READY_SAMPLE_GAP_MS    200U  /* 准备阶段相邻有效视觉样本允许的最大间隔。 */
#define APP_DYNAMIC_READY_POSITION_MM       5.0f /* 允许底盘起步的钢球位置误差绝对值上限。 */
#define APP_DYNAMIC_READY_SPEED_MM_S        15.0f /* 允许底盘起步的钢球速度绝对值上限。 */
#define APP_AB_TIMEOUT_MS                  8000U  /* 第四题从起步到通过 B 的软件限时。 */
#define APP_ABCD_TIMEOUT_MS               30000U  /* 第五题从起步到通过 A 的软件限时。 */
#define APP_BALL_LIMIT_WARNING_MM           10.0f /* 钢球中心误差达到 1 cm 时显示红圈。 */
#define APP_BALL_LIMIT_CLEAR_MM              8.0f /* 误差回到 0.8 cm 内后允许清除红圈。 */
#define APP_BALL_LIMIT_CLEAR_SAMPLES           3U /* 清除红圈需要连续满足的有效视觉样本数。 */
#define APP_BALL_OVERRANGE_CONFIRM_SAMPLES     2U /* 锁存超差结果需要连续越界的有效样本数。 */
#define APP_DYNAMIC_IMU_FAULT_DEBOUNCE_MS   300U  /* 第四、五题 IMU/姿态持续不健康才判失败，容忍起步变速瞬态。 */
#define APP_DYNAMIC_VISION_FAULT_DEBOUNCE_MS 500U /* 第四、五题视觉持续失联才判失败，容忍 K230 偶发丢帧。 */
#define APP_IR_LINE_LOST_RECOVERY_MS      200U  /* 丢线后按最近方向温和恢复的持续时间。 */
#define APP_IR_LINE_LOST_STOP_MS          500U  /* 持续丢线达到该时间后停车。 */

#endif /* APP_CONFIG_H */
