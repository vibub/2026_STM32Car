#ifndef CHASSIS_MOTION_PROFILE_H
#define CHASSIS_MOTION_PROFILE_H

#include <stdint.h>

typedef enum {
    CHASSIS_MOTION_STATE_IDLE = 0,
    CHASSIS_MOTION_STATE_RAMPING,
    CHASSIS_MOTION_STATE_CRUISING,
    CHASSIS_MOTION_STATE_PASSED,
    CHASSIS_MOTION_STATE_BRAKING,
    CHASSIS_MOTION_STATE_STOPPED
} ChassisMotionState;

typedef struct {
    float target_speed_cm_s;       /* 平滑行驶的巡航目标速度，单位为厘米每秒。 */
    float maximum_accel_cm_s2;     /* 正向加速阶段允许的最大加速度，单位为厘米每二次方秒。 */
    float maximum_decel_cm_s2;     /* 降速阶段允许的最大减速度绝对值，单位为厘米每二次方秒。 */
    float maximum_jerk_cm_s3;      /* 命令加速度允许的最大变化率，单位为厘米每三次方秒。 */
    float completion_distance_cm;  /* 行程达到事件的里程；0 表示只巡航、不判定通过。 */
} ChassisMotionConfig;

typedef struct {
    ChassisMotionState state;
    ChassisMotionConfig config;
    uint8_t active;
    uint8_t time_ready;
    uint8_t distance_reached;
    uint8_t stop_requested;
    uint32_t last_update_ms;
    float command_speed_cm_s;
    float command_accel_cm_s2;
    float travelled_distance_cm;
} ChassisMotionProfile;

typedef struct {
    ChassisMotionState state;
    uint8_t active;
    uint8_t distance_reached;
    uint8_t stop_requested;
    uint8_t stopped;
    float target_speed_cm_s;
    float command_speed_cm_s;
    float command_accel_cm_s2;
    float travelled_distance_cm;
} ChassisMotionOutput;

void ChassisMotionProfile_Init(ChassisMotionProfile *profile);
uint8_t ChassisMotionProfile_Start(ChassisMotionProfile *profile,
                                   uint32_t now_ms,
                                   float initial_speed_cm_s,
                                   const ChassisMotionConfig *config);
void ChassisMotionProfile_Stop(ChassisMotionProfile *profile);
void ChassisMotionProfile_SetTargetSpeed(ChassisMotionProfile *profile,
                                         float target_speed_cm_s);
void ChassisMotionProfile_RequestStop(ChassisMotionProfile *profile);
void ChassisMotionProfile_Update(ChassisMotionProfile *profile,
                                 uint32_t now_ms,
                                 float distance_delta_cm,
                                 ChassisMotionOutput *output);

#endif /* CHASSIS_MOTION_PROFILE_H */
