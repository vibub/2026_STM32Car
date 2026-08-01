#ifndef APP_MAIN_H
#define APP_MAIN_H

#include <stdint.h>

#include "dynamic_run_task.h"

extern volatile uint8_t g_app_init_step;

#define APP_MODE_STOP       0x00U /* 停车模式编号。 */
#define APP_MODE_BASIC_LINE 0x01U /* 基础八路红外巡线模式编号。 */
#define APP_MODE_AB         0x06U /* 第四项：A 到 B 中心稳定通过。 */
#define APP_MODE_ABCD       0x07U /* 第五项：ABCD 整圈中心稳定通过。 */

typedef enum {
    APP_DYNAMIC_START_REJECT_NONE = 0,
    APP_DYNAMIC_START_REJECT_NOT_STOPPED,
    APP_DYNAMIC_START_REJECT_CALIBRATING,
    APP_DYNAMIC_START_REJECT_NOT_CALIBRATED,
    APP_DYNAMIC_START_REJECT_OTHER_TASK,
    APP_DYNAMIC_START_REJECT_BALANCE_ENABLE
} AppDynamicStartRejectReason;

typedef enum {
    APP_SHUTTLE_START_REJECT_NONE = 0,
    APP_SHUTTLE_START_REJECT_NOT_STOPPED,
    APP_SHUTTLE_START_REJECT_BUSY,
    APP_SHUTTLE_START_REJECT_CENTER,
    APP_SHUTTLE_START_REJECT_CONFIG
} AppBallShuttleStartRejectReason;

/* USART1 PID调参遥测快照；由App一次性复制，避免协议层直接访问控制器内部状态。 */
typedef struct {
    uint8_t enabled;
    uint8_t tracking_valid;
    uint8_t timed_out;
    uint8_t integral_active;
    uint8_t braking_active;
    uint8_t attitude_valid;
    uint8_t imu_fault;
    uint8_t mapping_fault;
    uint8_t trajectory_phase;
    float target_mm;
    float position_mm;
    float velocity_mm_s;
    float error_mm;
    float p_term_deg;
    float i_term_deg;
    float d_term_deg;
    float desired_pipe_pitch_deg;
    float measured_pipe_pitch_deg;
    float measured_pipe_rate_dps;
    float pipe_p_term_deg;
    float pipe_d_term_deg;
    uint32_t servo_pwm_us;
} AppBallPidSnapshot;

extern volatile uint8_t g_app_dynamic_run_failure;
extern volatile uint8_t g_app_dynamic_start_reject;
extern volatile uint8_t g_app_ball_shuttle_start_reject;

void App_Init(void);
void App_Loop(void);
uint8_t App_RequestMode(uint8_t mode);
uint8_t App_GetCurrentMode(void);
uint8_t App_StartSmoothLineRun(float completion_distance_cm);
uint8_t App_StartSmoothAbRun(void);
uint8_t App_StartSmoothLapRun(void);
uint8_t App_StartAbRun(void);
uint8_t App_StartAbcdRun(void);
uint8_t App_IsDynamicRunCommandLocked(void);
uint8_t App_SetBallBalanceEnabled(uint8_t enabled);
uint8_t App_GetBallBalanceEnabled(void);
uint8_t App_StartBallShuttle(void);
uint8_t App_StopBallShuttle(void);
uint8_t App_SetBallShuttleConfig(float phase_1_angle_deg,
                                 uint32_t phase_1_time_ms,
                                 float phase_2_angle_deg,
                                 uint32_t phase_2_time_ms,
                                 float phase_3_angle_deg,
                                 uint32_t phase_3_time_ms);
uint8_t App_SaveBallShuttleTimes(uint32_t phase_1_time_ms,
                                 uint32_t phase_2_time_ms,
                                 uint32_t phase_3_time_ms);
void App_GetBallShuttleConfig(float *phase_1_angle_deg,
                              uint32_t *phase_1_time_ms,
                              float *phase_2_angle_deg,
                              uint32_t *phase_2_time_ms,
                              float *phase_3_angle_deg,
                              uint32_t *phase_3_time_ms);
uint8_t App_ResetBallShuttleConfig(void);
uint8_t App_GetBallShuttleState(void);
uint8_t App_GetBallShuttleStartReject(void);
uint8_t App_StartPipeCalibration(void);
uint8_t App_GetPipeCalibrationActive(void);
uint8_t App_GetPipeCalibrationValid(void);
void App_SetBallBalanceTargetMm(float target_mm);
uint8_t App_SetBallPidGains(float position_kp,
                            float position_ki,
                            float position_kd,
                            float pipe_kp,
                            float pipe_kd_s);
void App_GetBallPidGains(float *position_kp,
                         float *position_ki,
                         float *position_kd,
                         float *pipe_kp,
                         float *pipe_kd_s);
void App_GetBallPidSnapshot(AppBallPidSnapshot *snapshot);

/* 上位机标定模式开关与水管姿态数据访问（供 HostCalibrationProtocol 使用）。 */
uint8_t App_SetHostCalibrationEnabled(uint8_t enabled);
uint8_t App_GetHostCalibrationEnabled(void);
float App_GetPipePitchDeg(void);
float App_GetPipePitchRateDps(void);
uint8_t App_GetPipeAttitudeValid(void);
float App_GetPipeRawAccelPitchDeg(void);
float App_GetPipeRawPitchRateDps(void);

extern volatile uint8_t g_app_host_calibration_enabled;

#endif /* APP_MAIN_H */
