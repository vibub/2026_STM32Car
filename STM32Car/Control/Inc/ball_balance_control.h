#ifndef BALL_BALANCE_CONTROL_H
#define BALL_BALANCE_CONTROL_H

#include <stdint.h>

#include "linkage_kinematics.h"

#define BALL_BALANCE_DEFAULT_KP_DEG_PER_MM       0.014f /* 钢球位置比例增益，单位为度每毫米。 */
#define BALL_BALANCE_DEFAULT_KI_DEG_PER_MM_S     0.008f /* 接近目标点时的位置积分增益，单位为度每毫米秒。 */
#define BALL_BALANCE_DEFAULT_KD_DEG_PER_MM_S     0.014f /* 钢球速度阻尼增益，单位为度每毫米每秒。 */
#define BALL_BALANCE_DEFAULT_PIPE_KP              0.60f /* IMU姿态角误差比例修正。 */
#define BALL_BALANCE_DEFAULT_PIPE_KD_S            0.050f /* 使用120 Hz陀螺仪X轴角速度的阻尼系数，单位为秒。 */

typedef struct {
    uint8_t valid;
    int32_t position_d10_mm;
    uint8_t quality;
    uint16_t sequence;
} BallBalanceMeasurement;

typedef struct {
    uint8_t valid;
    uint32_t timestamp_ms;
    float pitch_deg;
    float pitch_rate_dps;
} BallBalanceAttitude;

typedef enum {
    BALL_BALANCE_PHASE_WAIT_PIPE = 0,
    BALL_BALANCE_PHASE_BREAKAWAY = 1,
    BALL_BALANCE_PHASE_MOVING = 2,
    BALL_BALANCE_PHASE_HOLD = 3,
    BALL_BALANCE_PHASE_MANUAL_PIPE = 4
} BallBalancePhase;

typedef struct {
    uint8_t enabled_requested;
    uint8_t enabled;
    uint8_t tracking_valid;
    uint8_t timed_out;
    uint8_t filter_ready;
    uint8_t sequence_ready;
    uint8_t task_time_ready;
    uint8_t mapping_valid;
    uint8_t mapping_fault;
    uint8_t attitude_ready;
    uint8_t attitude_stable_ready;
    uint8_t imu_fault;
    uint8_t integral_active;
    uint8_t braking_active;
    uint8_t zero_hold_freeze_active;
    uint8_t turnaround_freeze_active;
    uint8_t outward_motion_seen;
    uint8_t previous_error_ready;
    uint8_t zero_crossed;
    uint8_t pipe_settled;
    uint8_t pipe_settle_timer_ready;
    uint8_t stationary_sample_count;
    uint8_t moving_sample_count;
    int8_t moving_direction;
    uint8_t kick_attempt_count;
    uint8_t trajectory_phase;
    uint8_t feedforward_cache_valid;
    uint8_t desired_cache_valid;
    uint8_t endpoint_hold_enabled;
    uint8_t transport_mode_enabled;
    uint8_t manual_pipe_mode_enabled;
    uint8_t transport_imu_valid;
    uint8_t transport_wall_valid;
    uint8_t spike_reject_active;
    uint16_t last_sequence;
    uint32_t enabled_since_ms;
    uint32_t last_valid_ms;
    uint32_t last_spike_reject_ms;
    uint32_t last_task_ms;
    uint32_t last_attitude_ms;
    uint32_t last_attitude_timestamp_ms;
    uint32_t attitude_stable_since_ms;
    uint32_t attitude_unsafe_since_ms; /* 姿态不安全连续起始时刻，用于故障去抖。 */
    uint32_t pipe_settle_since_ms;
    uint32_t phase_started_ms;
    uint32_t kick_stage_started_ms;
    uint32_t turnaround_freeze_started_ms;
    float target_mm;
    float manual_pipe_target_deg;
    float position_kp_deg_per_mm;
    float position_ki_deg_per_mm_s;
    float position_kd_deg_per_mm_s;
    float pipe_kp;
    float pipe_kd_s;
    float last_measurement_dt_s;
    float filtered_position_mm;
    float predicted_position_mm;
    float predicted_accel_mm_s2;
    float velocity_mm_s;
    float integral_mm_s;
    float error_mm;
    float previous_error_mm;
    float p_term_deg;
    float i_term_deg;
    float d_term_deg;
    float brake_term_deg;
    float transport_command_accel_cm_s2;
    float transport_imu_disturbance_deg;
    float transport_wall_accel_cm_s2;
    float transport_wall_feedforward_pitch_deg;
    float transport_feedforward_pitch_deg;
    float transport_feedback_pitch_deg;
    float phase_pitch_deg;
    float stop_distance_mm;
    float desired_pipe_pitch_deg;
    float command_pipe_pitch_deg;
    float pipe_command_rate_dps;
    float measured_pipe_pitch_deg;
    float measured_pipe_rate_dps;
    float pipe_pitch_error_deg;
    float pipe_p_term_deg;
    float pipe_d_term_deg;
    float model_scale;
    float model_offset_deg;
    float mapping_pitch_deg[LINKAGE_KINEMATICS_CALIBRATION_POINT_COUNT];
    float minimum_pipe_pitch_deg;
    float maximum_pipe_pitch_deg;
    float feedforward_model_pitch_deg;
    float desired_model_pitch_deg;
    float servo_feedforward_deg; /* 兼容字段：线性前馈相对水管角。 */
    float desired_angle_deg;     /* 兼容字段：姿态修正后的目标相对水管角。 */
    float command_angle_deg;     /* 兼容字段：限速后的执行器相对水管角。 */
} BallBalanceControl;

typedef struct {
    uint8_t enable_requested;
    uint8_t enabled;
    uint8_t enable_pending;
    uint8_t tracking_valid;
    uint8_t timed_out;
    uint8_t measurement_accepted;
    uint8_t attitude_valid;
    uint8_t imu_fault;
    uint8_t mapping_valid;
    uint8_t mapping_fault;
    uint8_t integral_active;
    uint8_t braking_active;
    uint8_t zero_crossed;
    uint8_t turnaround_freeze_active;
    uint8_t pipe_settled;
    uint8_t moving_sample_count;
    int8_t moving_direction;
    uint8_t kick_attempt_count;
    uint8_t trajectory_phase;
    uint8_t endpoint_hold_enabled;
    uint8_t transport_mode_enabled;
    uint8_t manual_pipe_mode_enabled;
    float target_mm;
    float manual_pipe_target_deg;
    float position_kp_deg_per_mm;
    float position_ki_deg_per_mm_s;
    float position_kd_deg_per_mm_s;
    float pipe_kp;
    float pipe_kd_s;
    float filtered_position_mm;
    float predicted_position_mm;
    float predicted_accel_mm_s2;
    float velocity_mm_s;
    float integral_mm_s;
    float error_mm;
    float p_term_deg;
    float i_term_deg;
    float d_term_deg;
    float brake_term_deg;
    float transport_command_accel_cm_s2;
    float transport_imu_disturbance_deg;
    float transport_wall_accel_cm_s2;
    float transport_wall_feedforward_pitch_deg;
    float transport_feedforward_pitch_deg;
    float transport_feedback_pitch_deg;
    float phase_pitch_deg;
    float stop_distance_mm;
    float desired_pipe_pitch_deg;
    float command_pipe_pitch_deg;
    float pipe_command_rate_dps;
    float measured_pipe_pitch_deg;
    float measured_pipe_rate_dps;
    float pipe_pitch_error_deg;
    float pipe_p_term_deg;
    float pipe_d_term_deg;
    float model_scale;
    float model_offset_deg;
    float mapping_pitch_deg[LINKAGE_KINEMATICS_CALIBRATION_POINT_COUNT];
    float minimum_pipe_pitch_deg;
    float maximum_pipe_pitch_deg;
    float feedforward_model_pitch_deg;
    float desired_model_pitch_deg;
    float servo_feedforward_deg; /* 兼容字段：线性前馈相对水管角。 */
    float desired_angle_deg;     /* 兼容字段：姿态修正后的目标相对水管角。 */
    float command_angle_deg;     /* 兼容字段：限速后的执行器相对水管角。 */
} BallBalanceOutput;

void BallBalanceControl_Init(BallBalanceControl *control);
void BallBalanceControl_SetEnabled(BallBalanceControl *control,
                                   uint8_t enabled,
                                   uint32_t now_ms);
uint8_t BallBalanceControl_IsEnabled(const BallBalanceControl *control);
uint8_t BallBalanceControl_IsActive(const BallBalanceControl *control);
void BallBalanceControl_SetTargetMm(BallBalanceControl *control,
                                    float target_mm);
void BallBalanceControl_SetEndpointHoldEnabled(BallBalanceControl *control,
                                               uint8_t enabled);
void BallBalanceControl_SetTransportMode(BallBalanceControl *control,
                                         uint8_t enabled);
void BallBalanceControl_SetManualPipeMode(BallBalanceControl *control,
                                         uint8_t enabled,
                                         float target_pipe_pitch_deg);
void BallBalanceControl_SetTransportInputs(
    BallBalanceControl *control,
    float command_accel_cm_s2,
    float imu_disturbance_pitch_deg,
    uint8_t imu_valid);
void BallBalanceControl_SetTransportWallAcceleration(
    BallBalanceControl *control,
    float wall_accel_cm_s2,
    uint8_t wall_valid);
void BallBalanceControl_SetPidGains(BallBalanceControl *control,
                                    float kp_deg_per_mm,
                                    float ki_deg_per_mm_s,
                                    float kd_deg_per_mm_s);
void BallBalanceControl_SetPipeGains(BallBalanceControl *control,
                                     float pipe_kp,
                                     float pipe_kd_s);
void BallBalanceControl_SetPipeCalibration(
    BallBalanceControl *control,
    uint8_t valid,
    float model_scale,
    float model_offset_deg,
    const float mapping_pitch_deg[
        LINKAGE_KINEMATICS_CALIBRATION_POINT_COUNT]);
float BallBalanceControl_GetNeutralAngleDeg(void);
void BallBalanceControl_Update(BallBalanceControl *control,
                               uint32_t now_ms,
                               const BallBalanceMeasurement *measurement,
                               const BallBalanceAttitude *attitude,
                               BallBalanceOutput *output);

#endif /* BALL_BALANCE_CONTROL_H */
