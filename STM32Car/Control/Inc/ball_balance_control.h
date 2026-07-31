#ifndef BALL_BALANCE_CONTROL_H
#define BALL_BALANCE_CONTROL_H

#include <stdint.h>

#include "linkage_kinematics.h"

#define BALL_BALANCE_DEFAULT_KP_DEG_PER_MM       0.012f
#define BALL_BALANCE_DEFAULT_KI_DEG_PER_MM_S     0.000f
#define BALL_BALANCE_DEFAULT_KD_DEG_PER_MM_S     0.015f

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
    uint8_t feedforward_cache_valid;
    uint8_t desired_cache_valid;
    uint16_t last_sequence;
    uint32_t enabled_since_ms;
    uint32_t last_valid_ms;
    uint32_t last_task_ms;
    uint32_t last_attitude_ms;
    uint32_t last_attitude_timestamp_ms;
    uint32_t attitude_stable_since_ms;
    float target_mm;
    float position_kp_deg_per_mm;
    float position_ki_deg_per_mm_s;
    float position_kd_deg_per_mm_s;
    float last_measurement_dt_s;
    float filtered_position_mm;
    float predicted_position_mm;
    float predicted_accel_mm_s2;
    float velocity_mm_s;
    float integral_mm_s;
    float error_mm;
    float p_term_deg;
    float i_term_deg;
    float d_term_deg;
    float desired_pipe_pitch_deg;
    float command_pipe_pitch_deg;
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
    float servo_feedforward_deg;
    float desired_angle_deg;
    float command_angle_deg;
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
    float target_mm;
    float position_kp_deg_per_mm;
    float position_ki_deg_per_mm_s;
    float position_kd_deg_per_mm_s;
    float filtered_position_mm;
    float predicted_position_mm;
    float predicted_accel_mm_s2;
    float velocity_mm_s;
    float integral_mm_s;
    float error_mm;
    float p_term_deg;
    float i_term_deg;
    float d_term_deg;
    float desired_pipe_pitch_deg;
    float command_pipe_pitch_deg;
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
    float servo_feedforward_deg;
    float desired_angle_deg;
    float command_angle_deg;
} BallBalanceOutput;

void BallBalanceControl_Init(BallBalanceControl *control);
void BallBalanceControl_SetEnabled(BallBalanceControl *control,
                                   uint8_t enabled,
                                   uint32_t now_ms);
uint8_t BallBalanceControl_IsEnabled(const BallBalanceControl *control);
uint8_t BallBalanceControl_IsActive(const BallBalanceControl *control);
void BallBalanceControl_SetTargetMm(BallBalanceControl *control,
                                    float target_mm);
void BallBalanceControl_SetPidGains(BallBalanceControl *control,
                                    float kp_deg_per_mm,
                                    float ki_deg_per_mm_s,
                                    float kd_deg_per_mm_s);
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
