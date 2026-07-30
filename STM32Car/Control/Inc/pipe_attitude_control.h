#ifndef PIPE_ATTITUDE_CONTROL_H
#define PIPE_ATTITUDE_CONTROL_H

#include <stdint.h>

#include "linkage_kinematics.h"

typedef enum {
    PIPE_CALIBRATION_IDLE = 0,
    PIPE_CALIBRATION_SETTLE_LEVEL,
    PIPE_CALIBRATION_SAMPLE_LEVEL,
    PIPE_CALIBRATION_SETTLE_MAPPING_POINT,
    PIPE_CALIBRATION_SAMPLE_MAPPING_POINT,
    PIPE_CALIBRATION_SETTLE_RETURN,
    PIPE_CALIBRATION_SAMPLE_RETURN,
    PIPE_CALIBRATION_RETURN_ABORTED,
    PIPE_CALIBRATION_RETURN_FAILED,
    PIPE_CALIBRATION_SUCCESS,
    PIPE_CALIBRATION_FAILED
} PipeCalibrationState;

typedef enum {
    PIPE_CALIBRATION_EVENT_NONE = 0,
    PIPE_CALIBRATION_EVENT_READY,
    PIPE_CALIBRATION_EVENT_LEVEL,
    PIPE_CALIBRATION_EVENT_MAPPING_POINT,
    PIPE_CALIBRATION_EVENT_RETURN,
    PIPE_CALIBRATION_EVENT_SUCCESS,
    PIPE_CALIBRATION_EVENT_FAILED_IMU,
    PIPE_CALIBRATION_EVENT_FAILED_MOTION,
    PIPE_CALIBRATION_EVENT_FAILED_RANGE,
    PIPE_CALIBRATION_EVENT_ABORTED
} PipeCalibrationEvent;

typedef struct {
    uint8_t valid;
    uint32_t timestamp_ms;
    float accel_x_g;
    float accel_y_g;
    float accel_z_g;
    float gyro_x_dps;
    float gyro_y_dps;
    float gyro_z_dps;
} PipeAttitudeMeasurement;

typedef struct {
    PipeCalibrationState calibration_state;
    PipeCalibrationEvent pending_event;
    uint8_t event_pending;
    uint8_t level_calibrated;
    uint8_t calibration_valid;
    uint8_t filter_ready;
    uint8_t accel_trusted;
    uint8_t sample_ready;
    uint8_t settle_ready;
    uint8_t task_time_ready;
    uint8_t calibration_point_index;
    int8_t imu_pitch_sign;
    uint16_t sample_count;
    uint16_t rejected_sample_count;
    uint32_t state_started_ms;
    uint32_t settle_started_ms;
    uint32_t last_sample_ms;
    uint32_t last_task_ms;
    float sample_sum;
    float gyro_bias_sum;
    float accel_pitch_sum;
    float gyro_x_bias_dps;
    float accel_pitch_zero_deg;
    float raw_accel_pitch_deg;
    float accel_pitch_deg;
    float pitch_deg;
    float pitch_rate_dps;
    float negative_pitch_deg;
    float center_pitch_deg;
    float positive_pitch_deg;
    float return_pitch_deg;
    float model_scale;
    float model_offset_deg;
    float mapping_pitch_deg[LINKAGE_KINEMATICS_CALIBRATION_POINT_COUNT];
    float calibration_target_pitch_deg;
    float servo_target_deg;
    float servo_override_deg;
} PipeAttitudeControl;

typedef struct {
    uint8_t attitude_valid;
    uint8_t accel_trusted;
    uint8_t level_calibrated;
    uint8_t calibration_active;
    uint8_t calibration_valid;
    uint8_t servo_override_active;
    uint8_t status_changed;
    int8_t imu_pitch_sign;
    PipeCalibrationState calibration_state;
    PipeCalibrationEvent calibration_event;
    uint8_t calibration_point_index;
    uint16_t calibration_sample_count;
    float servo_override_deg;
    float servo_target_deg;
    float calibration_target_pitch_deg;
    float gyro_x_bias_dps;
    float accel_pitch_zero_deg;
    float raw_accel_pitch_deg;
    float raw_pitch_deg;
    float raw_pitch_rate_dps;
    float accel_pitch_deg;
    float pitch_deg;
    float pitch_rate_dps;
    float negative_pitch_deg;
    float center_pitch_deg;
    float positive_pitch_deg;
    float return_pitch_deg;
    float model_scale;
    float model_offset_deg;
    float mapping_pitch_deg[LINKAGE_KINEMATICS_CALIBRATION_POINT_COUNT];
} PipeAttitudeOutput;

void PipeAttitudeControl_Init(PipeAttitudeControl *control);
uint8_t PipeAttitudeControl_StartCalibration(PipeAttitudeControl *control,
                                             uint32_t now_ms,
                                             float current_servo_angle_deg);
void PipeAttitudeControl_AbortCalibration(PipeAttitudeControl *control,
                                          uint32_t now_ms);
uint8_t PipeAttitudeControl_IsCalibrationActive(
    const PipeAttitudeControl *control);
uint8_t PipeAttitudeControl_IsCalibrationValid(
    const PipeAttitudeControl *control);
void PipeAttitudeControl_Update(PipeAttitudeControl *control,
                                uint32_t now_ms,
                                const PipeAttitudeMeasurement *measurement,
                                float chassis_accel_cm_s2,
                                PipeAttitudeOutput *output);

#endif /* PIPE_ATTITUDE_CONTROL_H */
