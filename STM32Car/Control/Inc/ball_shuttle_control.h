#ifndef BALL_SHUTTLE_CONTROL_H
#define BALL_SHUTTLE_CONTROL_H

#include <stdint.h>

typedef enum {
    BALL_SHUTTLE_STATE_IDLE = 0,
    BALL_SHUTTLE_STATE_MOVING_POSITIVE,
    BALL_SHUTTLE_STATE_MOVING_NEGATIVE,
    BALL_SHUTTLE_STATE_SETTLING_NEGATIVE,
    BALL_SHUTTLE_STATE_HOLDING_NEGATIVE,
    BALL_SHUTTLE_STATE_FAILED
} BallShuttleState;

typedef enum {
    BALL_SHUTTLE_FAILURE_NONE = 0,
    BALL_SHUTTLE_FAILURE_TRACKING,
    BALL_SHUTTLE_FAILURE_BALANCE,
    BALL_SHUTTLE_FAILURE_TIMEOUT,
    BALL_SHUTTLE_FAILURE_POSITIVE_PEAK
} BallShuttleFailure;

typedef enum {
    BALL_SHUTTLE_EVENT_NONE = 0,
    BALL_SHUTTLE_EVENT_READY,
    BALL_SHUTTLE_EVENT_NOT_READY,
    BALL_SHUTTLE_EVENT_TURN_NEGATIVE,
    BALL_SHUTTLE_EVENT_ENABLE_ENDPOINT_HOLD,
    BALL_SHUTTLE_EVENT_SUCCEEDED,
    BALL_SHUTTLE_EVENT_FAILED
} BallShuttleEvent;

typedef struct {
    uint8_t ready_monitor_enabled;
    uint8_t measurement_accepted;
    uint8_t balance_enabled;
    uint8_t tracking_valid;
    uint8_t timed_out;
    uint8_t imu_fault;
    uint8_t mapping_fault;
    float filtered_position_mm;
    float predicted_position_mm;
    float stop_distance_mm;
    float velocity_mm_s;
} BallShuttleMeasurement;

typedef struct {
    BallShuttleState state;
    BallShuttleFailure failure;
    uint8_t start_ready;
    uint8_t ready_timer_active;
    uint8_t stable_ready;
    uint8_t turn_armed;
    uint8_t turn_confirm_count;
    uint8_t peak_reverse_confirm_count;
    uint8_t positive_peak_checked;
    uint8_t positive_peak_valid;
    uint32_t ready_since_ms;
    uint32_t started_ms;
    uint32_t stable_since_ms;
    uint32_t elapsed_ms;
    float predicted_turn_lead_mm;
    float predicted_peak_mm;
    float positive_peak_mm;
} BallShuttleControl;

void BallShuttleControl_Init(BallShuttleControl *control);
uint8_t BallShuttleControl_Start(BallShuttleControl *control,
                                 uint32_t now_ms);
void BallShuttleControl_Stop(BallShuttleControl *control);
BallShuttleEvent BallShuttleControl_Update(
    BallShuttleControl *control,
    uint32_t now_ms,
    const BallShuttleMeasurement *measurement);
uint8_t BallShuttleControl_IsReady(const BallShuttleControl *control);
uint8_t BallShuttleControl_IsRunning(const BallShuttleControl *control);
uint8_t BallShuttleControl_IsHolding(const BallShuttleControl *control);

#endif /* BALL_SHUTTLE_CONTROL_H */
