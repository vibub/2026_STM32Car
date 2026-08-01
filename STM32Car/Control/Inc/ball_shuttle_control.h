#ifndef BALL_SHUTTLE_CONTROL_H
#define BALL_SHUTTLE_CONTROL_H

#include <stdint.h>

#define BALL_SHUTTLE_PHASE_COUNT                 3U
#define BALL_SHUTTLE_DEFAULT_PHASE_1_ANGLE_DEG  3.0f
#define BALL_SHUTTLE_DEFAULT_PHASE_2_ANGLE_DEG  3.0f
#define BALL_SHUTTLE_DEFAULT_PHASE_3_ANGLE_DEG  5.0f
#define BALL_SHUTTLE_DEFAULT_PHASE_1_TIME_MS    700U
#define BALL_SHUTTLE_DEFAULT_PHASE_2_TIME_MS    848U
#define BALL_SHUTTLE_DEFAULT_PHASE_3_TIME_MS    545U
#define BALL_SHUTTLE_MAX_ANGLE_DEG                5.5f
#define BALL_SHUTTLE_MIN_PHASE_TIME_MS            20U
#define BALL_SHUTTLE_MAX_PHASE_TIME_MS          5000U
#define BALL_SHUTTLE_MAX_TOTAL_TIME_MS          5000U

typedef enum {
    BALL_SHUTTLE_STATE_IDLE = 0,
    BALL_SHUTTLE_STATE_PHASE_1_NEGATIVE,
    BALL_SHUTTLE_STATE_PHASE_2_POSITIVE,
    BALL_SHUTTLE_STATE_PHASE_3_NEGATIVE,
    BALL_SHUTTLE_STATE_HOLDING_LEVEL,
    BALL_SHUTTLE_STATE_FAILED
} BallShuttleState;

typedef enum {
    BALL_SHUTTLE_FAILURE_NONE = 0,
    BALL_SHUTTLE_FAILURE_PIPE_CONTROL
} BallShuttleFailure;

typedef enum {
    BALL_SHUTTLE_EVENT_NONE = 0,
    BALL_SHUTTLE_EVENT_PHASE_2,
    BALL_SHUTTLE_EVENT_PHASE_3,
    BALL_SHUTTLE_EVENT_SUCCEEDED,
    BALL_SHUTTLE_EVENT_FAILED
} BallShuttleEvent;

typedef struct {
    float angle_magnitude_deg[BALL_SHUTTLE_PHASE_COUNT];
    uint32_t duration_ms[BALL_SHUTTLE_PHASE_COUNT];
} BallShuttleConfig;

typedef struct {
    uint8_t pipe_control_enabled;
    uint8_t manual_pipe_mode_enabled;
    uint8_t attitude_valid;
    uint8_t imu_fault;
    uint8_t mapping_fault;
} BallShuttleMeasurement;

typedef struct {
    BallShuttleState state;
    BallShuttleFailure failure;
    BallShuttleConfig config;
    uint32_t started_ms;
    uint32_t phase_started_ms;
    uint32_t elapsed_ms;
    float target_pipe_pitch_deg;
} BallShuttleControl;

void BallShuttleControl_Init(BallShuttleControl *control);
uint8_t BallShuttleControl_SetConfig(BallShuttleControl *control,
                                     const BallShuttleConfig *config);
void BallShuttleControl_GetConfig(const BallShuttleControl *control,
                                  BallShuttleConfig *config);
uint8_t BallShuttleControl_ResetConfig(BallShuttleControl *control);
uint8_t BallShuttleControl_Start(BallShuttleControl *control,
                                 uint32_t now_ms);
void BallShuttleControl_HoldLevel(BallShuttleControl *control,
                                  uint32_t now_ms);
void BallShuttleControl_Stop(BallShuttleControl *control);
BallShuttleEvent BallShuttleControl_Update(
    BallShuttleControl *control,
    uint32_t now_ms,
    const BallShuttleMeasurement *measurement);
float BallShuttleControl_GetTargetPipePitchDeg(
    const BallShuttleControl *control);
uint8_t BallShuttleControl_IsRunning(const BallShuttleControl *control);
uint8_t BallShuttleControl_IsHolding(const BallShuttleControl *control);

#endif /* BALL_SHUTTLE_CONTROL_H */
