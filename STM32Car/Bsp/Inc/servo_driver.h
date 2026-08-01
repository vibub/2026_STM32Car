#ifndef SERVO_DRIVER_H
#define SERVO_DRIVER_H

#include <stdint.h>

#define SERVO_DRIVER_MIN_PULSE_US          500U     /* 上位机实测确认的安全最小脉宽。 */
#define SERVO_DRIVER_CENTER_PULSE_US       970U     /* 固定电池后的水管水平实测脉宽。 */
#define SERVO_DRIVER_MAX_PULSE_US         1600U     /* 上位机实测确认的安全最大脉宽。 */
#define SERVO_DRIVER_PIPE_DEG_PER_US         0.01009f /* PWM每增加1 us对应的正向水管角。 */
#define SERVO_DRIVER_MIN_PIPE_PITCH_DEG   (-4.7423f) /* 500 us相对970 us水平点的水管角下限。 */
#define SERVO_DRIVER_MAX_PIPE_PITCH_DEG     6.3567f  /* 1600 us相对970 us水平点的水管角上限。 */

/* 保留电气角接口供上位机SERVO命令兼容使用，正式平衡控制直接使用水管角或PWM。 */
#define SERVO_DRIVER_MIN_ANGLE_DEG            0.0f   /* 500 us对应的舵机电气角。 */
#define SERVO_DRIVER_MAX_ANGLE_DEG          148.5f   /* 1600 us对应的舵机电气角。 */

void ServoDriver_Init(float initial_angle_deg);
void ServoDriver_InitPulseUs(uint32_t initial_pulse_us);
void ServoDriver_SetAngle(float angle_deg);
void ServoDriver_SetPulseUs(uint32_t pulse_us);
void ServoDriver_SetPipePitchDeg(float pipe_pitch_deg);
uint8_t ServoDriver_SetCenterPulseUs(uint32_t center_pulse_us);
float ServoDriver_GetAngle(void);
uint32_t ServoDriver_GetPulseUs(void);
uint32_t ServoDriver_GetCenterPulseUs(void);
float ServoDriver_GetPipePitchDeg(void);
uint32_t ServoDriver_PipePitchToPulseUs(float pipe_pitch_deg);
float ServoDriver_PulseUsToPipePitchDeg(uint32_t pulse_us);

#endif /* SERVO_DRIVER_H */
