#ifndef SERVO_DRIVER_H
#define SERVO_DRIVER_H

/* 连杆装配状态下只允许使用覆盖水管 ±6°行程的安全舵机角窗口。 */
#define SERVO_DRIVER_MIN_ANGLE_DEG 100.0f /* 连杆机构允许设置的最小舵机内部角。 */
#define SERVO_DRIVER_MAX_ANGLE_DEG 170.0f /* 连杆机构允许设置的最大舵机内部角。 */

void ServoDriver_Init(float initial_angle_deg);
void ServoDriver_SetAngle(float angle_deg);
float ServoDriver_GetAngle(void);

#endif /* SERVO_DRIVER_H */
