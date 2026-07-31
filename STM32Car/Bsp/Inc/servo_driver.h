#ifndef SERVO_DRIVER_H
#define SERVO_DRIVER_H

/* 连接件重新定向后，连杆只允许使用实机确认无干涉的舵机角窗口。 */
#define SERVO_DRIVER_MIN_ANGLE_DEG   0.0f /* 重新装配后舵机允许设置的最小内部角。 */
#define SERVO_DRIVER_MAX_ANGLE_DEG 180.0f /* 连杆机构允许设置的最大舵机内部角。 */

void ServoDriver_Init(float initial_angle_deg);
void ServoDriver_SetAngle(float angle_deg);
float ServoDriver_GetAngle(void);

#endif /* SERVO_DRIVER_H */
