#ifndef INFRARED_LINE_SENSOR_H
#define INFRARED_LINE_SENSOR_H

#include <stdint.h>

#define INFRARED_LINE_X1_MASK 0x01U
#define INFRARED_LINE_X2_MASK 0x02U
#define INFRARED_LINE_X3_MASK 0x04U
#define INFRARED_LINE_X4_MASK 0x08U
#define INFRARED_LINE_ALL_MASK 0x0FU

/*
 * YH-4LXJ V1.1 已实测为黑线输出低电平、白底输出高电平。
 * 驱动统一转换为“掩码位为 1 表示检测到黑线”。
 */
#ifndef INFRARED_LINE_ACTIVE_HIGH
#define INFRARED_LINE_ACTIVE_HIGH 0U
#endif

/* 实测模块编号与车辆左右方向相反，归一化后仍按车辆从左到右排列。 */
#ifndef INFRARED_LINE_REVERSE_ORDER
#define INFRARED_LINE_REVERSE_ORDER 1U
#endif

void InfraredLineSensor_Init(void);
uint8_t InfraredLineSensor_ReadMask(void);

#endif /* INFRARED_LINE_SENSOR_H */
