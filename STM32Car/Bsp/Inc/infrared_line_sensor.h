#ifndef INFRARED_LINE_SENSOR_H
#define INFRARED_LINE_SENSOR_H

#include <stdint.h>

#define INFRARED_LINE_X1_MASK 0x01U
#define INFRARED_LINE_X2_MASK 0x02U
#define INFRARED_LINE_X3_MASK 0x04U
#define INFRARED_LINE_X4_MASK 0x08U
#define INFRARED_LINE_ALL_MASK 0x0FU

/*
 * YH-4LXJ V1.1 暂未找到可靠的官方电气资料。
 * 初始按同类模块常见的“黑线输出高电平”处理，实测相反时改为 0U。
 */
#ifndef INFRARED_LINE_ACTIVE_HIGH
#define INFRARED_LINE_ACTIVE_HIGH 1U
#endif

/* X1～X4 默认按车辆从左到右排列，安装方向相反时改为 1U。 */
#ifndef INFRARED_LINE_REVERSE_ORDER
#define INFRARED_LINE_REVERSE_ORDER 0U
#endif

void InfraredLineSensor_Init(void);
uint8_t InfraredLineSensor_ReadMask(void);

#endif /* INFRARED_LINE_SENSOR_H */
