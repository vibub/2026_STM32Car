#ifndef IMU_HEADING_H
#define IMU_HEADING_H

#include <stdint.h>

/* STM32 侧 IMU 接口尚未确定，先提供稳定的应用层边界。 */
void ImuHeading_Init(void);
void ImuHeading_Poll(void);
uint8_t ImuHeading_IsFresh(uint32_t now_ms);
float ImuHeading_GetDegrees(void);

#endif /* IMU_HEADING_H */
