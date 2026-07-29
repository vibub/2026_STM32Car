#ifndef INFRARED_LINE_SENSOR_H
#define INFRARED_LINE_SENSOR_H

#include <stdint.h>

#define INFRARED_LINE_ALL_MASK 0xFFU

void InfraredLineSensor_Init(void);
uint8_t InfraredLineSensor_ReadMask(uint8_t *sensor_mask);
uint8_t InfraredLineSensor_GetRawValue(void);
uint32_t InfraredLineSensor_GetReadCount(void);
uint32_t InfraredLineSensor_GetErrorCount(void);

#endif /* INFRARED_LINE_SENSOR_H */
