#ifndef LAP_STOP_DETECTOR_H
#define LAP_STOP_DETECTOR_H

#include <stdint.h>

typedef struct {
    float distance_cm;             /* 本次运行累计的车体中心行驶里程。 */
    float expected_distance_cm;    /* 编码器标定得到的名义整圈里程。 */
    uint8_t left_start_line;       /* 是否已连续确认车辆离开起点横线。 */
    uint8_t leave_clear_count;     /* 起点横线离开条件的连续有效样本数。 */
    uint8_t armed;                 /* 是否已达到返程 A 点识别里程门槛。 */
    uint8_t full_black_count;      /* 返程横线候选的连续有效样本数。 */
    uint8_t completed;             /* 是否已经确认返程通过 A 点。 */
} LapStopDetector;

void LapStopDetector_Init(LapStopDetector *detector);
void LapStopDetector_SetExpectedDistance(LapStopDetector *detector,
                                         float expected_distance_cm);
uint8_t LapStopDetector_Update(LapStopDetector *detector,
                               float distance_delta_cm,
                               uint8_t sensor_sample_valid,
                               uint8_t sensor_mask);

#endif /* LAP_STOP_DETECTOR_H */
