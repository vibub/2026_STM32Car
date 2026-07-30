#ifndef LAP_STOP_DETECTOR_H
#define LAP_STOP_DETECTOR_H

#include <stdint.h>

typedef struct {
    float distance_cm;
    uint8_t armed;
    uint8_t full_black_count;
    uint8_t completed;
} LapStopDetector;

void LapStopDetector_Init(LapStopDetector *detector);
uint8_t LapStopDetector_Update(LapStopDetector *detector,
                               float distance_delta_cm,
                               uint8_t sensor_sample_valid,
                               uint8_t sensor_mask);

#endif /* LAP_STOP_DETECTOR_H */
