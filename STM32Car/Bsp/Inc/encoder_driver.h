#ifndef ENCODER_DRIVER_H
#define ENCODER_DRIVER_H

#include <stdint.h>

typedef struct {
    int16_t left_counts;
    int16_t right_counts;
    float left_cm_s;
    float right_cm_s;
    float sample_seconds;
} EncoderSample;

void EncoderDriver_Init(void);
void EncoderDriver_Sample(EncoderSample *sample, float sample_seconds);

#endif /* ENCODER_DRIVER_H */
