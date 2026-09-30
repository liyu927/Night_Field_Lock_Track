#ifndef FILTER_H
#define FILTER_H

#include <stdint.h>

typedef struct
{
    uint16_t value;
    uint8_t initialized;
} LowPassFilter_t;

typedef struct
{
    float estimate;
    float covariance;
    float process_noise;
    float measurement_noise;
    uint8_t initialized;
} KalmanFilter_t;

uint16_t Filter_LowPass(LowPassFilter_t *filter, uint16_t input, uint8_t shift);
float Filter_Kalman(KalmanFilter_t *filter, float measurement);
uint16_t Filter_Median3(uint16_t sample1, uint16_t sample2, uint16_t sample3);

#endif
