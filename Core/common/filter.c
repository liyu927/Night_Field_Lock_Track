#include "filter.h"

uint16_t Filter_LowPass(LowPassFilter_t *filter, uint16_t input, uint8_t shift)
{
    int32_t difference;

    if (filter->initialized == 0U)
    {
        filter->value = input;
        filter->initialized = 1U;
        return input;
    }

    difference = (int32_t)input - (int32_t)filter->value;
    filter->value = (uint16_t)((int32_t)filter->value + (difference >> shift));
    return filter->value;
}

float Filter_Kalman(KalmanFilter_t *filter, float measurement)
{
    float gain;

    if (filter->initialized == 0U)
    {
        filter->estimate = measurement;
        filter->initialized = 1U;
        return measurement;
    }

    filter->covariance += filter->process_noise;
    gain = filter->covariance / (filter->covariance + filter->measurement_noise);
    filter->estimate += gain * (measurement - filter->estimate);
    filter->covariance *= (1.0f - gain);
    return filter->estimate;
}

uint16_t Filter_Median3(uint16_t sample1, uint16_t sample2, uint16_t sample3)
{
    uint16_t temporary;

    if (sample1 > sample2)
    {
        temporary = sample1;
        sample1 = sample2;
        sample2 = temporary;
    }
    if (sample2 > sample3)
    {
        temporary = sample2;
        sample2 = sample3;
        sample3 = temporary;
    }
    if (sample1 > sample2)
    {
        sample2 = sample1;
    }
    return sample2;
}
