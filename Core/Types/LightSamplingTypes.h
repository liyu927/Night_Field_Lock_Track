#ifndef LIGHT_SAMPLING_TYPES_H
#define LIGHT_SAMPLING_TYPES_H

#include <stdint.h>

/* 四路光敏电阻经滤波后的光照数据，用于任务间队列传递。 */
typedef struct
{
    uint16_t left_top;       /* PA0：左上角光敏电阻的滤波 ADC 值。 */
    uint16_t left_bottom;    /* PA1：左下角光敏电阻的滤波 ADC 值。 */
    uint16_t right_top;      /* PA2：右上角光敏电阻的滤波 ADC 值。 */
    uint16_t right_bottom;   /* PA3：右下角光敏电阻的滤波 ADC 值。 */
} LightData_t;

#endif
