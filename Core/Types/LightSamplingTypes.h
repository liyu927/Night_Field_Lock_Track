#ifndef LIGHT_SAMPLING_TYPES_H
#define LIGHT_SAMPLING_TYPES_H

#include <stdint.h>

typedef struct
{
    uint16_t left_top;
    uint16_t left_bottom;
    uint16_t right_top;
    uint16_t right_bottom;
} LightData_t;

#endif
