#ifndef FT6336_H
#define FT6336_H

#include "main.h"
#include <stdbool.h>

void FT6336_Init(void);
bool FT6336_IsTouched(void);
bool FT6336_Read(uint16_t *x, uint16_t *y);

#endif
