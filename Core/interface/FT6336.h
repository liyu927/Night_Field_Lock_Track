#ifndef FT6336_H
#define FT6336_H

#include "main.h"
#include <stdbool.h>
#include <stdint.h>

/**
 * @brief 复位并初始化 FT6336 触摸控制器。
 */
void FT6336_Init(void);

/**
 * @brief 通过 I2C 轮询当前是否存在触摸点。
 * @return true 表示当前存在有效触摸点，false 表示无触摸或读取失败。
 */
bool FT6336_IsTouched(void);

/**
 * @brief 读取当前第一个触摸点的显示坐标。
 * @param x 返回触摸点的 X 坐标。
 * @param y 返回触摸点的 Y 坐标。
 * @return true 表示坐标读取成功，false 表示无触摸或读取失败。
 */
bool FT6336_Read(uint16_t *x, uint16_t *y);

#endif
