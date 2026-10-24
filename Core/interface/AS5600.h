#ifndef AS5600_H
#define AS5600_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

/* AS5600 的 7 位 I2C 地址；0x0E 为滤波后的 ANGLE 高字节。 */
#define AS5600_I2C_ADDRESS          0x36U
#define AS5600_ANGLE_REGISTER       0x0EU
#define AS5600_ANGLE_RESOLUTION     4096.0f

/* 一个 AS5600 实例对应一条 I2C 总线。 */
typedef struct
{
    I2C_HandleTypeDef *i2c;
    uint16_t address;
} AS5600_Handle_t;

/* 读取 0 到 4095 范围的 12 位滤波后角度。 */
HAL_StatusTypeDef AS5600_ReadFilteredAngle(const AS5600_Handle_t *handle,
                                           uint16_t *filtered_angle);

/* 读取 0 到 2π 范围内的机械角度，单位为弧度。 */
HAL_StatusTypeDef AS5600_ReadMechanicalAngle(const AS5600_Handle_t *handle,
                                              float *angle_rad);

#ifdef __cplusplus
}
#endif

#endif /* AS5600_H */
