#ifndef AS5600_H
#define AS5600_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

/* AS5600 默认 7 位 I2C 地址；HAL 传输时需左移一位。 */
#define AS5600_I2C_ADDRESS          0x36U
/* 滤波后的 ANGLE 高字节地址；下一字节 0x0F 是低字节。 */
#define AS5600_ANGLE_REGISTER       0x0EU
/* 12 位计数值覆盖一圈，范围为 0～4095。 */
#define AS5600_ANGLE_RESOLUTION     4096.0f

/* 一个编码器句柄对应一条 I2C 总线；两轴地址相同但位于不同总线上。 */
typedef struct
{
    I2C_HandleTypeDef *i2c;  /* 对应轴的 HAL I2C 句柄，不由本驱动创建。 */
    uint16_t address;        /* 未左移的 7 位器件地址。 */
} AS5600_Handle_t;

/* 连续读取 ANGLE 高、低字节，输出 0～4095 的滤波角度计数。 */
/* handle 或输出指针无效、I2C 读取失败时返回 HAL_ERROR。 */
HAL_StatusTypeDef AS5600_ReadFilteredAngle(const AS5600_Handle_t *handle,
                                           uint16_t *filtered_angle);

/* 将滤波计数换算成 [0, 2π) 的单圈机械角，单位为弧度；不负责跨圈累计。 */
HAL_StatusTypeDef AS5600_ReadMechanicalAngle(const AS5600_Handle_t *handle,
                                              float *angle_rad);

#ifdef __cplusplus
}
#endif

#endif /* AS5600_H */
