#include "AS5600.h"

#define AS5600_TWO_PI               6.28318530717958647692f
#define AS5600_I2C_TIMEOUT_MS       2U

/* 从 0x0E/0x0F 连续读取滤波后的 12 位 ANGLE，屏蔽高字节无效位。 */
HAL_StatusTypeDef AS5600_ReadFilteredAngle(const AS5600_Handle_t *handle,
                                           uint16_t *filtered_angle)
{
    uint8_t data[2];

    if ((handle == 0) || (handle->i2c == 0) || (filtered_angle == 0))
    {
        return HAL_ERROR;
    }

    if (HAL_I2C_Mem_Read(handle->i2c,
                         (uint16_t)(handle->address << 1U),
                         AS5600_ANGLE_REGISTER,
                         I2C_MEMADD_SIZE_8BIT,
                         data,
                         sizeof(data),
                         AS5600_I2C_TIMEOUT_MS) != HAL_OK)
    {
        return HAL_ERROR;
    }

    *filtered_angle = (uint16_t)((((uint16_t)data[0] << 8U) | data[1]) & 0x0FFFU);
    return HAL_OK;
}

/* 4096 个计数对应机械一圈，转换为 FOC 使用的弧度。 */
HAL_StatusTypeDef AS5600_ReadMechanicalAngle(const AS5600_Handle_t *handle,
                                              float *angle_rad)
{
    uint16_t filtered_angle;

    if (angle_rad == 0)
    {
        return HAL_ERROR;
    }

    if (AS5600_ReadFilteredAngle(handle, &filtered_angle) != HAL_OK)
    {
        return HAL_ERROR;
    }

    *angle_rad = (float)filtered_angle * AS5600_TWO_PI / AS5600_ANGLE_RESOLUTION;
    return HAL_OK;
}
