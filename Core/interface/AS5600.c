#include "AS5600.h"

#define AS5600_TWO_PI               6.28318530717958647692f
#define AS5600_I2C_TIMEOUT_MS       2U

HAL_StatusTypeDef AS5600_ReadRawAngle(const AS5600_Handle_t *handle,
                                      uint16_t *raw_angle)
{
    uint8_t data[2];

    if ((handle == 0) || (handle->i2c == 0) || (raw_angle == 0))
    {
        return HAL_ERROR;
    }

    if (HAL_I2C_Mem_Read(handle->i2c,
                         (uint16_t)(handle->address << 1U),
                         AS5600_RAW_ANGLE_REGISTER,
                         I2C_MEMADD_SIZE_8BIT,
                         data,
                         sizeof(data),
                         AS5600_I2C_TIMEOUT_MS) != HAL_OK)
    {
        return HAL_ERROR;
    }

    *raw_angle = (uint16_t)((((uint16_t)data[0] << 8U) | data[1]) & 0x0FFFU);
    return HAL_OK;
}

HAL_StatusTypeDef AS5600_ReadMechanicalAngle(const AS5600_Handle_t *handle,
                                              float *angle_rad)
{
    uint16_t raw_angle;

    if (angle_rad == 0)
    {
        return HAL_ERROR;
    }

    if (AS5600_ReadRawAngle(handle, &raw_angle) != HAL_OK)
    {
        return HAL_ERROR;
    }

    *angle_rad = (float)raw_angle * AS5600_TWO_PI / AS5600_ANGLE_RESOLUTION;
    return HAL_OK;
}
