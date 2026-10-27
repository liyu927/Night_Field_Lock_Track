#include "AS5600.h"

/* 机械一圈的弧度值；I2C 超时以毫秒计，防止控制任务长期阻塞。 */
#define AS5600_TWO_PI               6.28318530717958647692f
#define AS5600_I2C_TIMEOUT_MS       2U

/* 从 0x0E/0x0F 连续读取滤波后的 12 位 ANGLE，成功后才写输出参数。 */
HAL_StatusTypeDef AS5600_ReadFilteredAngle(const AS5600_Handle_t *handle,
                                           uint16_t *filtered_angle)
{
    uint8_t data[2];

    if ((handle == 0) || (handle->i2c == 0) || (filtered_angle == 0))
    {
        return HAL_ERROR;
    }

    /* HAL 接收 8 位地址格式，故 7 位器件地址左移；寄存器地址宽度为 8 位。 */
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

    /* 高、低字节合并后屏蔽高 4 位，只保留 0～4095 的有效计数。 */
    *filtered_angle = (uint16_t)((((uint16_t)data[0] << 8U) | data[1]) & 0x0FFFU);
    return HAL_OK;
}

/* 4096 个计数对应机械一圈；这里只给单圈角，跨圈累计由控制任务完成。 */
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

    /* 每计数为 2π/4096 rad；最大计数 4095，因此结果始终小于 2π。 */
    *angle_rad = (float)filtered_angle * AS5600_TWO_PI / AS5600_ANGLE_RESOLUTION;
    return HAL_OK;
}
