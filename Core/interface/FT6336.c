#include "FT6336.h"
#include "app_system.h"
#include "i2c.h"
#include <stddef.h>

#define FT6336_I2C_ADDR          (0x38U << 1)
#define FT6336_REG_NUM_TOUCHES   0x02U
#define FT6336_REG_TOUCH_1       0x03U

#define FT6336_LCD_WIDTH         240U
#define FT6336_LCD_HEIGHT        320U

static uint8_t touch_count;                    /* 当前 I2C 轮询得到的有效触摸点数量。 */

/**
 * @brief 复位触摸控制器并清空轮询触摸状态。
 */
void FT6336_Init(void)
{
    HAL_GPIO_WritePin(CTP_RST_GPIO_Port, CTP_RST_Pin, GPIO_PIN_RESET);
    HAL_Delay(20);
    HAL_GPIO_WritePin(CTP_RST_GPIO_Port, CTP_RST_Pin, GPIO_PIN_SET);
    HAL_Delay(300);
    touch_count = 0;
}

/**
 * @brief 轮询 FT6336 的触摸点数量寄存器。
 * @note 此函数保持原有轮询机制，用于向 LVGL 提供实时触摸状态。
 */
bool FT6336_IsTouched(void)
{
    uint8_t count = 0;

    if (HAL_I2C_Mem_Read(&hi2c2, FT6336_I2C_ADDR, FT6336_REG_NUM_TOUCHES,
                         I2C_MEMADD_SIZE_8BIT, &count, 1, 100) != HAL_OK) {
        touch_count = 0;
        return false;
    }

    touch_count = count & 0x0FU;
    return touch_count > 0U && touch_count <= 5U;
}

/**
 * @brief 读取 FT6336 的第一个触摸点并转换为显示坐标。
 */
bool FT6336_Read(uint16_t *x, uint16_t *y)
{
    uint8_t buf[4];
    uint16_t raw_x;
    uint16_t raw_y;

    if (x == NULL || y == NULL || touch_count == 0U) {
        return false;
    }

    if (HAL_I2C_Mem_Read(&hi2c2, FT6336_I2C_ADDR, FT6336_REG_TOUCH_1,
                         I2C_MEMADD_SIZE_8BIT, buf, sizeof(buf), 100) != HAL_OK) {
        touch_count = 0;
        return false;
    }

    /* 00 为按下，10 为持续接触，抬起和无效事件不产生坐标。 */
    if ((buf[0] & 0xC0U) == 0x40U || (buf[0] & 0xC0U) == 0xC0U) {
        touch_count = 0;
        return false;
    }

    raw_x = (uint16_t)(((buf[0] & 0x0FU) << 8) | buf[1]);
    raw_y = (uint16_t)(((buf[2] & 0x0FU) << 8) | buf[3]);

    /* 保留原触摸坐标的 X 轴方向，避免更换控制器后触摸左右镜像。 */
    *x = raw_x >= FT6336_LCD_WIDTH ? 0U : FT6336_LCD_WIDTH - 1U - raw_x;
    *y = raw_y >= FT6336_LCD_HEIGHT ? FT6336_LCD_HEIGHT - 1U : raw_y;
    return true;
}

/**
 * @brief HAL 的 GPIO EXTI 回调，运行在 EXTI15_10 中断上下文。
 * @param GPIO_Pin 本次产生外部中断的 GPIO 引脚号。
 * @note 仅通知系统任务记录事件；中断内不访问 I2C、FreeRTOS 或 LVGL。
 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if (GPIO_Pin == CTP_INT_Pin)
    {
        /* 真正开背光和重置倒计时由系统任务处理。 */
        App_System_TouchInterrupt();
    }
}
