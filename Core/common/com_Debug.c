#include "com_Debug.h"
#include "usart.h"

#if VOFA_DEBUG
/* 将浮点数组按照 FireWater 多通道格式通过 USART1 发送。 */
HAL_StatusTypeDef VOFA_SendValues(const float *values, uint16_t count)
{
    char frame[VOFA_TX_BUFFER_SIZE];
    size_t length = 0U;
    int written;
    uint16_t index;

    if ((values == NULL) || (count == 0U) || (count > VOFA_MAX_CHANNEL_COUNT))
    {
        return HAL_ERROR;
    }

    for (index = 0U; index < count; index++)
    {
        written = snprintf(&frame[length],
                           sizeof(frame) - length,
                           "%.*f%c",
                           VOFA_FIREWATER_DECIMAL_PLACES,
                           (double)values[index],
                           ((index + 1U) == count) ? '\n' : ',');
        if ((written < 0) || ((size_t)written >= (sizeof(frame) - length)))
        {
            return HAL_ERROR;
        }

        length += (size_t)written;
    }

    return HAL_UART_Transmit(&huart1,
                             (uint8_t *)frame,
                             (uint16_t)length,
                             VOFA_UART_TIMEOUT_MS);
}
#else
/* 关闭 VOFA 调试时保留空实现，调用方无需修改。 */
HAL_StatusTypeDef VOFA_SendValues(const float *values, uint16_t count)
{
    (void)values;
    (void)count;
    return HAL_OK;
}
#endif

// GCC/newlib环境下将printf重定向到USART1
#ifdef __GNUC__
int _write(int file, char *ptr, int len)
{
    (void)file;
    HAL_UART_Transmit(&huart1, (uint8_t *)ptr, (uint16_t)len, DEBUG_UART_TIMEOUT);
    return len;
}
#endif

// Keil/IAR环境下将printf重定向到USART1
#if defined(__CC_ARM) || defined(__ICCARM__)
int fputc(int ch, FILE *f)
{
    (void)f;
    HAL_UART_Transmit(&huart1, (uint8_t *)&ch, 1U, DEBUG_UART_TIMEOUT);
    return ch;
}
#endif
