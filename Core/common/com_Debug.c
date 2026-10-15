#include "com_Debug.h"
#include "usart.h"

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
