#include "com_Debug.h"
#include "usart.h"

#if VOFA_DEBUG
#include "FreeRTOS.h"
#include "task.h"
#define VOFA_COMMAND_SLOTS  16U

static uint8_t vofa_rx_byte;
static char vofa_rx_line[VOFA_COMMAND_LENGTH];
static char vofa_commands[VOFA_COMMAND_SLOTS][VOFA_COMMAND_LENGTH];
static volatile uint8_t vofa_write_index;
static volatile uint8_t vofa_read_index;
static uint8_t vofa_rx_length;
static uint8_t vofa_rx_discard;

HAL_StatusTypeDef VOFA_StartReceive(void)
{
    /* 中断优先级允许以后在回调中使用 FreeRTOS API；当前回调只操作静态内存。 */
    HAL_NVIC_SetPriority(USART1_IRQn, 5U, 0U);
    HAL_NVIC_EnableIRQ(USART1_IRQn);
    return HAL_UART_Receive_IT(&huart1, &vofa_rx_byte, 1U);
}

uint8_t VOFA_GetCommand(char *command, uint16_t size)
{
    uint8_t index;

    if (command == NULL || size < VOFA_COMMAND_LENGTH) return 0U;
    taskENTER_CRITICAL();
    if (vofa_read_index == vofa_write_index)
    {
        taskEXIT_CRITICAL();
        return 0U;
    }
    index = vofa_read_index;
    memcpy(command, vofa_commands[index], VOFA_COMMAND_LENGTH);
    vofa_read_index = (uint8_t)((index + 1U) % VOFA_COMMAND_SLOTS);
    taskEXIT_CRITICAL();
    return 1U;
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *uart)
{
    uint8_t next;

    if (uart != &huart1) return;
    if (vofa_rx_byte == '\n' || vofa_rx_byte == '\r')
    {
        if (vofa_rx_length != 0U && vofa_rx_discard == 0U)
        {
            next = (uint8_t)((vofa_write_index + 1U) % VOFA_COMMAND_SLOTS);
            if (next != vofa_read_index)
            {
                vofa_rx_line[vofa_rx_length] = '\0';
                memcpy(vofa_commands[vofa_write_index], vofa_rx_line, vofa_rx_length + 1U);
                vofa_write_index = next;
            }
            vofa_rx_length = 0U;
        }
        vofa_rx_length = 0U;
        vofa_rx_discard = 0U;
    }
    else if (vofa_rx_discard != 0U)
    {
        /* 等待行尾，整条超长命令都不执行。 */
    }
    else if (vofa_rx_length < VOFA_COMMAND_LENGTH - 1U)
    {
        vofa_rx_line[vofa_rx_length++] = (char)vofa_rx_byte;
    }
    else
    {
        /* 超长命令丢弃，避免截断内容被误当成有效参数。 */
        vofa_rx_length = 0U;
        vofa_rx_discard = 1U;
    }
    (void)HAL_UART_Receive_IT(&huart1, &vofa_rx_byte, 1U);
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *uart)
{
    if (uart == &huart1)
    {
        /* 串口溢出后重新启动接收，后续完整命令仍可处理。 */
        vofa_rx_length = 0U;
        vofa_rx_discard = 0U;
        (void)HAL_UART_Receive_IT(&huart1, &vofa_rx_byte, 1U);
    }
}

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
