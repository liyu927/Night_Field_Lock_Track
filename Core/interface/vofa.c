/* 引入 VOFA 接口声明和可调配置宏。 */
#include "vofa.h"

/* 引入 snprintf 函数，用于把浮点采样值转换为 FireWater 文本帧。 */
#include <stdio.h>

/* 引入 USART1 句柄 huart1 的外部声明。 */
#include "usart.h"

/* 将浮点数组按照 FireWater 多通道格式通过 USART1 发送。 */
HAL_StatusTypeDef VOFA_SendValues(const float *values, uint16_t count)
/* 开始函数实现。 */
{
    /* 创建局部字符缓冲区，用于保存本次待发送的完整文本帧。 */
    char frame[VOFA_TX_BUFFER_SIZE];
    /* 保存当前已经写入 frame 的有效字符数量。 */
    size_t length = 0U;
    /* 保存单次 snprintf 尝试写入的字符数量。 */
    int written;
    /* 保存当前正在格式化的波形通道下标。 */
    uint16_t index;

    /* 检查数组指针、通道数量以及配置的最大通道数量是否合法。 */
    if ((values == NULL) || (count == 0U) || (count > VOFA_MAX_CHANNEL_COUNT))
    /* 开始处理输入参数无效的分支。 */
    {
        /* 返回 HAL_ERROR，避免访问空指针或写出 frame 边界。 */
        return HAL_ERROR;
    /* 结束输入参数无效的处理分支。 */
    }

    /* 按数组顺序依次把每个采样值写入同一个 FireWater 数据帧。 */
    for (index = 0U; index < count; index++)
    /* 开始处理当前通道数据。 */
    {
        /* 将当前浮点数转换为 ASCII，通道之间使用逗号，最后一个通道使用换行符。 */
        written = snprintf(&frame[length],
                           /* 只允许 snprintf 使用 frame 中尚未占用的空间。 */
                           sizeof(frame) - length,
                           /* 使用可配置的小数精度，并通过字符参数选择逗号或换行符。 */
                           "%.*f%c",
                           /* 向星号精度占位符传入要保留的小数位数。 */
                           VOFA_FIREWATER_DECIMAL_PLACES,
                           /* 浮点可变参数会按 double 传递，显式转换使意图更清晰。 */
                           (double)values[index],
                           /* 最后一个通道结束整帧，其余通道使用逗号分隔。 */
                           ((index + 1U) == count) ? '\n' : ',');
        /* 检查格式化失败或当前通道文本超出剩余缓冲区的情况。 */
        if ((written < 0) || ((size_t)written >= (sizeof(frame) - length)))
        /* 开始处理文本帧生成失败的分支。 */
        {
            /* 返回 HAL_ERROR，避免发送无效或被截断的 FireWater 数据帧。 */
            return HAL_ERROR;
        /* 结束文本帧生成失败的处理分支。 */
        }

        /* 累加当前通道产生的有效字符数量，为下一个通道确定写入位置。 */
        length += (size_t)written;
    /* 结束当前通道数据处理。 */
    }

    /* 调用 STM32 HAL 阻塞发送函数，并把底层发送结果直接返回给调用者。 */
    return HAL_UART_Transmit(&huart1,
                             /* HAL 接口需要 uint8_t 指针，因此转换字符缓冲区指针类型。 */
                             (uint8_t *)frame,
                             /* 发送实际有效字符数量，不发送末尾的字符串结束符。 */
                             (uint16_t)length,
                             /* 使用可配置的串口发送超时时间，防止永久阻塞。 */
                             VOFA_UART_TIMEOUT_MS);
/* 结束 VOFA_SendValues 函数实现。 */
}
