/* 防止头文件被重复包含，避免宏和函数声明重复定义。 */
#ifndef VOFA_H
/* 定义头文件保护宏，首次包含本文件时该宏尚未定义。 */
#define VOFA_H

/* 当本头文件被 C++ 源文件包含时，启用 C 语言链接规则。 */
#ifdef __cplusplus
/* 禁止 C++ 编译器修改函数名称，确保能够链接 C 文件中的实现。 */
extern "C" {
/* 结束 C++ 编译环境判断。 */
#endif

/* 引入 STM32 HAL 类型定义，以使用 HAL_StatusTypeDef 返回发送状态。 */
#include "main.h"

/* 设置 FireWater 浮点数的小数位数，默认保留六位小数。 */
#define VOFA_FIREWATER_DECIMAL_PLACES    6
/* 设置一次最多发送的波形通道数量，可根据实际观察需求调整。 */
#define VOFA_MAX_CHANNEL_COUNT           8U
/* 为每个浮点数预留符号、整数、小数和分隔符所需的最大字符数量。 */
#define VOFA_VALUE_TEXT_SIZE             48U
/* 根据最大通道数量计算完整 FireWater 帧缓冲区大小，并额外预留字符串结束符。 */
#define VOFA_TX_BUFFER_SIZE              ((VOFA_MAX_CHANNEL_COUNT * VOFA_VALUE_TEXT_SIZE) + 1U)
/* 设置 USART1 阻塞发送的最大等待时间，单位为毫秒。 */
#define VOFA_UART_TIMEOUT_MS             100U

/* 声明多通道采样值发送接口，数组顺序对应 VOFA+ 中的通道顺序。 */
HAL_StatusTypeDef VOFA_SendValues(const float *values, uint16_t count);

/* 当本头文件被 C++ 源文件包含时，结束 C 语言链接规则区域。 */
#ifdef __cplusplus
/* 与前面的 extern "C" 左花括号配对。 */
}
/* 结束 C++ 编译环境判断。 */
#endif

/* 结束 VOFA_H 头文件保护区域。 */
#endif /* VOFA_H */
