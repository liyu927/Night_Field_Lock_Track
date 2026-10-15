#ifndef COM_DEBUG_H
#define COM_DEBUG_H

#include <stdio.h>
#include <string.h>
#include "main.h"

// 日志输出打印在CPU运行上非常占用资源 => 通过比特率可以计算  打印10字节左右大概需要1ms
// 所以在后续飞机需要正常飞行的时候  需要关闭打印功能
// 设计一个日志输出打印开关
#ifndef DEBUG_LOG_ENABLE
#define DEBUG_LOG_ENABLE 1
#endif

#ifndef DEBUG_UART_TIMEOUT
#define DEBUG_UART_TIMEOUT 1000U
#endif

/* VOFA+ FireWater 波形输出总开关：1 为正常发送，0 为关闭输出。 */
#ifndef VOFA_DEBUG
#define VOFA_DEBUG 1
#endif

/* FireWater 数据格式配置。 */
#define VOFA_FIREWATER_DECIMAL_PLACES    6
#define VOFA_MAX_CHANNEL_COUNT           8U
#define VOFA_VALUE_TEXT_SIZE             48U
#define VOFA_TX_BUFFER_SIZE              ((VOFA_MAX_CHANNEL_COUNT * VOFA_VALUE_TEXT_SIZE) + 1U)
#define VOFA_UART_TIMEOUT_MS             100U

/* 将同一采样时刻的多通道数据按 FireWater 格式发送到 VOFA+。 */
HAL_StatusTypeDef VOFA_SendValues(const float *values, uint16_t count);

#if DEBUG_LOG_ENABLE

// 使用宏定义的方式 只打印文件名称 不打印路径名称
// strrchr()从后向前查找字符串中的字符
#define COM_DEBUG_FILE_NAME_BACKSLASH (strrchr(__FILE__, '\\') ? strrchr(__FILE__, '\\') + 1 : __FILE__)
#define COM_DEBUG_FILE_NAME (strrchr(COM_DEBUG_FILE_NAME_BACKSLASH, '/') ? strrchr(COM_DEBUG_FILE_NAME_BACKSLASH, '/') + 1 : COM_DEBUG_FILE_NAME_BACKSLASH)

// 使用宏定义的方式能实现打印日志之前 先添加文件名和行号
#define debug_printf(format, ...) printf("[%s:%d]  " format, COM_DEBUG_FILE_NAME, __LINE__, ##__VA_ARGS__)

#else
// 如果没有开启日志输出打印
#define debug_printf(...) ((void)0)
#endif

#endif /* COM_DEBUG_H */
