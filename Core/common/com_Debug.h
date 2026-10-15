#ifndef COM_DEBUG_H
#define COM_DEBUG_H

#include <stdio.h>
#include <string.h>

// 日志输出打印在CPU运行上非常占用资源 => 通过比特率可以计算  打印10字节左右大概需要1ms
// 所以在后续飞机需要正常飞行的时候  需要关闭打印功能
// 设计一个日志输出打印开关
#ifndef DEBUG_LOG_ENABLE
#define DEBUG_LOG_ENABLE 1
#endif

#ifndef DEBUG_UART_TIMEOUT
#define DEBUG_UART_TIMEOUT 1000U
#endif

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
