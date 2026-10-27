#include "app_system.h"
#include "cmsis_os2.h"

#define LIGHT_SAMPLE_PERIOD_MS 20U /* 光照任务周期，符合设计文档的 20 ms 发布周期。 */

/**
 * @brief 启动 ADC1 DMA，周期性滤波四路光照数据并写入消息队列。
 * @param argument CMSIS-RTOS 任务入口保留参数，本任务不使用。
 */
void StartSmaplingTask(void *argument)
{
    (void)argument; /* 明确该任务不使用入口参数，避免编译器产生未使用参数告警。 */
    App_Smapling_Init();

    for (;;)
    {
        App_Smapling_Run();
        osDelay(LIGHT_SAMPLE_PERIOD_MS); /* 让出 CPU，保持固定的采样与发布周期。 */
    }
}
